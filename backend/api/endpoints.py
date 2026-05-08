"""
endpoints.py — FastAPI Motor Control System  v3.0  (Combined Best-of-Both)
===========================================================================

SOURCE MERGE:
  v2_upload/endpoints.py — clean error handling, Pydantic validators,
                            DB layer integration, all existing routes
  our endpoints.py       — config management endpoints (GET/PATCH/DELETE
                            /motor/{id}/config, GET /config/bounds)

NEW IN v3.0 — Full fleet control suite (previously missing from all versions):
  POST /fleet/power_on   — power on all motors
  POST /fleet/start      — start all IDLE, non-faulted motors
  POST /fleet/stop       — gracefully stop all RUNNING motors
  POST /fleet/power_off  — power off all motors (clears faults, resets state)
  POST /fleet/emergency  — emergency stop entire fleet immediately

RESPONSE CONTRACT:
  success  -> { "status": "success", ... }
  failure  -> HTTP 4xx with { "detail": "<reason>" }
  error    -> HTTP 500 with { "detail": "Internal server error" }

HTTPException inside a try block is always re-raised before the generic except
clause, so a valid 400/404 is never swallowed as a 500.
"""

from __future__ import annotations

import logging
import os
from datetime import datetime
from typing import Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, validator

from backend.services.motor_service import (
    MotorFleetService, FAULT_CATALOGUE, ConnectionStatus,
)
from backend.core.cache_persistence import (
    init_cache_persistence, CachedMotorService, DatabaseLayer,
)
from backend.models.motor_model import DCMotor12V, SAFETY_BOUNDS, OPERATOR_READONLY

log = logging.getLogger(__name__)

app = FastAPI(
    title="Motor Control System",
    version="3.0.0",
    description="12V DC Brushed Motor Fleet Management API",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Singletons ────────────────────────────────────────────────────────────────
motor_service:  MotorFleetService  = MotorFleetService(timeout_sec=3.0)
cached_service: CachedMotorService = None
db_layer:       DatabaseLayer      = None


@app.on_event("startup")
async def startup():
    global cached_service, db_layer
    db_url = os.getenv("DATABASE_URL", "")
    cached_service, cache, db_layer = init_cache_persistence(motor_service, db_url)
    motor_service.start_monitoring()
    log.info("Startup complete | cache=%s | db=%s",
             cache.backend,
             "connected" if db_layer else "not configured")


@app.on_event("shutdown")
async def shutdown():
    motor_service.stop_monitoring()
    motor_service.fleet_shutdown()
    if cached_service:
        cached_service.stop()
    log.info("Shutdown complete")


# =============================================================================
# REQUEST SCHEMAS
# =============================================================================

class MotorCreate(BaseModel):
    motor_id: str = Field(..., min_length=1, max_length=50)

    @validator("motor_id")
    def valid_id(cls, v):
        if not all(c.isalnum() or c in "-_" for c in v):
            raise ValueError("motor_id must be alphanumeric with optional - or _")
        return v.upper()


class MotorUpdate(BaseModel):
    motor_id:    str   = Field(..., min_length=1)
    voltage:     float = Field(..., ge=0, le=20)
    current:     float = Field(..., ge=0, le=15)
    speed:       int   = Field(..., ge=0, le=3500)
    temperature: float = Field(..., ge=-40, le=120)


class ThresholdOverrides(BaseModel):
    """
    Operator threshold overrides. Only supply the fields you want to change.
    All values are range-validated server-side against SAFETY_BOUNDS.
    Motor constants (R, Km, Rth_ws etc.) are NOT configurable via API.

    All temperature values are SHELL temperature.
    The sensor is on the motor casing — not the winding.
    """
    temp_ambient_c:        Optional[float] = Field(None, description="Ambient environment temperature [C]")
    temp_warning_c:        Optional[float] = Field(None, description="Shell temp for OVERHEAT warning [C]")
    temp_critical_c:       Optional[float] = Field(None, description="Shell temp for THERMAL_RUNAWAY [C]")
    temp_absolute_max_c:   Optional[float] = Field(None, description="Absolute shell hard-limit [C]")
    temp_runaway_rate_c_s: Optional[float] = Field(None, description="Shell rise rate for THERMAL_RUNAWAY [C/s]")
    current_stall_a:       Optional[float] = Field(None, description="Current threshold for STALL detection [A]")
    current_high_a:        Optional[float] = Field(None, description="HIGH_CURRENT fault threshold [A]")
    voltage_low_v:         Optional[float] = Field(None, description="VOLTAGE_DROP threshold [V]")
    efficiency_low_pct:    Optional[float] = Field(None, description="LOW_EFFICIENCY threshold [%]")
    ripple_fault_pct:      Optional[float] = Field(None, description="BRUSH_WEAR ripple threshold [%]")
    ke_deviation_fault:    Optional[float] = Field(None, description="Ke deviation for WINDING_SHORT [fraction]")
    startup_grace_ms:      Optional[int]   = Field(None, description="Startup transient window [ms]")
    persist_severe:        Optional[int]   = Field(None, description="Cycles for SEVERE fault to activate")
    persist_warning:       Optional[int]   = Field(None, description="Cycles for WARNING fault to activate")


# =============================================================================
# MOTOR MANAGEMENT
# =============================================================================

@app.post("/motor/create", tags=["Motor Management"])
def create_motor(data: MotorCreate):
    """Register a new motor in the fleet."""
    try:
        motor = DCMotor12V(name=data.motor_id)
        motor_service.register_motor(data.motor_id, motor)
        if db_layer:
            db_layer.save_motor_state(data.motor_id, motor)
        log.info("Motor '%s' registered", data.motor_id)
        return {"status": "success", "motor_id": data.motor_id,
                "message": f"Motor '{data.motor_id}' registered"}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception:
        log.exception("create_motor unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.delete("/motor/{motor_id}", tags=["Motor Management"])
def delete_motor(motor_id: str):
    """Unregister a motor from the fleet."""
    try:
        motor_service.remove_motor(motor_id)
        log.info("Motor '%s' removed", motor_id)
        return {"status": "success", "message": f"Motor '{motor_id}' removed"}
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("delete_motor unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


# =============================================================================
# SINGLE-MOTOR CONTROL
# =============================================================================

@app.post("/motor/{motor_id}/power_on", tags=["Control"])
def power_on(motor_id: str):
    """Power on a single motor (executes immediately)."""
    try:
        result = motor_service.queue_command(motor_id, "POWER_ON")
        if db_layer:
            db_layer.log_command(motor_id, "POWER_ON", "EXECUTED")
        return result
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("power_on unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/motor/{motor_id}/start", tags=["Control"])
def start_motor(motor_id: str):
    """Queue START for a single motor. Motor must be IDLE and powered on."""
    try:
        result = motor_service.start_motor(motor_id)
        if result["status"] == "failed":
            if db_layer:
                db_layer.log_command(motor_id, "START", "FAILED")
            raise HTTPException(status_code=400, detail=result["error"])
        if db_layer:
            db_layer.log_command(motor_id, "START", "QUEUED")
        return result
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("start_motor unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/motor/{motor_id}/stop", tags=["Control"])
def stop_motor(motor_id: str):
    """Queue graceful STOP for a single motor. Motor must be RUNNING."""
    try:
        result = motor_service.stop_motor(motor_id)
        if result["status"] == "failed":
            if db_layer:
                db_layer.log_command(motor_id, "STOP", "FAILED")
            raise HTTPException(status_code=400, detail=result["error"])
        if db_layer:
            db_layer.log_command(motor_id, "STOP", "QUEUED")
        return result
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("stop_motor unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/motor/{motor_id}/power_off", tags=["Control"])
def power_off(motor_id: str):
    """Power off a single motor (executes immediately, clears faults)."""
    try:
        result = motor_service.queue_command(motor_id, "POWER_OFF")
        if db_layer:
            db_layer.log_command(motor_id, "POWER_OFF", "EXECUTED")
        return result
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("power_off unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/motor/{motor_id}/emergency", tags=["Control"])
def emergency_stop(motor_id: str):
    """Emergency stop a single motor — immediate, bypasses queue, clears all pending commands."""
    try:
        result = motor_service.emergency_stop(motor_id)
        if db_layer:
            db_layer.log_command(motor_id, "EMERGENCY_STOP", "EXECUTED")
        log.warning("EMERGENCY STOP: %s", motor_id)
        return result
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("emergency_stop unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


# =============================================================================
# FLEET CONTROL  (new in v3.0)
# =============================================================================

@app.post("/fleet/power_on", tags=["Fleet Control"])
def fleet_power_on():
    """
    Power on ALL registered motors simultaneously.

    Returns per-motor result so you know exactly what happened to each one.
    Safe to call even if some motors are already powered on (they are skipped).
    """
    try:
        result = motor_service.fleet_power_on()
        log.info("Fleet power_on: %d/%d succeeded",
                 result["succeeded"], result["total"])
        return result
    except Exception:
        log.exception("fleet_power_on unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/fleet/start", tags=["Fleet Control"])
def fleet_start():
    """
    Start ALL IDLE motors that are not in FAULT state.

    Motors that are already RUNNING, in FAULT, or have a connection timeout
    are skipped (not treated as errors). The response tells you exactly which
    motors were started, which were skipped, and why.

    Returns per-motor result with status: success | skipped | error.
    """
    try:
        result = motor_service.fleet_start()
        log.info("Fleet start: %d started, %d skipped, %d failed",
                 result["succeeded"], result["skipped"], result["failed"])
        return result
    except Exception:
        log.exception("fleet_start unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/fleet/stop", tags=["Fleet Control"])
def fleet_stop():
    """
    Gracefully stop ALL RUNNING motors.

    Motors that are not RUNNING (IDLE, FAULT, OFF) are skipped.
    STOP is queued — the ESP32 picks it up on its next command poll.
    For an immediate hard stop use POST /fleet/emergency instead.

    Returns per-motor result with status: success | skipped | error.
    """
    try:
        result = motor_service.fleet_stop()
        log.info("Fleet stop: %d stopped, %d skipped, %d failed",
                 result["succeeded"], result["skipped"], result["failed"])
        return result
    except Exception:
        log.exception("fleet_stop unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/fleet/power_off", tags=["Fleet Control"])
def fleet_power_off():
    """
    Power off ALL motors (clears faults, resets state to OFF).

    Unlike fleet/emergency, this is a clean shutdown — faults are cleared
    and motors can be restarted normally after power_on.
    All pending command queues are flushed.

    Returns per-motor result with status: success | error.
    """
    try:
        result = motor_service.fleet_power_off()
        log.info("Fleet power_off: %d/%d succeeded",
                 result["succeeded"], result["total"])
        return result
    except Exception:
        log.exception("fleet_power_off unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/fleet/emergency", tags=["Fleet Control"])
def fleet_emergency():
    """
    EMERGENCY STOP the entire fleet immediately.

    Bypasses all command queues. No grace period. All motors go to FAULT state.
    Restart requires individual power_on + start per motor (or fleet/power_on
    followed by fleet/start once the emergency condition is resolved).

    Use this when there is a safety threat affecting the whole installation.

    Returns per-motor result with status: success | error.
    """
    try:
        result = motor_service.fleet_emergency()
        log.critical("FLEET EMERGENCY STOP: %d/%d motors stopped",
                     result["succeeded"], result["total"])
        return result
    except Exception:
        log.exception("fleet_emergency unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


# =============================================================================
# ESP32 INTERFACE
# =============================================================================

@app.get("/motor/{motor_id}/command", tags=["ESP32"])
def get_next_command(motor_id: str):
    """ESP32 polls this endpoint to retrieve its next command."""
    try:
        return motor_service.get_next_command(motor_id)
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("get_next_command unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/update", tags=["ESP32"])
def update_motor(data: MotorUpdate):
    """
    Receive sensor readings from ESP32.
    Runs the full control cycle: validate -> history -> metrics -> state -> faults.
    """
    try:
        cached_service.update_motor_cached(
            motor_id    = data.motor_id,
            voltage     = data.voltage,
            current     = data.current,
            speed       = data.speed,
            temperature = data.temperature,
        )
        return {"status": "success", "message": "Sensor data accepted",
                "motor_id": data.motor_id}
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception:
        log.exception("update_motor unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


# =============================================================================
# STATUS
# =============================================================================

@app.get("/motor/{motor_id}", tags=["Status"])
def get_motor_status(motor_id: str):
    """Full status snapshot for one motor (cache-first, 5s TTL)."""
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")
        return motor_service.get_motor_status(motor_id)
    except HTTPException:
        raise
    except Exception:
        log.exception("get_motor_status unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/fleet/status", tags=["Status"])
def fleet_status():
    """Status snapshot for every registered motor."""
    try:
        return motor_service.get_all_status()
    except Exception:
        log.exception("fleet_status unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/fleet/connections", tags=["Status"])
def fleet_connections():
    """Connection status (CONNECTED / TIMEOUT / NEVER_CONNECTED) per motor."""
    try:
        return {"status": "success",
                "connections": motor_service.get_connection_map()}
    except Exception:
        log.exception("fleet_connections unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


# =============================================================================
# FAULT DIAGNOSIS
# =============================================================================

@app.get("/motor/{motor_id}/faults", tags=["Fault Diagnosis"])
def get_motor_faults(motor_id: str):
    """
    Full fault diagnosis for one motor.

    Returns per fault: name, severity, category, description, cause, effect,
    could_also_be, recommended action, auto_stop flag, and live sensor readings.
    Also returns highest_severity, safe_to_run flag, and immediate_action.
    """
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")
        return motor_service.get_motor_diagnosis(motor_id)
    except HTTPException:
        raise
    except Exception:
        log.exception("get_motor_faults unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/fleet/faults", tags=["Fault Diagnosis"])
def fleet_faults():
    """
    Fault diagnosis for the entire fleet.
    Returns per-motor diagnosis plus fleet-level rollup:
      total_active_faults, fleet_worst_severity, fleet_safe_to_run.
    """
    try:
        return motor_service.get_fleet_diagnosis()
    except Exception:
        log.exception("fleet_faults unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/fault/catalogue", tags=["Fault Diagnosis"])
def fault_catalogue():
    """Static reference: full detail for every fault type the system can detect."""
    return {
        "status":    "success",
        "count":     len(FAULT_CATALOGUE),
        "catalogue": FAULT_CATALOGUE,
    }


# =============================================================================
# CONFIGURATION  (operator-facing threshold management)
# =============================================================================

@app.get("/motor/{motor_id}/config", tags=["Configuration"])
def get_motor_config(motor_id: str):
    """
    Get current effective configuration for a motor.

    Returns:
      - schema_version
      - constants: motor hardware parameters (read-only — change in source code)
      - thresholds: current active values with default, modified flag, and bounds per field

    NOTE: all temperature thresholds are SHELL temperature.
    The sensor is on the motor casing, not the winding.
    """
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")
        return {"status": "success", "motor_id": motor_id,
                "config": motor_service.get_motor_config(motor_id)}
    except HTTPException:
        raise
    except Exception:
        log.exception("get_motor_config unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.patch("/motor/{motor_id}/config", tags=["Configuration"])
def update_motor_config(motor_id: str, overrides: ThresholdOverrides):
    """
    Apply operator threshold overrides to a motor at runtime.

    - Only threshold fields are configurable (not motor constants).
    - Values are range-validated against SAFETY_BOUNDS before applying.
    - Cross-field validation: temp_warning_c must remain below temp_critical_c.
    - Changes take effect immediately on the next sensor update cycle.

    Returns:
      - applied: fields accepted and applied
      - rejected: fields rejected, with reason
    """
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")

        supplied = {k: v for k, v in overrides.dict().items() if v is not None}
        if not supplied:
            raise HTTPException(status_code=400, detail="No threshold values supplied")

        result = motor_service.apply_motor_config(motor_id, supplied)

        if db_layer:
            db_layer.log_command(motor_id,
                                 f"CONFIG_UPDATE:{list(result.get('applied', {}).keys())}",
                                 "APPLIED")
        log.info("Config updated for %s: applied=%s rejected=%s",
                 motor_id,
                 list(result.get("applied", {}).keys()),
                 list(result.get("rejected", {}).keys()))
        return result

    except HTTPException:
        raise
    except Exception:
        log.exception("update_motor_config unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.delete("/motor/{motor_id}/config", tags=["Configuration"])
def reset_motor_config(motor_id: str):
    """
    Reset all operator threshold overrides to engineering defaults.
    Motor constants are unaffected.
    """
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")
        result = motor_service.reset_motor_config(motor_id)
        if db_layer:
            db_layer.log_command(motor_id, "CONFIG_RESET", "APPLIED")
        log.info("Config reset to defaults for %s", motor_id)
        return result
    except HTTPException:
        raise
    except Exception:
        log.exception("reset_motor_config unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/config/bounds", tags=["Configuration"])
def get_config_bounds():
    """
    Valid range for every operator-configurable threshold field.

    Use this in the frontend to:
      - Validate inputs before submitting PATCH /motor/{id}/config
      - Drive slider min/max or numeric input constraints
      - Show field descriptions in tooltips

    Fields not listed here are either motor constants (engineer-only)
    or physics-based gates that operators must not change.

    NOTE: all temperature bounds are SHELL temperature.
    """
    try:
        bounds = motor_service.get_config_bounds()
        return {
            "status": "success",
            "count":  len(bounds),
            "note":   "All temperature bounds are SHELL temperature. "
                      "Sensor is on motor casing, not winding.",
            "bounds": bounds,
        }
    except Exception:
        log.exception("get_config_bounds unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


# =============================================================================
# HISTORICAL DATA  (DB-backed)
# =============================================================================

@app.get("/motor/{motor_id}/history", tags=["History"])
def get_motor_history(motor_id: str, hours: int = 1):
    """Sensor reading history (requires DATABASE_URL to be configured)."""
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")
        history = cached_service.get_motor_history(motor_id, hours)
        return {"status": "success", "motor_id": motor_id,
                "hours": hours, "count": len(history), "readings": history}
    except HTTPException:
        raise
    except Exception:
        log.exception("get_motor_history unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/motor/{motor_id}/faults/history", tags=["History"])
def get_fault_history(motor_id: str):
    """Fault event history (requires DATABASE_URL to be configured)."""
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")
        faults = cached_service.get_fault_history(motor_id)
        return {"status": "success", "motor_id": motor_id,
                "count": len(faults), "faults": faults}
    except HTTPException:
        raise
    except Exception:
        log.exception("get_fault_history unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/motor/{motor_id}/commands/history", tags=["History"])
def get_command_history(motor_id: str):
    """Command audit trail (requires DATABASE_URL to be configured)."""
    try:
        if not motor_service.exists(motor_id):
            raise HTTPException(status_code=404,
                                detail=f"Motor '{motor_id}' not registered")
        commands = cached_service.get_command_history(motor_id)
        return {"status": "success", "motor_id": motor_id,
                "count": len(commands), "commands": commands}
    except HTTPException:
        raise
    except Exception:
        log.exception("get_command_history unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")


# =============================================================================
# SYSTEM
# =============================================================================

@app.get("/health", tags=["System"])
def health():
    """System health — reflects actual cache and DB status."""
    return {
        "status":             "healthy",
        "timestamp":          datetime.utcnow().isoformat(),
        "motors_registered":  len(motor_service.motors),
        "cache":    cached_service.cache.backend if cached_service else "not initialised",
        "database": "connected" if db_layer else "not configured",
    }


@app.get("/stats", tags=["System"])
def stats():
    """Fleet statistics summary."""
    try:
        wrappers = list(motor_service.motors.values())
        return {
            "status": "success",
            "motors": {
                "total":     len(wrappers),
                "running":   sum(1 for w in wrappers if w.motor.running),
                "idle":      sum(1 for w in wrappers if w.motor.state.value == "IDLE"),
                "fault":     sum(1 for w in wrappers if w.motor.state.value == "FAULT"),
                "off":       sum(1 for w in wrappers if w.motor.state.value == "OFF"),
                "connected": sum(1 for w in wrappers
                                 if w.connection_status == ConnectionStatus.CONNECTED),
            },
            "faults": {
                "active_total": sum(len(w.motor.faults) for w in wrappers),
                "by_type":      motor_service.get_fault_summary(),
            },
        }
    except Exception:
        log.exception("stats unexpected error")
        raise HTTPException(status_code=500, detail="Internal server error")
