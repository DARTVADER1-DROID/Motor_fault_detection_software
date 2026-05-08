"""
motor_service.py — Motor Fleet Service  v3.0  (Combined Best-of-Both)
======================================================================

SOURCE MERGE:
  v2_upload/motor_service.py  — cleaner FAULT_CATALOGUE, correct live_readings keys,
                                 solid CommandQueue, ConnectionStatus, MotorWrapper
  our motor_service.py        — config management layer (get/apply/reset/bounds),
                                 could_also_be field, winding_temp in live_readings

NEW IN v3.0:
  Fleet control suite — previously only fleet_power_on and fleet_shutdown existed
  and neither was exposed at any endpoint. Now complete:
    fleet_power_on()   — power on all registered motors
    fleet_start()      — start all IDLE, non-faulted motors
    fleet_stop()       — gracefully stop all RUNNING motors
    fleet_power_off()  — power off all motors
    fleet_emergency()  — emergency stop entire fleet immediately
  Each returns a per-motor result dict so callers know exactly what happened.

THREAD SAFETY:
  RLock (re-entrant) throughout. get_fleet_status calls get_motor_status
  while holding the same lock — a plain Lock would deadlock here.
"""

from __future__ import annotations

import threading
import time
from collections import defaultdict, deque
from datetime import datetime
from typing import Dict, List, Optional

from backend.models.motor_model import (
    DCMotor12V, MotorState, FaultType,
    MotorThresholds, SAFETY_BOUNDS, OPERATOR_READONLY,
)


# =============================================================================
# FAULT CATALOGUE
# Full diagnostic detail for every fault the model can raise.
# =============================================================================

FAULT_CATALOGUE: Dict[str, Dict] = {
    "STALL": {
        "severity":      "CRITICAL",
        "category":      "Mechanical",
        "description":   "Rotor locked — current > 4.5 A with speed < 80 RPM after startup grace.",
        "cause":         "Mechanical jam, seized bearing, sudden overload.",
        "effect":        "All electrical energy converts to winding heat (I^2 * R). "
                         "At 5 A stall: ~60 W. Permanent damage within 5-10 seconds.",
        "could_also_be": "Broken speed sensor reading 0 RPM while motor is running. "
                         "Distinguish: if current drops normally after stop -> sensor fault.",
        "action":        "Motor stopped automatically. Clear jam, inspect bearing. "
                         "Power cycle required before restart.",
        "auto_stop":     True,
    },
    "THERMAL_RUNAWAY": {
        "severity":      "CRITICAL",
        "category":      "Thermal",
        "description":   "Shell temp > 70 C AND rising > 1.5 C/s, OR shell exceeded 80 C absolute. "
                         "NOTE: sensor is on the shell — winding is hotter than the reading.",
        "cause":         "Sustained overload, blocked ventilation, high ambient temperature.",
        "effect":        "Winding insulation past Class-B limit (130 C). "
                         "Insulation breakdown, permanent winding damage, fire risk.",
        "could_also_be": "Blocked motor enclosure. High ambient environment (foundry, engine bay). "
                         "Temperature sensor fault reading high.",
        "action":        "Emergency stop. Minimum 30 min cool-down. "
                         "Inspect insulation before restart. Clear ventilation.",
        "auto_stop":     True,
    },
    "WINDING_SHORT": {
        "severity":      "CRITICAL",
        "category":      "Electrical",
        "description":   "Live Ke deviates > 25% from nameplate Km AND back-EMF is > 30% "
                         "below speed-predicted value AND current > 120% of nominal. "
                         "Indicates shorted coils reducing effective turns.",
        "cause":         "Insulation failure from heat cycling, moisture ingress, over-voltage transient.",
        "effect":        "Reduced torque, excess current, accelerating thermal damage.",
        "could_also_be": "Wrong Km constant configured — verify against motor datasheet. "
                         "Partial brush contact loss affecting commutation.",
        "action":        "Emergency stop. Do NOT restart. Motor requires rewinding or replacement.",
        "auto_stop":     True,
    },
    "OVERHEAT": {
        "severity":      "SEVERE",
        "category":      "Thermal",
        "description":   "Shell temperature in 55-80 C range without active thermal runaway. "
                         "Estimated winding temperature 70-100 C. "
                         "NOTE: sensor is on the shell — winding is always hotter.",
        "cause":         "Prolonged high load, poor ventilation, high ambient temperature, "
                         "duty cycle exceeding motor rating.",
        "effect":        "Accelerated insulation degradation above 80 C winding temperature.",
        "could_also_be": "High ambient installation — update temp_ambient_c in config. "
                         "Motor running above rated duty cycle.",
        "action":        "Reduce load or duty cycle. Improve ventilation. Clean motor casing.",
        "auto_stop":     False,
    },
    "HIGH_CURRENT": {
        "severity":      "SEVERE",
        "category":      "Electrical",
        "description":   "Sustained current > 5.5 A after startup grace. "
                         "Average over last 5 cycles also above threshold. "
                         "Motor operating beyond continuous current rating.",
        "cause":         "Mechanical overload, sudden load spike, supply voltage sag.",
        "effect":        "Brush erosion, commutator pitting, winding stress, accelerated wear.",
        "could_also_be": "VOLTAGE_DROP causing motor to draw more current to maintain torque. "
                         "Check voltage first. Current sensor drift reading high.",
        "action":        "Check load profile. Reduce duty cycle. Verify supply rating.",
        "auto_stop":     False,
    },
    "VOLTAGE_DROP": {
        "severity":      "SEVERE",
        "category":      "Electrical",
        "description":   "Supply voltage below 10.5 V while running (> 1.5 V drop from nominal). "
                         "VOLTAGE_DROP can cause HIGH_CURRENT and STALL as secondary effects. "
                         "Fix the supply before diagnosing other simultaneous faults.",
        "cause":         "Weak/undersized supply, long cable run, loose terminal, battery near discharge.",
        "effect":        "Speed instability, torque loss, positive-feedback current rise, potential stall.",
        "could_also_be": "Voltage drop in cable not supply — measure at motor terminals. "
                         "Check cable gauge and connector torque.",
        "action":        "Measure at motor terminals. Verify PSU current rating. "
                         "Inspect cable gauge and connectors.",
        "auto_stop":     False,
    },
    "LOW_EFFICIENCY": {
        "severity":      "WARNING",
        "category":      "Mechanical",
        "description":   "Efficiency below 50% at load > 1 A and speed > 900 RPM. "
                         "Not flagged at light load — that is normal DC motor physics.",
        "cause":         "Increased bearing friction, worn brushes, partial winding fault.",
        "effect":        "Increased power consumption and excess heat generation.",
        "could_also_be": "Wrong R constant in config inflating P_loss calculation. "
                         "Normal operation at partial load — verify load percentage.",
        "action":        "Lubricate bearings. Inspect brushes. "
                         "Verify motor constants R and Km are correctly configured.",
        "auto_stop":     False,
    },
    "BRUSH_WEAR": {
        "severity":      "WARNING",
        "category":      "Mechanical",
        "description":   "Current ripple > 30% during steady-state running (speed stable, I > 1 A). "
                         "Indicates poor brush-to-commutator contact.",
        "cause":         "Worn carbon brushes, dirty/pitted commutator, weak spring tension.",
        "effect":        "Arcing at commutation, accelerating commutator damage, torque fluctuation.",
        "could_also_be": "Electrical noise on current measurement line (shielding issue). "
                         "PWM switching interference from motor controller.",
        "action":        "Clean commutator with dry cloth. Measure brush length against minimum. "
                         "Replace brushes if worn. Check spring tension.",
        "auto_stop":     False,
    },
}

_SEVERITY_RANK = {"CRITICAL": 0, "SEVERE": 1, "WARNING": 2, "UNKNOWN": 3}


def diagnose_faults(faults: List[str], motor_status: Dict) -> Dict:
    """
    Build structured fault diagnosis from active fault names + motor status snapshot.
    Returns full detail per fault, highest severity, recommended action, safe_to_run flag.
    """
    if not faults:
        return {
            "active_fault_count": 0,
            "fault_count":        0,
            "faults":             [],
            "highest_severity":   None,
            "safe_to_run":        True,
            "immediate_action":   "None — motor is healthy.",
            "summary":            "No active faults.",
        }

    detailed = []
    for name in faults:
        info = FAULT_CATALOGUE.get(name, {
            "severity":      "UNKNOWN",
            "category":      "Unknown",
            "description":   f"Unrecognised fault: {name}",
            "cause":         "—",
            "effect":        "—",
            "could_also_be": "—",
            "action":        "Investigate immediately.",
            "auto_stop":     False,
        })
        detailed.append({
            "fault_type":    name,
            "severity":      info["severity"],
            "category":      info["category"],
            "description":   info["description"],
            "cause":         info["cause"],
            "effect":        info["effect"],
            "could_also_be": info.get("could_also_be", "—"),
            "action":        info["action"],
            "auto_stop":     info["auto_stop"],
            "live_readings": {
                "voltage_v":         motor_status.get("voltage_v"),
                "current_a":         motor_status.get("current_a"),
                "speed_rpm":         motor_status.get("speed_rpm"),
                "temperature_c":     motor_status.get("temperature_c"),
                "winding_temp_c":    motor_status.get("winding_temp_c"),
                "efficiency_pct":    motor_status.get("efficiency_pct"),
                "ke_deviation_pct":  motor_status.get("ke_deviation_pct"),
                "current_ripple_pct":motor_status.get("current_ripple_pct"),
            },
        })

    detailed.sort(key=lambda x: _SEVERITY_RANK.get(x["severity"], 99))
    highest = detailed[0]["severity"]
    safe    = highest != "CRITICAL"
    action  = detailed[0]["action"]
    summary = "{} fault(s): {}".format(
        len(faults),
        ", ".join(f"{d['fault']}[{d['severity']}]" for d in detailed)
    )

    return {
        "active_fault_count": len(faults),
        "fault_count":        len(faults),
        "faults":             detailed,
        "highest_severity":   highest,
        "safe_to_run":        safe,
        "immediate_action":   action,
        "summary":            summary,
    }


# =============================================================================
# CONNECTION STATUS
# =============================================================================

class ConnectionStatus:
    CONNECTED       = "CONNECTED"
    TIMEOUT         = "TIMEOUT"
    NEVER_CONNECTED = "NEVER_CONNECTED"


# =============================================================================
# COMMAND QUEUE  (deque + set, O(1), dedup, bounded depth=5)
# =============================================================================

class CommandQueue:
    MAX_DEPTH = 5

    def __init__(self):
        self._q    = deque()
        self._seen = set()
        self._lock = threading.Lock()

    def enqueue(self, command: str) -> Dict:
        with self._lock:
            if command in self._seen:
                return {"queued": False, "depth": len(self._q),
                        "reason": f"'{command}' already queued"}
            if len(self._q) >= self.MAX_DEPTH:
                dropped = self._q.popleft()
                self._seen.discard(dropped["command"])
            self._q.append({"command": command, "ts": time.time()})
            self._seen.add(command)
            return {"queued": True, "depth": len(self._q)}

    def dequeue(self) -> Optional[str]:
        with self._lock:
            if self._q:
                obj = self._q.popleft()
                self._seen.discard(obj["command"])
                return obj["command"]
            return None

    def depth(self) -> int:
        return len(self._q)

    def clear(self) -> int:
        with self._lock:
            n = len(self._q)
            self._q.clear()
            self._seen.clear()
            return n


# =============================================================================
# MOTOR WRAPPER
# =============================================================================

class MotorWrapper:
    def __init__(self, motor: DCMotor12V, timeout_sec: float):
        self.motor             = motor
        self.timeout_sec       = timeout_sec
        self.last_update_time  = time.time()
        self.connection_status = ConnectionStatus.NEVER_CONNECTED
        self.command_queue     = CommandQueue()

    def heartbeat(self):
        self.last_update_time  = time.time()
        self.connection_status = ConnectionStatus.CONNECTED

    def update_connection(self):
        """Only transitions to TIMEOUT if ESP32 has sent data at least once."""
        if self.connection_status == ConnectionStatus.NEVER_CONNECTED:
            return
        if time.time() - self.last_update_time > self.timeout_sec:
            self.connection_status = ConnectionStatus.TIMEOUT
        else:
            self.connection_status = ConnectionStatus.CONNECTED

    def queue_command(self, command: str) -> Dict:
        return self.command_queue.enqueue(command)

    def next_command(self) -> Optional[str]:
        return self.command_queue.dequeue()

    def queue_depth(self) -> int:
        return self.command_queue.depth()

    def flush_commands(self) -> int:
        return self.command_queue.clear()


# =============================================================================
# MOTOR FLEET SERVICE
# =============================================================================

class MotorFleetService:
    """
    Fleet supervisor.
    RLock (re-entrant) throughout — get_fleet_status calls get_motor_status
    while holding the same lock; a plain Lock would deadlock here.
    """

    def __init__(self,
                 timeout_sec: float = 3.0,
                 monitor_interval: float = 1.0,
                 cache=None):
        self.motors:          Dict[str, MotorWrapper] = {}
        self.timeout_sec      = timeout_sec
        self.monitor_interval = monitor_interval
        self.cache            = cache
        self.lock             = threading.RLock()
        self._monitor_running = False

    # ── Registration ──────────────────────────────────────────────────────────

    def register_motor(self, motor_id: str, motor: DCMotor12V):
        with self.lock:
            if motor_id in self.motors:
                raise ValueError(f"Motor '{motor_id}' already registered")
            self.motors[motor_id] = MotorWrapper(motor, self.timeout_sec)

    def remove_motor(self, motor_id: str):
        with self.lock:
            if motor_id not in self.motors:
                raise ValueError(f"Motor '{motor_id}' not registered")
            self.motors[motor_id].flush_commands()
            del self.motors[motor_id]
            if self.cache:
                self.cache.delete(f"motor_status:{motor_id}")

    def exists(self, motor_id: str) -> bool:
        with self.lock:
            return motor_id in self.motors

    def get_motor_ids(self) -> List[str]:
        with self.lock:
            return list(self.motors.keys())

    # ── Sensor ingestion ──────────────────────────────────────────────────────

    def ingest_sensor_data(self, motor_id: str, voltage: float,
                           current: float, speed: int, temperature: float):
        with self.lock:
            w = self._get(motor_id)
            w.heartbeat()
            w.motor.update(voltage, current, speed, temperature)
            if self.cache:
                self.cache.delete(f"motor_status:{motor_id}")

    def update_motor(self, motor_id: str, voltage: float,
                     current: float, speed: int, temperature: float):
        """Alias for ingest_sensor_data — backward compatibility."""
        self.ingest_sensor_data(motor_id, voltage, current, speed, temperature)

    # ── Single-motor commands ─────────────────────────────────────────────────

    def queue_command(self, motor_id: str, command: str) -> Dict:
        VALID = {"POWER_ON", "POWER_OFF", "START", "STOP", "EMERGENCY_STOP"}
        with self.lock:
            w = self._get(motor_id)
            if command not in VALID:
                raise ValueError(f"Invalid command '{command}'")

            if command == "POWER_ON":
                w.motor.power_on()
                if self.cache:
                    self.cache.delete(f"motor_status:{motor_id}")
                return {"status": "success", "command": command,
                        "message": "Motor powered ON"}

            if command == "POWER_OFF":
                w.motor.power_off()
                if self.cache:
                    self.cache.delete(f"motor_status:{motor_id}")
                return {"status": "success", "command": command,
                        "message": "Motor powered OFF"}

            res = w.queue_command(command)
            if self.cache and res["queued"]:
                self.cache.delete(f"motor_status:{motor_id}")
            return {
                "status":      "success",
                "command":     command,
                "queued":      res["queued"],
                "queue_depth": res["depth"],
                "message":     "Command queued" if res["queued"] else "Command already in queue",
            }

    def start_motor(self, motor_id: str) -> Dict:
        with self.lock:
            w = self._get(motor_id)
            m = w.motor
            if w.connection_status == ConnectionStatus.TIMEOUT:
                return {"status": "failed",
                        "error":  "ESP32 connection timed out — no recent sensor data"}
            if w.connection_status == ConnectionStatus.NEVER_CONNECTED:
                return {"status": "failed",
                        "error":  "ESP32 has never connected — ensure ESP32 is running and sending sensor data"}
            if m.state == MotorState.FAULT:
                return {"status": "failed",
                        "error":  "Motor in FAULT state — resolve faults before starting"}
            if m.state != MotorState.IDLE:
                return {"status": "failed",
                        "error":  f"Cannot start: motor is {m.state.value} (must be IDLE)"}
            res = w.queue_command("START")
            return {"status": "success", "message": "START queued",
                    "queue_depth": res["depth"]}

    def stop_motor(self, motor_id: str) -> Dict:
        with self.lock:
            w = self._get(motor_id)
            m = w.motor
            if m.state != MotorState.RUNNING and not m.running:
                return {"status": "failed",
                        "error":  f"Cannot stop: motor is {m.state.value} (must be RUNNING)"}
            res = w.queue_command("STOP")
            return {"status": "success", "message": "STOP queued",
                    "queue_depth": res["depth"]}

    def emergency_stop(self, motor_id: str) -> Dict:
        with self.lock:
            w = self._get(motor_id)
            w.flush_commands()
            w.motor.emergency_stop()
            if self.cache:
                self.cache.delete(f"motor_status:{motor_id}")
            return {"status": "success", "command": "EMERGENCY_STOP",
                    "message": "Emergency stop executed — motor is OFF"}

    def get_next_command(self, motor_id: str) -> Dict:
        with self.lock:
            w   = self._get(motor_id)
            cmd = w.next_command()
            return {
                "status":      "success" if cmd else "no_command",
                "command":     cmd,
                "timestamp":   time.time(),
                "queue_depth": w.queue_depth(),
            }

    # ── Status ────────────────────────────────────────────────────────────────

    def get_motor_status(self, motor_id: str) -> Dict:
        """Cache-first. RLock safe to call while lock is held."""
        cache_key = f"motor_status:{motor_id}"
        if self.cache:
            hit = self.cache.get(cache_key)
            if hit:
                return hit

        with self.lock:
            w    = self._get(motor_id)
            data = w.motor.status()
            data["connection_status"] = w.connection_status
            data["last_update_sec"]   = round(time.time() - w.last_update_time, 2)
            data["queued_commands"]   = w.queue_depth()

        if self.cache:
            self.cache.set(cache_key, data, ttl=5)
        return data

    def get_all_status(self) -> Dict[str, Dict]:
        with self.lock:
            ids = list(self.motors.keys())
        return {mid: self.get_motor_status(mid) for mid in ids}

    def get_fleet_status(self) -> Dict[str, Dict]:
        return self.get_all_status()

    def get_connection_map(self) -> Dict[str, str]:
        with self.lock:
            return {mid: w.connection_status for mid, w in self.motors.items()}

    def get_fault_summary(self) -> Dict:
        summary = defaultdict(int)
        with self.lock:
            for w in self.motors.values():
                for f in w.motor.faults:
                    summary[f.value] += 1
        return dict(summary)

    # ── Fault diagnosis ───────────────────────────────────────────────────────

    def get_motor_diagnosis(self, motor_id: str) -> Dict:
        status = self.get_motor_status(motor_id)
        diag   = diagnose_faults(status.get("faults", []), status)
        diag["motor_id"]  = motor_id
        diag["state"]     = status.get("state")
        diag["timestamp"] = datetime.utcnow().isoformat()
        return diag

    def get_fleet_diagnosis(self) -> Dict:
        with self.lock:
            ids = list(self.motors.keys())

        results      = {}
        total_faults = 0
        worst        = None

        for mid in ids:
            d             = self.get_motor_diagnosis(mid)
            results[mid]  = d
            total_faults += d["fault_count"]
            sev           = d["highest_severity"]
            if sev and (worst is None or
                        _SEVERITY_RANK.get(sev, 99) < _SEVERITY_RANK.get(worst, 99)):
                worst = sev

        return {
            "timestamp":            datetime.utcnow().isoformat(),
            "motors_checked":       len(ids),
            "total_active_faults":  total_faults,
            "fleet_worst_severity": worst,
            "fleet_safe_to_run":    worst not in ("CRITICAL",),
            "motors":               results,
        }

    # ── Configuration management ──────────────────────────────────────────────

    def get_motor_config(self, motor_id: str) -> Dict:
        """Return full config snapshot (constants + thresholds with modified flags)."""
        with self.lock:
            w = self._get(motor_id)
            return {
                "schema_version": w.motor._th.schema_version,
                "constants":      {k: v for k, v in vars(w.motor._c).items()
                                   if not k.startswith("_")},
                "thresholds":     w.motor.get_threshold_config(),
            }

    def apply_motor_config(self, motor_id: str, overrides: Dict) -> Dict:
        """
        Apply validated operator threshold overrides.
        Includes cross-field validation (warning < critical).
        Motor constants are never accessible here.
        """
        # Cross-field guard: temp_warning must remain below temp_critical
        tw = overrides.get("temp_warning_c")
        tc = overrides.get("temp_critical_c")
        if tw is not None and tc is not None and float(tw) >= float(tc):
            return {
                "status":   "rejected",
                "applied":  {},
                "rejected": {
                    "temp_warning_c": f"Must be below temp_critical_c ({tc})"
                },
            }

        with self.lock:
            w      = self._get(motor_id)
            result = w.motor.apply_threshold_overrides(overrides)
            if self.cache:
                self.cache.delete(f"motor_status:{motor_id}")

        result["status"]   = "success" if result["applied"] else "no_changes"
        result["motor_id"] = motor_id
        return result

    def reset_motor_config(self, motor_id: str) -> Dict:
        """Reset all operator overrides — revert to engineering defaults."""
        with self.lock:
            w = self._get(motor_id)
            w.motor.reset_thresholds_to_defaults()
            if self.cache:
                self.cache.delete(f"motor_status:{motor_id}")
        return {"status": "success", "motor_id": motor_id,
                "message": "All thresholds reset to engineering defaults"}

    def get_config_bounds(self) -> Dict:
        """Return validation bounds for every operator-configurable threshold field."""
        return {
            k: {"min": lo, "max": hi}
            for k, (lo, hi) in SAFETY_BOUNDS.items()
            if k not in OPERATOR_READONLY
        }

    # ── Fleet-wide control (NEW in v3.0) ──────────────────────────────────────

    def fleet_power_on(self) -> Dict:
        """
        Power on all registered motors.
        Returns per-motor result so caller knows exactly what happened.
        """
        results = {}
        with self.lock:
            ids = list(self.motors.keys())

        for mid in ids:
            try:
                with self.lock:
                    w = self._get(mid)
                    prev_state = w.motor.state.value
                    w.motor.power_on()
                    new_state = w.motor.state.value
                    if self.cache:
                        self.cache.delete(f"motor_status:{mid}")
                results[mid] = {
                    "status":     "success",
                    "prev_state": prev_state,
                    "new_state":  new_state,
                }
            except Exception as e:
                results[mid] = {"status": "error", "error": str(e)}

        succeeded = sum(1 for r in results.values() if r["status"] == "success")
        return {
            "operation":   "fleet_power_on",
            "total":       len(ids),
            "succeeded":   succeeded,
            "failed":      len(ids) - succeeded,
            "motors":      results,
            "timestamp":   datetime.utcnow().isoformat(),
        }

    def fleet_start(self) -> Dict:
        """
        Start all motors that are IDLE and not in FAULT.
        Skips motors that are already RUNNING, in FAULT, or have connection timeout.
        Returns per-motor result.
        """
        results = {}
        with self.lock:
            ids = list(self.motors.keys())

        for mid in ids:
            try:
                with self.lock:
                    w = self._get(mid)
                    m = w.motor

                    if w.connection_status == ConnectionStatus.TIMEOUT:
                        results[mid] = {"status": "skipped",
                                        "reason": "ESP32 connection timeout"}
                        continue
                    if m.state == MotorState.FAULT:
                        results[mid] = {"status": "skipped",
                                        "reason": "Motor in FAULT state"}
                        continue
                    if m.state == MotorState.RUNNING:
                        results[mid] = {"status": "skipped",
                                        "reason": "Already RUNNING"}
                        continue
                    if m.state != MotorState.IDLE:
                        results[mid] = {"status": "skipped",
                                        "reason": f"Motor is {m.state.value} (not IDLE)"}
                        continue

                    # Queue START for the ESP32 to pick up on its next command poll.
                    # Do NOT call m.start() directly — the relay is controlled by the
                    # ESP32 hardware, not the backend model. The state transitions to
                    # RUNNING when the ESP32 executes the command and sensor data flows.
                    res = w.queue_command("START")
                    if self.cache:
                        self.cache.delete(f"motor_status:{mid}")
                    results[mid] = {"status": "success",
                                    "message": "START queued for ESP32",
                                    "queued": res["queued"],
                                    "queue_depth": res["depth"]}
            except Exception as e:
                results[mid] = {"status": "error", "error": str(e)}

        succeeded = sum(1 for r in results.values() if r["status"] == "success")
        skipped   = sum(1 for r in results.values() if r["status"] == "skipped")
        return {
            "operation": "fleet_start",
            "total":     len(ids),
            "succeeded": succeeded,
            "skipped":   skipped,
            "failed":    len(ids) - succeeded - skipped,
            "motors":    results,
            "timestamp": datetime.utcnow().isoformat(),
        }

    def fleet_stop(self) -> Dict:
        """
        Gracefully stop all RUNNING motors.
        Skips motors that are not RUNNING.
        Returns per-motor result.
        """
        results = {}
        with self.lock:
            ids = list(self.motors.keys())

        for mid in ids:
            try:
                with self.lock:
                    w = self._get(mid)
                    m = w.motor

                    if m.state != MotorState.RUNNING and not m.running:
                        results[mid] = {"status": "skipped",
                                        "reason": f"Motor is {m.state.value} (not RUNNING)"}
                        continue

                    # Queue STOP for the ESP32 to pick up on its next command poll.
                    res = w.queue_command("STOP")
                    if self.cache:
                        self.cache.delete(f"motor_status:{mid}")
                    results[mid] = {"status": "success",
                                    "message": "STOP queued for ESP32",
                                    "queued": res["queued"],
                                    "queue_depth": res["depth"]}
            except Exception as e:
                results[mid] = {"status": "error", "error": str(e)}

        succeeded = sum(1 for r in results.values() if r["status"] == "success")
        skipped   = sum(1 for r in results.values() if r["status"] == "skipped")
        return {
            "operation": "fleet_stop",
            "total":     len(ids),
            "succeeded": succeeded,
            "skipped":   skipped,
            "failed":    len(ids) - succeeded - skipped,
            "motors":    results,
            "timestamp": datetime.utcnow().isoformat(),
        }

    def fleet_power_off(self) -> Dict:
        """
        Power off all motors gracefully (clears faults, resets state).
        Returns per-motor result.
        """
        results = {}
        with self.lock:
            ids = list(self.motors.keys())

        for mid in ids:
            try:
                with self.lock:
                    w = self._get(mid)
                    prev_state = w.motor.state.value
                    w.motor.power_off()
                    w.flush_commands()
                    if self.cache:
                        self.cache.delete(f"motor_status:{mid}")
                results[mid] = {"status": "success",
                                "prev_state": prev_state,
                                "new_state":  "OFF"}
            except Exception as e:
                results[mid] = {"status": "error", "error": str(e)}

        succeeded = sum(1 for r in results.values() if r["status"] == "success")
        return {
            "operation": "fleet_power_off",
            "total":     len(ids),
            "succeeded": succeeded,
            "failed":    len(ids) - succeeded,
            "motors":    results,
            "timestamp": datetime.utcnow().isoformat(),
        }

    def fleet_emergency(self) -> Dict:
        """
        Emergency stop the entire fleet immediately.
        Flushes all command queues. No grace period.
        Returns per-motor result.
        """
        results = {}
        with self.lock:
            ids = list(self.motors.keys())

        for mid in ids:
            try:
                with self.lock:
                    w = self._get(mid)
                    prev_state = w.motor.state.value
                    w.flush_commands()
                    w.motor.emergency_stop()
                    if self.cache:
                        self.cache.delete(f"motor_status:{mid}")
                results[mid] = {"status": "success",
                                "prev_state": prev_state,
                                "new_state":  "FAULT"}
            except Exception as e:
                results[mid] = {"status": "error", "error": str(e)}

        succeeded = sum(1 for r in results.values() if r["status"] == "success")
        return {
            "operation": "fleet_emergency",
            "total":     len(ids),
            "succeeded": succeeded,
            "failed":    len(ids) - succeeded,
            "motors":    results,
            "timestamp": datetime.utcnow().isoformat(),
        }

    def fleet_shutdown(self):
        """Internal shutdown hook (called on server exit). Uses fleet_emergency."""
        self.fleet_emergency()

    # ── Monitor loop ──────────────────────────────────────────────────────────

    def start_monitoring(self):
        if self._monitor_running:
            return
        self._monitor_running = True
        t = threading.Thread(target=self._monitor_loop,
                             daemon=True, name="MotorMonitor")
        t.start()

    def stop_monitoring(self):
        self._monitor_running = False

    def _monitor_loop(self):
        while self._monitor_running:
            try:
                with self.lock:
                    for w in self.motors.values():
                        w.update_connection()
            except Exception:
                pass   # monitor thread must never die
            time.sleep(self.monitor_interval)

    # ── Internal ──────────────────────────────────────────────────────────────

    def _get(self, motor_id: str) -> MotorWrapper:
        """Must be called with self.lock held."""
        if motor_id not in self.motors:
            raise ValueError(f"Motor '{motor_id}' not registered")
        return self.motors[motor_id]
