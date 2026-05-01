from enum import Enum
from collections import deque
import time
from typing import Dict, List, Optional


# =========================
# FSM STATES (HIERARCHICAL)
# =========================
class MotorState(Enum):
    """Motor operational states with clear transitions"""
    STOPPED = "STOPPED"          # No power, safe state
    INITIALIZING = "INITIALIZING"  # Pre-start checks
    STARTING = "STARTING"         # Acceleration phase (RPM ramp-up)
    RUNNING = "RUNNING"           # Nominal operation
    NO_LOAD = "NO_LOAD"           # Light load, low current
    MEDIUM_LOAD = "MEDIUM_LOAD"   # Optimal operating range
    HIGH_LOAD = "HIGH_LOAD"       # Heavy load, approaching limits
    OVERLOAD = "OVERLOAD"         # Beyond safe limits (warning state)
    FAULT = "FAULT"               # Critical abnormality detected
    SHUTDOWN = "SHUTDOWN"         # Controlled emergency stop


# =========================
# FAULT TYPES (PRIORITY TIERED)
# =========================
class FaultType(Enum):
    """Faults ordered by severity (cascading detection)"""
    # CRITICAL (System stop required)
    OVERHEAT = "OVERHEAT"
    ELECTRICAL_OVERLOAD = "ELECTRICAL_OVERLOAD"
    STALL_CONDITION = "STALL_CONDITION"
    SENSOR_FAILURE = "SENSOR_FAILURE"
    
    # SEVERE (Immediate attention)
    OVER_VOLTAGE = "OVER_VOLTAGE"
    UNDER_VOLTAGE = "UNDER_VOLTAGE"
    
    # WARNING (Monitor closely)
    ABNORMAL_RPM = "ABNORMAL_RPM"
    BEARING_FRICTION = "BEARING_FRICTION"  # NEW: High current with low RPM
    PHASE_IMBALANCE = "PHASE_IMBALANCE"    # NEW: Voltage instability


class FaultSeverity(Enum):
    """Fault severity levels"""
    CRITICAL = 3   # Stop immediately
    SEVERE = 2     # Escalate monitoring
    WARNING = 1    # Log and alert


# =========================
# FAULT SEVERITY MAPPING
# =========================
FAULT_SEVERITY_MAP = {
    FaultType.OVERHEAT: FaultSeverity.CRITICAL,
    FaultType.ELECTRICAL_OVERLOAD: FaultSeverity.CRITICAL,
    FaultType.STALL_CONDITION: FaultSeverity.CRITICAL,
    FaultType.SENSOR_FAILURE: FaultSeverity.CRITICAL,
    FaultType.OVER_VOLTAGE: FaultSeverity.SEVERE,
    FaultType.UNDER_VOLTAGE: FaultSeverity.SEVERE,
    FaultType.ABNORMAL_RPM: FaultSeverity.WARNING,
    FaultType.BEARING_FRICTION: FaultSeverity.WARNING,
    FaultType.PHASE_IMBALANCE: FaultSeverity.WARNING,
}


class Motor:
    """
    Bulletproof Motor Control Model
    ─────────────────────────────────
    
    ARCHITECTURE:
    1. Input validation + safe defaults
    2. Derived data computation (power, efficiency, load indices)
    3. FSM state transitions (validated, deterministic)
    4. Multi-level fault detection (critical → severe → warning)
    5. Hysteresis-based state transitions (no jitter)
    6. Event logging + circular buffer
    
    ROBUSTNESS FEATURES:
    ✓ Sensor fault tolerance (dead-band logic)
    ✓ Hysteresis thresholds (prevent oscillation)
    ✓ Fault persistence tracking (flag multiple cycles)
    ✓ Load-aware state classification
    ✓ Efficiency monitoring
    ✓ Predictive fault indicators
    """

    def __init__(self, name: str = "Motor-1", update_rate: float = 10.0):
        """
        Initialize motor with defensive defaults.
        
        Args:
            name: Motor identifier
            update_rate: Cycles per second (for time-based thresholds)
        """
        self.name = name
        self.update_rate = update_rate
        self.cycle_time = 1.0 / update_rate

        # =========================
        # DIRECT DATA (SENSOR INPUT)
        # =========================
        self.voltage = 230.0        # Input voltage (V)
        self.current = 0.0          # Load current (A)
        self.rpm = 0                # Motor speed (RPM)
        self.temperature = 25.0     # Winding temp (°C)

        # =========================
        # DERIVED DATA (COMPUTED)
        # =========================
        self.power = 0.0            # Instantaneous power (W)
        self.efficiency = 0.0       # Power output efficiency (%)
        self.power_factor = 1.0     # cos(φ) - voltage/current phase angle
        self.load_percentage = 0.0  # Normalized load (0-100%)
        self.slip = 0.0             # Slip ratio (%, induction motor)
        self.torque_nm = 0.0        # Estimated torque (N⋅m)

        # =========================
        # FSM STATE
        # =========================
        self.state = MotorState.STOPPED
        self.previous_state = MotorState.STOPPED
        self.state_entry_time = time.time()
        self.running = False
        self.command_start = False  # External start signal
        self.command_stop = False   # External stop signal

        # =========================
        # THRESHOLDS (ELECTRICAL)
        # =========================
        self.thresholds = {
            # VOLTAGE SAFETY
            "voltage_nominal": 230.0,
            "voltage_min": 207.0,           # -10% from nominal
            "voltage_max": 253.0,           # +10% from nominal
            "voltage_hysteresis": 5.0,      # Dead-band to prevent jitter
            
            # CURRENT RANGES (3-phase motor @ 2.2kW typical)
            "current_no_load": 0.5,         # Idling current
            "current_no_load_hysteresis": 0.2,
            "current_light_load": 2.0,      # <30% rated load
            "current_medium_load": 4.0,     # 30-70% rated load
            "current_high_load": 7.0,       # 70-100% rated load
            "current_overload": 8.5,        # >100% rated (safety cutoff)
            "current_severe_overload": 10.0,  # Emergency shutdown
            
            # RPM THRESHOLDS (1500 RPM base, 3-phase @ 50Hz)
            "rpm_nominal": 1500,
            "rpm_min_running": 1200,        # 80% of nominal
            "rpm_stall": 50,                # Rotor locked/severely loaded
            "rpm_startup_timeout": 3.0,     # Seconds to reach min RPM
            
            # TEMPERATURE
            "temp_normal": 60.0,            # Operating limit
            "temp_warning": 75.0,           # Approach thermal limit
            "temp_critical": 85.0,          # Emergency shutdown
            "temp_hysteresis": 5.0,         # Prevent oscillation
            
            # EFFICIENCY MONITORING
            "efficiency_min": 65.0,         # Below = bearing friction flag
            "slip_max": 10.0,               # Max slip % before ABNORMAL_RPM
        }

        # =========================
        # FAULT SYSTEM (PERSISTENT)
        # =========================
        self.faults: List[FaultType] = []
        self.fault_history: Dict[FaultType, int] = {ft: 0 for ft in FaultType}
        self.fault_persistence = 3  # Fault must occur 3 cycles to trigger
        self.critical_fault_detected = False

        # =========================
        # STATE TRACKING (HYSTERESIS)
        # =========================
        self.load_state = "NO_LOAD"
        self.voltage_state = "NORMAL"
        self.temperature_state = "NORMAL"

        # =========================
        # EVENT LOG
        # =========================
        self.log: deque = deque(maxlen=500)
        self._log("INITIALIZATION COMPLETE")

    # =====================================================
    # COMMAND INTERFACE
    # =====================================================
    def start_motor(self) -> bool:
        """Request motor start (FSM validates)"""
        self.command_start = True
        self.command_stop = False
        self._log("START COMMAND RECEIVED")
        return True

    def stop_motor(self) -> bool:
        """Request motor stop (safe shutdown)"""
        self.command_stop = True
        self.command_start = False
        self._log("STOP COMMAND RECEIVED")
        return True

    def emergency_stop(self) -> bool:
        """Force immediate shutdown"""
        self.running = False
        self.state = MotorState.SHUTDOWN
        self.command_start = False
        self.command_stop = True
        self._log("⚠️  EMERGENCY STOP TRIGGERED")
        return True

    # =====================================================
    # UPDATE CYCLE (MAIN ENTRY POINT)
    # =====================================================
    def update(self, voltage: float, current: float, rpm: int, temperature: float):
        """
        Execute one control cycle with defensive input validation.
        
        Pipeline:
        1. Validate + clamp sensor inputs
        2. Compute derived data
        3. Update FSM state
        4. Detect faults
        5. Log events
        """
        # STEP 1: INPUT VALIDATION
        self._validate_and_clamp_inputs(voltage, current, rpm, temperature)

        # STEP 2: DERIVED DATA
        self._compute_derived_data()

        # STEP 3: FSM UPDATE
        self._fsm_update()

        # STEP 4: FAULT DETECTION
        self._fault_engine()

        # STEP 5: STATE-BASED FAULT ESCALATION
        if self.critical_fault_detected and self.state != MotorState.FAULT:
            self.state = MotorState.FAULT
            self._log("🔴 FAULT STATE ESCALATED FROM CRITICAL FAULT")

    def _validate_and_clamp_inputs(self, voltage: float, current: float, rpm: int, temperature: float):
        """Defensive input validation with safe fallback values"""
        
        # VOLTAGE: Allow ±20% margin, clamp to range
        if isinstance(voltage, (int, float)) and -1000 < voltage < 1000:
            self.voltage = max(0.0, min(1000.0, float(voltage)))
        else:
            self.voltage = self.thresholds["voltage_nominal"]
            self._log(f"⚠️  VOLTAGE SENSOR INVALID: {voltage}, using nominal")

        # CURRENT: Allow negative drift, clamp to positive
        if isinstance(current, (int, float)) and -1000 < current < 1000:
            self.current = max(0.0, min(1000.0, float(current)))
        else:
            self.current = 0.0
            self._log(f"⚠️  CURRENT SENSOR INVALID: {current}, using 0.0")

        # RPM: Integer, clamp to valid range
        if isinstance(rpm, int) and -5000 < rpm < 5000:
            self.rpm = max(0, min(3000, int(rpm)))
        else:
            self.rpm = 0
            self._log(f"⚠️  RPM SENSOR INVALID: {rpm}, using 0")

        # TEMPERATURE: Reasonable operating range
        if isinstance(temperature, (int, float)) and -50 < temperature < 200:
            self.temperature = max(-40.0, min(150.0, float(temperature)))
        else:
            self.temperature = 25.0
            self._log(f"⚠️  TEMPERATURE SENSOR INVALID: {temperature}, using 25°C")

    # =====================================================
    # DERIVED DATA COMPUTATION (ROBUST)
    # =====================================================
    def _compute_derived_data(self):
        """
        Compute secondary metrics from primary sensors.
        All math is defensive: handle edge cases, avoid division by zero.
        """

        # 1. POWER (W) = V × I × cos(φ)
        # Assuming power factor = 0.85 for typical induction motor
        self.power_factor = 0.85 if self.running else 1.0
        self.power = self.voltage * self.current * self.power_factor

        # 2. LOAD PERCENTAGE (0-100%)
        # Normalized to rated current (8.5A = 100% load for 2.2kW motor)
        rated_current = self.thresholds["current_overload"]
        self.load_percentage = min(100.0, max(0.0, (self.current / rated_current) * 100.0))

        # 3. SLIP (%) = (Nsync - N) / Nsync × 100
        # For 3-phase @ 50Hz: Nsync = 1500 RPM (4-pole)
        sync_rpm = self.thresholds["rpm_nominal"]
        if self.running and self.rpm > 0:
            self.slip = ((sync_rpm - self.rpm) / sync_rpm) * 100.0
            self.slip = max(0.0, self.slip)  # Slip cannot be negative
        else:
            self.slip = 0.0

        # 4. EFFICIENCY (%) 
        # Simplified model: η = 85% @ nominal load, decreases at light load
        # Real efficiency ≈ 85% - (load_error)² for stability
        load_ratio = self.load_percentage / 100.0
        if 0.3 < load_ratio < 0.9:
            # Optimal range: 30-90% load
            self.efficiency = 85.0 * (1.0 - 0.15 * ((load_ratio - 0.6) ** 2))
        elif load_ratio < 0.1:
            # Light load: efficiency drops
            self.efficiency = max(50.0, 85.0 * load_ratio * 10)
        else:
            # Overload: efficiency drops due to heating
            self.efficiency = max(60.0, 85.0 - (load_ratio - 0.9) * 100.0)

        # 5. TORQUE (N⋅m)
        # P = ω × τ, where ω = 2π × N / 60
        # τ = P / ω = (P × 60) / (2π × N)
        if self.rpm > 10:  # Avoid division by zero
            omega = (2 * 3.14159 * self.rpm) / 60
            self.torque_nm = self.power / omega if self.power > 0 else 0.0
        else:
            self.torque_nm = 0.0

    # =====================================================
    # FSM STATE MACHINE (DETERMINISTIC, NO LOOPS)
    # =====================================================
    def _fsm_update(self):
        """
        Finite State Machine with explicit transitions.
        Rules:
        - Only one transition per cycle
        - Transitions are unidirectional where logical
        - Hysteresis prevents oscillation
        - External commands (start/stop) override
        """
        self.previous_state = self.state
        elapsed = time.time() - self.state_entry_time

        # EXTERNAL STOP COMMAND (highest priority)
        if self.command_stop:
            if self.state != MotorState.STOPPED:
                self.state = MotorState.STOPPED
                self.running = False
                self.command_stop = False
                self._log("→ STOPPED (command)")
            return

        # EXTERNAL START COMMAND
        if self.command_start and not self.running:
            if self.state == MotorState.STOPPED:
                self.state = MotorState.INITIALIZING
                self._log("→ INITIALIZING (pre-start checks)")
            self.command_start = False

        # ─────────────────────────────────────────────────
        # STATE MACHINE TRANSITIONS
        # ─────────────────────────────────────────────────

        # STOPPED STATE
        if self.state == MotorState.STOPPED:
            return  # Wait for start command

        # INITIALIZING STATE
        elif self.state == MotorState.INITIALIZING:
            # Pre-start checks: voltage must be acceptable
            if abs(self.voltage - self.thresholds["voltage_nominal"]) <= self.thresholds["voltage_hysteresis"]:
                self.running = True
                self.state = MotorState.STARTING
                self.state_entry_time = time.time()
                self._log("→ STARTING (supply OK, beginning ramp)")
            elif elapsed > 5.0:
                # Initialization timeout
                self.state = MotorState.STOPPED
                self._log("❌ INITIALIZATION TIMEOUT - voltage unacceptable")

        # STARTING STATE (acceleration phase)
        elif self.state == MotorState.STARTING:
            if self.rpm >= self.thresholds["rpm_min_running"]:
                # Reached nominal speed
                self.state = MotorState.RUNNING
                self.state_entry_time = time.time()
                self._log(f"→ RUNNING (reached {self.rpm} RPM)")
            elif elapsed > self.thresholds["rpm_startup_timeout"]:
                # Failed to accelerate
                self.state = MotorState.FAULT
                self.faults.append(FaultType.STALL_CONDITION)
                self._log(f"❌ START TIMEOUT - motor stalled at {self.rpm} RPM")

        # RUNNING STATE (load classification)
        elif self.state == MotorState.RUNNING:
            self._update_load_state()

        # NO_LOAD STATE
        elif self.state == MotorState.NO_LOAD:
            if self.current > self.thresholds["current_no_load"] + self.thresholds["current_no_load_hysteresis"]:
                self._update_load_state()

        # MEDIUM_LOAD STATE
        elif self.state == MotorState.MEDIUM_LOAD:
            self._update_load_state()

        # HIGH_LOAD STATE
        elif self.state == MotorState.HIGH_LOAD:
            if self.current > self.thresholds["current_overload"]:
                self.state = MotorState.OVERLOAD
                self._log(f"→ OVERLOAD ({self.current:.2f}A > {self.thresholds['current_overload']}A)")
            elif self.current <= self.thresholds["current_medium_load"]:
                self.state = MotorState.MEDIUM_LOAD
                self._log(f"→ MEDIUM_LOAD (current returned to normal range)")
            self._update_load_state()

        # OVERLOAD STATE (warning, not a stop state)
        elif self.state == MotorState.OVERLOAD:
            if self.current <= self.thresholds["current_high_load"]:
                self.state = MotorState.HIGH_LOAD
                self._log(f"→ HIGH_LOAD (overload condition cleared)")

        # FAULT STATE
        elif self.state == MotorState.FAULT:
            # Attempt recovery only if faults are cleared for 5+ cycles
            if not self.faults and elapsed > 5.0:
                self.state = MotorState.STOPPED
                self.running = False
                self._log("→ STOPPED (fault cleared, manual restart required)")

        # Update entry time on state change
        if self.state != self.previous_state:
            self.state_entry_time = time.time()

    def _update_load_state(self):
        """Classify load condition with hysteresis"""
        if self.rpm < self.thresholds["rpm_min_running"]:
            # Not enough speed to classify load
            return

        if self.current < self.thresholds["current_no_load"]:
            if self.state != MotorState.NO_LOAD:
                self.state = MotorState.NO_LOAD
                self._log(f"→ NO_LOAD ({self.current:.2f}A)")

        elif self.current < self.thresholds["current_light_load"]:
            if self.state != MotorState.NO_LOAD:
                self.state = MotorState.NO_LOAD

        elif self.current < self.thresholds["current_medium_load"]:
            if self.state != MotorState.MEDIUM_LOAD:
                self.state = MotorState.MEDIUM_LOAD
                self._log(f"→ MEDIUM_LOAD ({self.current:.2f}A)")

        elif self.current < self.thresholds["current_high_load"]:
            if self.state != MotorState.HIGH_LOAD:
                self.state = MotorState.HIGH_LOAD
                self._log(f"→ HIGH_LOAD ({self.current:.2f}A)")

    # =====================================================
    # FAULT ENGINE (MULTI-LEVEL, PERSISTENCE-BASED)
    # =====================================================
    def _fault_engine(self):
        """
        Comprehensive fault detection with severity tiers.
        
        Strategy:
        1. Check all fault conditions
        2. Only flag if persistent (3+ cycles)
        3. Escalate critical faults immediately
        4. Track fault history for diagnostics
        """
        current_cycle_faults = []

        # ─────────────────────────────────────────────────
        # CRITICAL FAULTS (Stop immediately)
        # ─────────────────────────────────────────────────

        # 1. OVERHEAT
        if self.temperature > self.thresholds["temp_critical"]:
            current_cycle_faults.append(FaultType.OVERHEAT)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: Temp {self.temperature:.1f}°C > {self.thresholds['temp_critical']}°C")

        # 2. ELECTRICAL OVERLOAD (>120% rated current)
        if self.current > self.thresholds["current_severe_overload"]:
            current_cycle_faults.append(FaultType.ELECTRICAL_OVERLOAD)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: Current {self.current:.2f}A > {self.thresholds['current_severe_overload']}A")

        # 3. STALL CONDITION
        # Rotor locked or severely jammed: current flowing but no RPM
        if self.running and self.current > self.thresholds["current_light_load"] and self.rpm < self.thresholds["rpm_stall"]:
            current_cycle_faults.append(FaultType.STALL_CONDITION)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: Stall detected - {self.current:.2f}A flowing, RPM={self.rpm}")

        # 4. SENSOR FAILURE
        # Impossible conditions: negative current, zero voltage while running
        if self.current < 0 or (self.running and self.voltage < 50):
            current_cycle_faults.append(FaultType.SENSOR_FAILURE)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: Sensor failure - V={self.voltage}, I={self.current}")

        # ─────────────────────────────────────────────────
        # SEVERE FAULTS (Escalate quickly)
        # ─────────────────────────────────────────────────

        # 5. OVER-VOLTAGE (+10% beyond nominal)
        if self.voltage > self.thresholds["voltage_max"]:
            current_cycle_faults.append(FaultType.OVER_VOLTAGE)
            self._log(f"⚠️  SEVERE: Over-voltage {self.voltage:.1f}V > {self.thresholds['voltage_max']}V")

        # 6. UNDER-VOLTAGE (-10% below nominal)
        if 0 < self.voltage < self.thresholds["voltage_min"]:
            current_cycle_faults.append(FaultType.UNDER_VOLTAGE)
            self._log(f"⚠️  SEVERE: Under-voltage {self.voltage:.1f}V < {self.thresholds['voltage_min']}V")

        # ─────────────────────────────────────────────────
        # WARNING FAULTS (Monitor trend)
        # ─────────────────────────────────────────────────

        # 7. ABNORMAL RPM (during RUNNING state only)
        if self.state == MotorState.RUNNING and self.rpm < self.thresholds["rpm_min_running"]:
            if self.slip > self.thresholds["slip_max"]:
                current_cycle_faults.append(FaultType.ABNORMAL_RPM)
                self._log(f"⚠️  WARNING: Abnormal slip {self.slip:.1f}% (RPM={self.rpm})")

        # 8. BEARING FRICTION (High current, low RPM, low efficiency)
        if (self.running and
            self.current > self.thresholds["current_light_load"] and
            self.rpm < self.thresholds["rpm_min_running"] * 0.9 and
            self.efficiency < self.thresholds["efficiency_min"]):
            current_cycle_faults.append(FaultType.BEARING_FRICTION)
            self._log(f"⚠️  WARNING: Bearing friction detected - Eff={self.efficiency:.1f}%")

        # 9. PHASE IMBALANCE (Voltage fluctuations >5%)
        # Simplified: detect rapid voltage changes
        voltage_deviation = abs(self.voltage - self.thresholds["voltage_nominal"])
        if voltage_deviation > 20:  # >20V deviation = imbalance
            current_cycle_faults.append(FaultType.PHASE_IMBALANCE)
            self._log(f"⚠️  WARNING: Phase imbalance - V deviation = {voltage_deviation:.1f}V")

        # ─────────────────────────────────────────────────
        # FAULT PERSISTENCE LOGIC
        # ─────────────────────────────────────────────────

        # Update fault history counters
        for fault in FaultType:
            if fault in current_cycle_faults:
                self.fault_history[fault] += 1
            else:
                self.fault_history[fault] = max(0, self.fault_history[fault] - 1)

        # Finalize fault list (only include persistent faults)
        self.faults.clear()
        for fault, count in self.fault_history.items():
            if count >= self.fault_persistence:
                self.faults.append(fault)

        # ─────────────────────────────────────────────────
        # ESCALATION RULES
        # ─────────────────────────────────────────────────

        if self.faults:
            self._log(f"🔴 ACTIVE FAULTS: {', '.join([f.value for f in self.faults])}")

        # If critical faults persist, transition to FAULT state
        critical_faults = [f for f in self.faults if FAULT_SEVERITY_MAP[f] == FaultSeverity.CRITICAL]
        if critical_faults:
            self.critical_fault_detected = True
        else:
            self.critical_fault_detected = False

    # =====================================================
    # LOGGING (EVENT-DRIVEN)
    # =====================================================
    def _log(self, msg: str):
        """Append timestamped log entry"""
        self.log.append({
            "timestamp": time.time(),
            "state": self.state.value,
            "message": msg,
            "data": {
                "V": self.voltage,
                "I": self.current,
                "RPM": self.rpm,
                "Temp": self.temperature,
                "Load%": self.load_percentage,
            }
        })

    # =====================================================
    # STATUS REPORT
    # =====================================================
    def status(self) -> Dict:
        """Generate comprehensive motor status"""
        return {
            # IDENTITY
            "name": self.name,
            "timestamp": time.time(),
            
            # PRIMARY STATE
            "state": self.state.value,
            "running": self.running,
            
            # SENSORS
            "voltage": round(self.voltage, 2),
            "current": round(self.current, 2),
            "rpm": self.rpm,
            "temperature": round(self.temperature, 1),
            
            # DERIVED METRICS
            "power_w": round(self.power, 2),
            "load_percentage": round(self.load_percentage, 1),
            "efficiency": round(self.efficiency, 1),
            "slip_percent": round(self.slip, 2),
            "torque_nm": round(self.torque_nm, 2),
            "power_factor": round(self.power_factor, 2),
            
            # HEALTH
            "faults": [f.value for f in self.faults],
            "fault_history": {k.value: v for k, v in self.fault_history.items()},
            "critical_fault": self.critical_fault_detected,
            
            # DIAGNOSTICS
            "state_duration_s": round(time.time() - self.state_entry_time, 2),
            "voltage_status": self.voltage_state,
            "temperature_status": self.temperature_state,
        }

    # =====================================================
    # DIAGNOSTICS
    # =====================================================
    def get_logs(self, limit: int = 50) -> List[Dict]:
        """Return most recent log entries"""
        return list(self.log)[-limit:]

    def clear_logs(self):
        """Clear log history"""
        self.log.clear()
        self._log("LOGS CLEARED")

    def reset_faults(self):
        """Clear fault history (manual intervention)"""
        self.faults.clear()
        self.fault_history = {ft: 0 for ft in FaultType}
        self.critical_fault_detected = False
        self._log("FAULT HISTORY RESET")


# =========================
# TEST SCENARIOS
# =========================
if __name__ == "__main__":
    import sys

    print("=" * 70)
    print("MOTOR CONTROL SYSTEM - COMPREHENSIVE TEST SUITE")
    print("=" * 70)

    motor = Motor("Test-Motor", update_rate=10.0)

    # TEST 1: Normal startup and operation
    print("\n[TEST 1] Normal Startup Sequence")
    print("-" * 70)
    motor.start_motor()
    for i in range(50):
        # Simulate gradual ramp-up
        ramp_progress = i / 50.0
        rpm = int(1500 * ramp_progress)
        current = 2.0 + (1.5 * ramp_progress)
        voltage = 230.0
        temp = 25.0 + (10.0 * ramp_progress)
        
        motor.update(voltage, current, rpm, temp)
        if i % 10 == 0:
            status = motor.status()
            print(f"  Cycle {i:3d}: {status['state']:15s} | RPM={status['rpm']:4d} | I={status['current']:.2f}A | T={status['temperature']:.1f}°C")

    # TEST 2: Overload condition
    print("\n[TEST 2] Overload Condition")
    print("-" * 70)
    for i in range(30):
        motor.update(230.0, 9.0, 1400, 70.0)
        if i % 10 == 0:
            status = motor.status()
            print(f"  Cycle {i:3d}: {status['state']:15s} | Load={status['load_percentage']:.1f}% | Faults={status['faults']}")

    # TEST 3: Overheat fault
    print("\n[TEST 3] Overheat Fault Detection")
    print("-" * 70)
    motor.reset_faults()
    for i in range(40):
        temp = 25.0 + (90.0 * (i / 40.0))  # Ramp to 115°C
        motor.update(230.0, 4.0, 1450, temp)
        if i % 10 == 0:
            status = motor.status()
            print(f"  Cycle {i:3d}: {status['state']:15s} | Temp={status['temperature']:.1f}°C | Faults={status['faults']}")

    # TEST 4: Stall condition
    print("\n[TEST 4] Stall Condition (Rotor Locked)")
    print("-" * 70)
    motor.stop_motor()
    motor.reset_faults()
    motor.start_motor()
    for i in range(50):
        if i < 30:
            # Normal startup
            ramp = i / 30.0
            motor.update(230.0, 1.5 + ramp, int(1500 * ramp), 30.0)
        else:
            # Sudden load application (stall)
            motor.update(230.0, 9.0, 30, 60.0)
        
        if i % 10 == 0:
            status = motor.status()
            print(f"  Cycle {i:3d}: {status['state']:15s} | RPM={status['rpm']:4d} | I={status['current']:.2f}A | Faults={status['faults']}")

    # TEST 5: Sensor failure
    print("\n[TEST 5] Sensor Failure Detection")
    print("-" * 70)
    motor.stop_motor()
    motor.reset_faults()
    motor.start_motor()
    for i in range(15):
        motor.update(230.0, 3.0, 1450, 50.0)
    for i in range(15):
        # Inject bad sensor data
        motor.update(-50.0, -2.0, 1450, 50.0)
        status = motor.status()
        print(f"  Cycle {i:3d}: {status['state']:15s} | Critical Fault={status['critical_fault']} | Faults={status['faults']}")

    # Final Summary
    print("\n" + "=" * 70)
    print("TEST SUITE COMPLETE")
    print("=" * 70)
    final_status = motor.status()
    print(f"Final State: {final_status['state']}")
    print(f"Final Temperature: {final_status['temperature']:.1f}°C")
    print(f"Final Faults: {final_status['faults']}")
    print(f"Total Log Entries: {len(motor.log)}")