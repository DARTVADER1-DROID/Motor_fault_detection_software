from enum import Enum
from collections import deque
import time
from typing import Dict, List, Optional


# =========================
# FSM STATES (DC MOTOR)
# =========================
class MotorState(Enum):
    """Motor operational states for 12V brushed DC motor"""
    OFFLINE = "OFFLINE"              # Motor powered off, safe state
    IDLE = "IDLE"                    # Power on, motor stationary (startup preparation)
    ACCELERATING = "ACCELERATING"    # Speed ramping up from idle
    RUNNING = "RUNNING"              # Nominal operation, stable speed
    DECELERATING = "DECELERATING"    # Speed ramping down before stop
    STALLED = "STALLED"              # Motor locked, high current
    OVERHEATING = "OVERHEATING"      # Thermal warning, continue with caution
    FAULT = "FAULT"                  # Critical abnormality, requires intervention
    SHUTDOWN = "SHUTDOWN"            # Controlled emergency stop


# =========================
# FAULT TYPES (BRUSHED DC)
# =========================
class FaultType(Enum):
    """Fault conditions specific to brushed DC motors"""
    # CRITICAL FAULTS (Stop immediately)
    STALL_CONDITION = "STALL_CONDITION"              # High current, zero/near-zero speed
    THERMAL_RUNAWAY = "THERMAL_RUNAWAY"              # Temperature >90°C
    WINDING_SHORT = "WINDING_SHORT"                  # Abnormally high current at normal speed
    EXCESSIVE_BRUSH_WEAR = "EXCESSIVE_BRUSH_WEAR"   # High ripple current + low speed
    VOLTAGE_COLLAPSE = "VOLTAGE_COLLAPSE"            # Supply voltage dropped dangerously
    
    # SEVERE FAULTS (Escalate quickly)
    OVERHEAT_WARNING = "OVERHEAT_WARNING"            # Temperature 70-90°C
    HIGH_CURRENT_SPIKE = "HIGH_CURRENT_SPIKE"       # Transient overcurrent
    VOLTAGE_INSTABILITY = "VOLTAGE_INSTABILITY"      # Voltage fluctuation >2V
    BACK_EMF_ANOMALY = "BACK_EMF_ANOMALY"           # Calculated back-EMF inconsistency
    
    # WARNING FAULTS (Monitor and log)
    REDUCED_EFFICIENCY = "REDUCED_EFFICIENCY"        # Efficiency drops below 40%
    ABNORMAL_ACCELERATION = "ABNORMAL_ACCELERATION"  # Slow ramp-up (friction/wear)
    CURRENT_RIPPLE_HIGH = "CURRENT_RIPPLE_HIGH"     # High current variance (commutator wear)
    THERMAL_DRIFT = "THERMAL_DRIFT"                 # Steady temp increase trend


class FaultSeverity(Enum):
    """Fault severity classification"""
    CRITICAL = 3   # Immediate shutdown required
    SEVERE = 2     # Escalate within 2-3 cycles
    WARNING = 1    # Log and monitor


# =========================
# FAULT SEVERITY MAPPING
# =========================
FAULT_SEVERITY_MAP = {
    FaultType.STALL_CONDITION: FaultSeverity.CRITICAL,
    FaultType.THERMAL_RUNAWAY: FaultSeverity.CRITICAL,
    FaultType.WINDING_SHORT: FaultSeverity.CRITICAL,
    FaultType.EXCESSIVE_BRUSH_WEAR: FaultSeverity.CRITICAL,
    FaultType.VOLTAGE_COLLAPSE: FaultSeverity.CRITICAL,
    
    FaultType.OVERHEAT_WARNING: FaultSeverity.SEVERE,
    FaultType.HIGH_CURRENT_SPIKE: FaultSeverity.SEVERE,
    FaultType.VOLTAGE_INSTABILITY: FaultSeverity.SEVERE,
    FaultType.BACK_EMF_ANOMALY: FaultSeverity.SEVERE,
    
    FaultType.REDUCED_EFFICIENCY: FaultSeverity.WARNING,
    FaultType.ABNORMAL_ACCELERATION: FaultSeverity.WARNING,
    FaultType.CURRENT_RIPPLE_HIGH: FaultSeverity.WARNING,
    FaultType.THERMAL_DRIFT: FaultSeverity.WARNING,
}


# =========================
# LOAD CONDITIONS
# =========================
class LoadCondition(Enum):
    """Motor load classification"""
    LIGHT = "LIGHT"          # 0-20% current
    MODERATE = "MODERATE"    # 20-50% current
    NOMINAL = "NOMINAL"      # 50-80% current
    HEAVY = "HEAVY"          # 80-100% current


class DCMotor:
    """
    Bulletproof 12V Brushed DC Motor Control Model
    ──────────────────────────────────────────────
    
    DESIGN FOR COLLEGE PROJECT:
    • Sensors: Voltage, Current, Temperature, Speed only
    • No load state considered as fault (light load = normal operation)
    • Extensive derived metrics for monitoring
    • Robust fault detection with persistence
    • Deterministic FSM with hysteresis
    • Real-time efficiency analysis
    
    MOTOR SPECS (Typical 12V brushed):
    • Nominal voltage: 12V
    • Max continuous current: 5A (60W)
    • Max torque: ~0.5-1.0 N⋅m
    • Speed range: 0-3000 RPM
    • Winding resistance: ~2.4Ω (R = V_no_load / I_no_load)
    • No-load current: ~0.5A
    • Thermal time constant: ~30-60 seconds
    """

    def __init__(self, name: str = "DC-Motor-1", update_rate: float = 10.0):
        """
        Initialize 12V brushed DC motor with defensive defaults.
        
        Args:
            name: Motor identifier
            update_rate: Control loop frequency (Hz)
        """
        self.name = name
        self.update_rate = update_rate
        self.cycle_time = 1.0 / update_rate

        # =========================
        # SENSOR INPUTS (RAW)
        # =========================
        self.voltage = 12.0        # Supply voltage (V)
        self.current = 0.0         # Motor current (A)
        self.speed = 0             # Motor speed (RPM)
        self.temperature = 25.0    # Winding temperature (°C)

        # =========================
        # DERIVED METRICS (COMPUTED)
        # =========================
        # Power & Energy
        self.power_input = 0.0     # Input power: V × I (W)
        self.power_loss = 0.0      # Loss power: I²R + friction (W)
        self.power_mechanical = 0.0  # Mechanical output power (W)
        self.efficiency = 0.0      # η = P_mech / P_input (%)
        
        # Back-EMF & Motor Constants
        self.back_emf = 0.0        # Induced voltage (V)
        self.back_emf_constant = 0.0  # Ke (V⋅s/rad) calculated per cycle
        self.motor_constant = 0.0  # Km (N⋅m/A) estimated from data
        
        # Resistance & Current Analysis
        self.winding_resistance = 2.4  # Ohms (nominal for 12V brushed)
        self.resistive_loss = 0.0  # I²R losses (W)
        self.speed_rate_of_change = 0.0  # dRPM/dt (RPM/s)
        self.current_ripple = 0.0  # Current variance indicator (%)
        self.current_smoothness = 0.0  # Inverse of ripple (0-1)
        
        # Thermal Analysis
        self.thermal_power = 0.0   # Heat generation rate (W)
        self.temp_rate_of_change = 0.0  # dT/dt (°C/s)
        self.temp_rise_from_ambient = 0.0  # ΔT above 25°C baseline (°C)
        self.heat_dissipation_rate = 0.0  # Estimated cooling rate (W)
        
        # Load & Operating Point
        self.load_condition = LoadCondition.LIGHT
        self.load_percentage = 0.0  # Normalized to max current (%)
        self.speed_percentage = 0.0  # Normalized to max speed (%)
        self.duty_cycle = 0.0       # V / V_nominal (%)
        
        # Torque Estimation
        self.estimated_torque = 0.0  # N⋅m (from Km × I)
        self.estimated_mechanical_power = 0.0  # W (from τ × ω)

        # =========================
        # FSM STATE
        # =========================
        self.state = MotorState.OFFLINE
        self.previous_state = MotorState.OFFLINE
        self.state_entry_time = time.time()
        self.powered_on = False
        self.command_start = False
        self.command_stop = False
        self.acceleration_phase_time = 0.0

        # =========================
        # SPEED TRACKING (For ramp detection)
        # =========================
        self.speed_history = deque(maxlen=10)  # Last 10 speed samples
        self.current_history = deque(maxlen=10)
        self.temp_history = deque(maxlen=20)  # Last 20 temp samples for trend
        self.voltage_history = deque(maxlen=10)  # Last 10 voltage samples

        # =========================
        # THRESHOLDS (12V Brushed DC Motor)
        # =========================
        self.thresholds = {
            # VOLTAGE SAFETY (12V nominal, ±10% tolerance)
            "voltage_nominal": 12.0,
            "voltage_min_safe": 10.8,    # -10% from nominal
            "voltage_max_safe": 13.2,    # +10% from nominal
            "voltage_critical_min": 9.0,  # Severe undervoltage
            "voltage_critical_max": 14.4,  # Severe overvoltage
            "voltage_change_threshold": 0.5,  # Instability detection
            
            # CURRENT RANGES (Typical 12V 60W = 5A rated)
            "current_no_load": 0.5,       # Idling current (friction)
            "current_light_load": 1.5,    # <30% load
            "current_moderate_load": 2.5,  # 30-50% load
            "current_nominal_load": 4.0,   # 50-80% nominal operating range
            "current_heavy_load": 5.0,    # 80-100% load
            "current_stall": 5.5,         # Locked rotor current threshold
            "current_max_safe": 6.5,      # Absolute maximum (hard limit)
            "current_spike_detection": 1.5,  # Transient detection threshold above baseline
            
            # SPEED THRESHOLDS
            "speed_nominal": 3000,        # Max speed @ 12V (RPM)
            "speed_min_running": 100,     # Minimum speed to be considered "running"
            "speed_stalled": 20,          # RPM below this = stalled
            
            # ACCELERATION CHARACTERISTICS
            "acceleration_timeout": 5.0,   # Seconds to reach min speed (fault if exceeded)
            "acceleration_ramp_min": 50.0, # Min RPM/s during startup (fault if slower)
            
            # TEMPERATURE (Thermal management)
            "temp_ambient": 25.0,         # Baseline ambient
            "temp_nominal_max": 70.0,     # Normal operating limit
            "temp_warning": 75.0,         # Thermal warning threshold
            "temp_critical": 90.0,        # Thermal runaway shutdown
            "temp_hysteresis": 3.0,       # Prevent oscillation
            "temp_rise_rate_max": 1.5,    # °C/second - early warning if faster
            
            # EFFICIENCY
            "efficiency_min": 40.0,       # Below = reduced efficiency fault
            "efficiency_nominal": 75.0,   # Expected @ 50-70% load
            
            # BACK-EMF
            "back_emf_ratio_min": 0.7,    # Minimum Vb/V ratio (fault if lower)
            "back_emf_ratio_max": 0.95,   # Maximum Vb/V ratio
            
            # CURRENT RIPPLE (Commutator condition)
            "current_ripple_threshold": 30.0,  # % ripple - high = brush wear
            
            # THERMAL TIME CONSTANT
            "thermal_time_constant": 45.0,  # Seconds (exponential heating model)
        }

        # =========================
        # FAULT SYSTEM
        # =========================
        self.faults: List[FaultType] = []
        self.fault_history: Dict[FaultType, int] = {ft: 0 for ft in FaultType}
        self.fault_persistence = 3  # Fault must occur 3+ cycles to trigger
        self.critical_fault_detected = False
        self.fault_recovery_timer = 0.0

        # =========================
        # EVENT LOG
        # =========================
        self.log: deque = deque(maxlen=500)
        self._log("INITIALIZATION COMPLETE", level="info")

    # =====================================================
    # COMMAND INTERFACE
    # =====================================================
    def power_on(self) -> bool:
        """Enable motor power supply"""
        if self.state == MotorState.OFFLINE:
            self.powered_on = True
            self.state = MotorState.IDLE
            self.state_entry_time = time.time()
            self._log("Power supply ON → IDLE state", level="info")
            return True
        return False

    def power_off(self) -> bool:
        """Disable motor power supply"""
        self.powered_on = False
        self.state = MotorState.OFFLINE
        self.command_start = False
        self.command_stop = False
        self._log("Power supply OFF → OFFLINE state", level="info")
        return True

    def start_motor(self) -> bool:
        """Request motor to start accelerating"""
        if self.state in [MotorState.IDLE, MotorState.DECELERATING]:
            self.command_start = True
            self.command_stop = False
            self._log("START command received", level="info")
            return True
        return False

    def stop_motor(self) -> bool:
        """Request smooth deceleration and stop"""
        if self.state != MotorState.OFFLINE:
            self.command_stop = True
            self.command_start = False
            self._log("STOP command received", level="info")
            return True
        return False

    def emergency_stop(self) -> bool:
        """Force immediate shutdown"""
        self.powered_on = False
        self.state = MotorState.SHUTDOWN
        self.command_start = False
        self.command_stop = True
        self._log("⚠️  EMERGENCY STOP TRIGGERED", level="critical")
        return True

    # =====================================================
    # UPDATE CYCLE (MAIN ENTRY POINT)
    # =====================================================
    def update(self, voltage: float, current: float, speed: int, temperature: float):
        """
        Execute one control cycle.
        
        Pipeline:
        1. Validate + clamp sensor inputs
        2. Compute derived metrics
        3. Update FSM state machine
        4. Detect faults
        5. Log events
        """
        # STEP 1: INPUT VALIDATION & DEFENSIVE CLAMPING
        self._validate_and_clamp_inputs(voltage, current, speed, temperature)

        # STEP 2: DERIVED DATA COMPUTATION
        self._compute_derived_metrics()

        # STEP 3: HISTORY TRACKING
        self._update_history()

        # STEP 4: FSM STATE MACHINE
        self._fsm_update()

        # STEP 5: FAULT DETECTION
        self._fault_engine()

        # STEP 6: CRITICAL FAULT ESCALATION
        if self.critical_fault_detected and self.state not in [MotorState.FAULT, MotorState.SHUTDOWN]:
            self.state = MotorState.FAULT
            self._log("🔴 FAULT STATE ESCALATED FROM CRITICAL CONDITION", level="critical")

    def _validate_and_clamp_inputs(self, voltage: float, current: float, speed: int, temperature: float):
        """Defensive input validation with safe fallback values"""
        
        # VOLTAGE: Allow range 0-20V, default to 12V if invalid
        if isinstance(voltage, (int, float)) and -1 < voltage < 25:
            self.voltage = max(0.0, min(20.0, float(voltage)))
        else:
            self.voltage = self.thresholds["voltage_nominal"]
            self._log(f"⚠️  VOLTAGE SENSOR INVALID: {voltage}, using nominal 12V", level="warning")

        # CURRENT: Allow range 0-20A, default to 0A if invalid
        if isinstance(current, (int, float)) and -0.5 < current < 25:
            self.current = max(0.0, min(20.0, float(current)))
        else:
            self.current = 0.0
            self._log(f"⚠️  CURRENT SENSOR INVALID: {current}, using 0A", level="warning")

        # SPEED: Integer RPM, clamp to valid range
        if isinstance(speed, int) and -100 < speed < 4000:
            self.speed = max(0, min(3500, int(speed)))
        else:
            self.speed = 0
            self._log(f"⚠️  SPEED SENSOR INVALID: {speed}, using 0 RPM", level="warning")

        # TEMPERATURE: Reasonable range -10 to 150°C, default to ambient
        if isinstance(temperature, (int, float)) and -20 < temperature < 160:
            self.temperature = max(-10.0, min(150.0, float(temperature)))
        else:
            self.temperature = self.thresholds["temp_ambient"]
            self._log(f"⚠️  TEMPERATURE SENSOR INVALID: {temperature}, using 25°C", level="warning")

    def _update_history(self):
        """Maintain sliding window history for trend analysis"""
        self.speed_history.append(self.speed)
        self.current_history.append(self.current)
        self.temp_history.append(self.temperature)
        self.voltage_history.append(self.voltage)

    # =====================================================
    # DERIVED METRICS COMPUTATION (EXTENSIVE)
    # =====================================================
    def _compute_derived_metrics(self):
        """
        Compute 14+ derived metrics from 4 sensor inputs.
        All calculations are defensive: handle edge cases, avoid division by zero.
        """

        # ─────────────────────────────────────────────────
        # 1. POWER ANALYSIS
        # ─────────────────────────────────────────────────
        
        # Input power: P_in = V × I
        self.power_input = self.voltage * self.current

        # Resistive loss: P_loss = I² × R (assuming 2.4Ω winding resistance)
        self.resistive_loss = (self.current ** 2) * self.winding_resistance

        # Mechanical power: P_mech = P_in - P_loss
        # (Simplified: assumes friction loss is proportional to speed)
        friction_loss = max(0.0, 0.3 * self.power_input)  # ~30% at nominal load
        self.power_mechanical = max(0.0, self.power_input - self.resistive_loss - friction_loss)

        # Efficiency: η = P_mech / P_in (%)
        if self.power_input > 0.1:
            self.efficiency = max(0.0, (self.power_mechanical / self.power_input) * 100.0)
        else:
            self.efficiency = 0.0

        # ─────────────────────────────────────────────────
        # 2. BACK-EMF & MOTOR CONSTANTS
        # ─────────────────────────────────────────────────

        # Back-EMF: Vb = V - (I × R)
        # This is the induced voltage that opposes applied voltage
        self.back_emf = max(0.0, self.voltage - (self.current * self.winding_resistance))

        # Back-EMF ratio: Vb/V (0-1 scale)
        # High ratio = motor running at full speed with light load
        # Low ratio = motor loaded heavily or stalled
        if self.voltage > 0.5:
            back_emf_ratio = self.back_emf / self.voltage
        else:
            back_emf_ratio = 0.0

        # Ke (back-EMF constant): Vb = Ke × ω
        # Vb = Ke × (RPM × 2π / 60)
        if self.speed > 100:
            omega = (self.speed * 2.0 * 3.14159) / 60.0
            self.back_emf_constant = self.back_emf / omega if omega > 0 else 0.0
        else:
            self.back_emf_constant = 0.0

        # Km (torque constant): τ = Km × I
        # For most DC motors: Ke ≈ Km (in SI units)
        self.motor_constant = self.back_emf_constant if self.back_emf_constant > 0 else 0.01

        # ─────────────────────────────────────────────────
        # 3. SPEED & ACCELERATION ANALYSIS
        # ─────────────────────────────────────────────────

        # Speed rate of change: dRPM/dt (RPM/s)
        if len(self.speed_history) >= 2:
            prev_speed = self.speed_history[-2]
            speed_delta = self.speed - prev_speed
            self.speed_rate_of_change = speed_delta * self.update_rate
        else:
            self.speed_rate_of_change = 0.0

        # Speed percentage: 0-100% of max speed (3000 RPM)
        self.speed_percentage = min(100.0, (self.speed / self.thresholds["speed_nominal"]) * 100.0)

        # ─────────────────────────────────────────────────
        # 4. CURRENT RIPPLE & COMMUTATOR CONDITION
        # ─────────────────────────────────────────────────

        # Current ripple: variance of current over last 10 samples
        if len(self.current_history) >= 5 and self.current > 0.1:
            avg_current = sum(self.current_history) / len(self.current_history)
            variance = sum((i - avg_current) ** 2 for i in self.current_history) / len(self.current_history)
            std_dev = variance ** 0.5
            self.current_ripple = (std_dev / avg_current * 100.0) if avg_current > 0 else 0.0
            self.current_smoothness = max(0.0, 1.0 - (self.current_ripple / 100.0))
        else:
            self.current_ripple = 0.0
            self.current_smoothness = 1.0

        # ─────────────────────────────────────────────────
        # 5. THERMAL ANALYSIS
        # ─────────────────────────────────────────────────

        # Heat generation: primarily from I²R losses + friction
        self.thermal_power = self.resistive_loss + friction_loss

        # Temperature rise from ambient
        self.temp_rise_from_ambient = self.temperature - self.thresholds["temp_ambient"]

        # Rate of temperature change: dT/dt (°C/s)
        if len(self.temp_history) >= 2:
            prev_temp = self.temp_history[-2]
            temp_delta = self.temperature - prev_temp
            self.temp_rate_of_change = temp_delta * self.update_rate
        else:
            self.temp_rate_of_change = 0.0

        # Heat dissipation rate (Newton's Law of Cooling approximation)
        # Higher motor running = faster heat dissipation (air flow effect)
        # Q_dissipate ≈ k × ΔT × (1 + v_factor)
        speed_factor = 1.0 + (self.speed / self.thresholds["speed_nominal"]) * 0.5
        self.heat_dissipation_rate = 0.05 * self.temp_rise_from_ambient * speed_factor

        # ─────────────────────────────────────────────────
        # 6. LOAD CLASSIFICATION
        # ─────────────────────────────────────────────────

        # Load percentage: normalized to max safe current
        max_safe_current = self.thresholds["current_nominal_load"]
        self.load_percentage = min(100.0, (self.current / max_safe_current) * 100.0)

        # Load condition classification
        if self.current < self.thresholds["current_light_load"]:
            self.load_condition = LoadCondition.LIGHT
        elif self.current < self.thresholds["current_moderate_load"]:
            self.load_condition = LoadCondition.MODERATE
        elif self.current < self.thresholds["current_nominal_load"]:
            self.load_condition = LoadCondition.NOMINAL
        else:
            self.load_condition = LoadCondition.HEAVY

        # Duty cycle: voltage ratio
        if self.thresholds["voltage_nominal"] > 0:
            self.duty_cycle = min(100.0, (self.voltage / self.thresholds["voltage_nominal"]) * 100.0)
        else:
            self.duty_cycle = 0.0

        # ─────────────────────────────────────────────────
        # 7. TORQUE ESTIMATION
        # ─────────────────────────────────────────────────

        # Estimated torque: τ = Km × I
        # Km is motor constant (N⋅m/A), typically 0.01-0.1 for small brushed DC
        self.estimated_torque = self.motor_constant * self.current

        # Estimated mechanical power from torque
        # P_mech = τ × ω
        if self.speed > 10:
            omega = (self.speed * 2.0 * 3.14159) / 60.0
            self.estimated_mechanical_power = self.estimated_torque * omega
        else:
            self.estimated_mechanical_power = 0.0

    # =====================================================
    # FSM STATE MACHINE (DETERMINISTIC, ROBUST)
    # =====================================================
    def _fsm_update(self):
        """
        Finite State Machine for 12V brushed DC motor.
        
        State Transitions:
        OFFLINE ← power_off
          ↓ power_on
        IDLE ← stop/decelerate to zero
          ↓ start command
        ACCELERATING → speed reaches min_running
          ↓
        RUNNING ← stable speed
          ↓ stop command
        DECELERATING → speed reaches zero
          ↓
        IDLE/OFFLINE
        
        STALLED ← high current + low speed
        FAULT ← critical condition
        """
        self.previous_state = self.state
        elapsed = time.time() - self.state_entry_time

        # ─────────────────────────────────────────────────
        # EXTERNAL COMMANDS (Highest priority)
        # ─────────────────────────────────────────────────

        if not self.powered_on:
            if self.state != MotorState.OFFLINE:
                self.state = MotorState.OFFLINE
                self.speed = 0
                self._log("→ OFFLINE (power disabled)", level="info")
            return

        # ─────────────────────────────────────────────────
        # STATE MACHINE LOGIC
        # ─────────────────────────────────────────────────

        # OFFLINE STATE
        if self.state == MotorState.OFFLINE:
            # Transition on power_on
            if self.powered_on:
                self.state = MotorState.IDLE
                self.state_entry_time = time.time()
                self._log("→ IDLE (power enabled)", level="info")

        # IDLE STATE (Power on, motor stationary)
        elif self.state == MotorState.IDLE:
            if self.command_start:
                self.state = MotorState.ACCELERATING
                self.state_entry_time = time.time()
                self.acceleration_phase_time = 0.0
                self.command_start = False
                self._log("→ ACCELERATING (start command)", level="info")

        # ACCELERATING STATE (Ramping up speed)
        elif self.state == MotorState.ACCELERATING:
            self.acceleration_phase_time = elapsed

            if self.speed >= self.thresholds["speed_min_running"]:
                # Reached minimum running speed
                self.state = MotorState.RUNNING
                self.state_entry_time = time.time()
                self._log(f"→ RUNNING (speed reached {self.speed} RPM)", level="info")

            elif elapsed > self.thresholds["acceleration_timeout"]:
                # Failed to accelerate within timeout
                self.state = MotorState.FAULT
                self.faults.append(FaultType.STALL_CONDITION)
                self._log(f"❌ ACCELERATION TIMEOUT - stalled at {self.speed} RPM after {elapsed:.1f}s", level="critical")

            elif self.speed_rate_of_change < self.thresholds["acceleration_ramp_min"] and elapsed > 0.5:
                # Acceleration is too slow (friction/mechanical jam)
                # Log as warning but don't fault immediately (allow 3-cycle persistence)
                self._log(f"⚠️  SLOW ACCELERATION: {self.speed_rate_of_change:.1f} RPM/s", level="warning")

        # RUNNING STATE (Nominal operation)
        elif self.state == MotorState.RUNNING:
            if self.command_stop:
                self.state = MotorState.DECELERATING
                self.state_entry_time = time.time()
                self.command_stop = False
                self._log("→ DECELERATING (stop command)", level="info")

            # Transition to STALLED if conditions met
            if self.current > self.thresholds["current_stall"] and self.speed < self.thresholds["speed_stalled"]:
                self.state = MotorState.STALLED
                self.state_entry_time = time.time()
                self._log(f"⚠️  STALLED: {self.current:.2f}A at {self.speed} RPM", level="warning")

        # DECELERATING STATE (Ramping down speed)
        elif self.state == MotorState.DECELERATING:
            if self.speed < self.thresholds["speed_min_running"]:
                self.state = MotorState.IDLE
                self.state_entry_time = time.time()
                self._log("→ IDLE (speed decayed)", level="info")

        # STALLED STATE (High current, low speed)
        elif self.state == MotorState.STALLED:
            # If stall condition clears (current drops or speed increases)
            if (self.current < self.thresholds["current_heavy_load"] and
                self.speed > self.thresholds["speed_stalled"]):
                self.state = MotorState.RUNNING
                self.state_entry_time = time.time()
                self._log("→ RUNNING (stall condition cleared)", level="info")

            # Or transition to fault if severe
            elif (self.current > self.thresholds["current_max_safe"] and
                  self.speed < self.thresholds["speed_stalled"]):
                self.state = MotorState.FAULT
                self._log("❌ STALL ESCALATED TO FAULT (overcurrent)", level="critical")

        # OVERHEATING STATE
        elif self.state == MotorState.OVERHEATING:
            if self.temperature < self.thresholds["temp_warning"] - self.thresholds["temp_hysteresis"]:
                self.state = MotorState.RUNNING
                self.state_entry_time = time.time()
                self._log("→ RUNNING (temperature normalized)", level="info")

        # FAULT STATE
        elif self.state == MotorState.FAULT:
            # Only recover if faults are cleared AND timeout elapsed
            if not self.critical_fault_detected and elapsed > 3.0:
                self.state = MotorState.IDLE
                self.state_entry_time = time.time()
                self._log("→ IDLE (fault recovery, manual restart needed)", level="info")

        # SHUTDOWN STATE
        elif self.state == MotorState.SHUTDOWN:
            # Force offline after 2 seconds
            if elapsed > 2.0:
                self.state = MotorState.OFFLINE
                self._log("→ OFFLINE (emergency shutdown complete)", level="info")

        # Update entry time on state change
        if self.state != self.previous_state:
            self.state_entry_time = time.time()

    # =====================================================
    # FAULT ENGINE (COMPREHENSIVE & PERSISTENT)
    # =====================================================
    def _fault_engine(self):
        """
        Multi-level fault detection with severity classification.
        
        Strategy:
        1. Check all fault conditions in this cycle
        2. Persist faults across 3+ cycles to filter noise
        3. Escalate critical faults immediately
        4. Track history for diagnostics
        """
        current_cycle_faults = []

        # ─────────────────────────────────────────────────
        # CRITICAL FAULTS (Immediate stop required)
        # ─────────────────────────────────────────────────

        # 1. STALL CONDITION
        # High current flowing but motor locked or severely jammed
        if (self.powered_on and
            self.current > self.thresholds["current_heavy_load"] and
            self.speed < self.thresholds["speed_stalled"]):
            current_cycle_faults.append(FaultType.STALL_CONDITION)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: STALL - {self.current:.2f}A @ {self.speed} RPM", level="critical")

        # 2. THERMAL RUNAWAY
        # Temperature exceeds critical threshold (90°C)
        if self.temperature > self.thresholds["temp_critical"]:
            current_cycle_faults.append(FaultType.THERMAL_RUNAWAY)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: THERMAL RUNAWAY - {self.temperature:.1f}°C", level="critical")

        # 3. WINDING SHORT CIRCUIT
        # High current at normal/high speed = low impedance (short)
        # Indicator: current > 6A while speed > 1500 RPM
        if (self.speed > 1500 and
            self.current > 6.0 and
            self.back_emf < self.voltage * 0.5):
            current_cycle_faults.append(FaultType.WINDING_SHORT)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: WINDING SHORT - {self.current:.2f}A, Back-EMF={self.back_emf:.2f}V", level="critical")

        # 4. EXCESSIVE BRUSH WEAR
        # High current ripple + low back-EMF constant = worn commutator
        if (self.current > 2.0 and
            self.speed < 1000 and
            self.current_ripple > self.thresholds["current_ripple_threshold"]):
            current_cycle_faults.append(FaultType.EXCESSIVE_BRUSH_WEAR)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: BRUSH WEAR - Ripple {self.current_ripple:.1f}%, Speed {self.speed} RPM", level="critical")

        # 5. VOLTAGE COLLAPSE
        # Supply voltage dropped below safe minimum during operation
        if (self.powered_on and
            self.voltage < self.thresholds["voltage_critical_min"] and
            self.speed > self.thresholds["speed_min_running"]):
            current_cycle_faults.append(FaultType.VOLTAGE_COLLAPSE)
            self.critical_fault_detected = True
            self._log(f"🔴 CRITICAL: VOLTAGE COLLAPSE - {self.voltage:.2f}V", level="critical")

        # ─────────────────────────────────────────────────
        # SEVERE FAULTS (Escalate within 2-3 cycles)
        # ─────────────────────────────────────────────────

        # 6. OVERHEAT WARNING
        # Temperature in warning range (70-90°C)
        if (self.thresholds["temp_warning"] <= self.temperature <
            self.thresholds["temp_critical"]):
            current_cycle_faults.append(FaultType.OVERHEAT_WARNING)
            self._log(f"⚠️  SEVERE: OVERHEAT WARNING - {self.temperature:.1f}°C", level="warning")

        # 7. HIGH CURRENT SPIKE
        # Transient overcurrent above baseline (e.g., sudden load spike)
        if len(self.current_history) > 1:
            baseline_current = sum(list(self.current_history)[:-1]) / (len(self.current_history) - 1)
        else:
            baseline_current = 0
        current_spike = self.current - baseline_current
        if (current_spike > self.thresholds["current_spike_detection"] and
            self.current > 3.0):
            current_cycle_faults.append(FaultType.HIGH_CURRENT_SPIKE)
            self._log(f"⚠️  SEVERE: CURRENT SPIKE - {self.current:.2f}A (Δ{current_spike:.2f}A)", level="warning")

        # 8. VOLTAGE INSTABILITY
        # Supply voltage fluctuating more than 2V
        if len(self.voltage_history) >= 3:
            voltage_range = max(self.voltage_history) - min(self.voltage_history)
            if voltage_range > self.thresholds["voltage_change_threshold"] * 2:
                current_cycle_faults.append(FaultType.VOLTAGE_INSTABILITY)
                self._log(f"⚠️  SEVERE: VOLTAGE INSTABILITY - Range {voltage_range:.2f}V", level="warning")

        # 9. BACK-EMF ANOMALY
        # Back-EMF ratio outside expected range (indicates motor damage)
        if self.voltage > 0.5:
            back_emf_ratio = self.back_emf / self.voltage
            if (self.speed > 1000 and
                (back_emf_ratio < self.thresholds["back_emf_ratio_min"] or
                 back_emf_ratio > self.thresholds["back_emf_ratio_max"])):
                current_cycle_faults.append(FaultType.BACK_EMF_ANOMALY)
                self._log(f"⚠️  SEVERE: BACK-EMF ANOMALY - Ratio {back_emf_ratio:.3f}", level="warning")

        # ─────────────────────────────────────────────────
        # WARNING FAULTS (Monitor and log)
        # ─────────────────────────────────────────────────

        # 10. REDUCED EFFICIENCY
        # Efficiency drops below 40% (friction, load mismatch, aging)
        if (self.powered_on and
            self.speed > self.thresholds["speed_min_running"] and
            self.efficiency < self.thresholds["efficiency_min"]):
            current_cycle_faults.append(FaultType.REDUCED_EFFICIENCY)
            self._log(f"⚠️  WARNING: REDUCED EFFICIENCY - {self.efficiency:.1f}%", level="warning")

        # 11. ABNORMAL ACCELERATION
        # Slow ramp-up rate during starting phase
        if (self.state == MotorState.ACCELERATING and
            self.acceleration_phase_time > 1.0 and
            self.speed_rate_of_change < self.thresholds["acceleration_ramp_min"]):
            current_cycle_faults.append(FaultType.ABNORMAL_ACCELERATION)
            self._log(f"⚠️  WARNING: SLOW ACCELERATION - {self.speed_rate_of_change:.1f} RPM/s", level="warning")

        # 12. CURRENT RIPPLE (Commutator wear indicator)
        # High ripple suggests worn brushes or commutator segments
        if (self.current > 1.0 and
            self.current_ripple > self.thresholds["current_ripple_threshold"]):
            current_cycle_faults.append(FaultType.CURRENT_RIPPLE_HIGH)
            self._log(f"⚠️  WARNING: HIGH CURRENT RIPPLE - {self.current_ripple:.1f}%", level="warning")

        # 13. THERMAL DRIFT
        # Temperature increasing too rapidly (trend warning)
        if (self.temp_rate_of_change >
            self.thresholds["temp_rise_rate_max"] and
            self.temperature > 50.0):
            current_cycle_faults.append(FaultType.THERMAL_DRIFT)
            self._log(f"⚠️  WARNING: THERMAL DRIFT - {self.temp_rate_of_change:.2f}°C/s", level="warning")

        # ─────────────────────────────────────────────────
        # FAULT PERSISTENCE LOGIC
        # ─────────────────────────────────────────────────

        # Update fault history: increment on detection, decrement otherwise
        for fault in FaultType:
            if fault in current_cycle_faults:
                self.fault_history[fault] += 1
            else:
                self.fault_history[fault] = max(0, self.fault_history[fault] - 1)

        # Finalize fault list: only include persistent faults
        self.faults.clear()
        for fault, count in self.fault_history.items():
            if count >= self.fault_persistence:
                self.faults.append(fault)

        # ─────────────────────────────────────────────────
        # ESCALATION RULES
        # ─────────────────────────────────────────────────

        if self.faults:
            fault_names = ', '.join([f.value for f in self.faults])
            self._log(f"🔴 ACTIVE FAULTS: {fault_names}", level="critical" if self.critical_fault_detected else "warning")

        # Check for critical faults
        critical_faults = [f for f in self.faults if FAULT_SEVERITY_MAP[f] == FaultSeverity.CRITICAL]
        if critical_faults:
            self.critical_fault_detected = True
        else:
            self.critical_fault_detected = False

    # =====================================================
    # LOGGING (EVENT-DRIVEN)
    # =====================================================
    def _log(self, msg: str, level: str = "info"):
        """Append timestamped log entry with severity level"""
        self.log.append({
            "timestamp": time.time(),
            "state": self.state.value,
            "level": level,
            "message": msg,
            "snapshot": {
                "V": round(self.voltage, 2),
                "I": round(self.current, 2),
                "RPM": self.speed,
                "T": round(self.temperature, 1),
                "Eff": round(self.efficiency, 1),
            }
        })

    # =====================================================
    # STATUS REPORT
    # =====================================================
    def status(self) -> Dict:
        """Generate comprehensive motor status snapshot"""
        return {
            # IDENTITY & TIMING
            "name": self.name,
            "timestamp": time.time(),
            "state": self.state.value,
            
            # SENSORS (Raw)
            "voltage": round(self.voltage, 2),
            "current": round(self.current, 2),
            "speed": self.speed,
            "temperature": round(self.temperature, 1),
            
            # POWER & ENERGY
            "power_input_w": round(self.power_input, 2),
            "power_mechanical_w": round(self.power_mechanical, 2),
            "power_loss_w": round(self.resistive_loss + 0.3 * self.power_input, 2),
            "efficiency_percent": round(self.efficiency, 1),
            
            # BACK-EMF & MOTOR CONSTANTS
            "back_emf_v": round(self.back_emf, 2),
            "back_emf_constant_ke": round(self.back_emf_constant, 4),
            "motor_constant_km": round(self.motor_constant, 4),
            
            # SPEED & ACCELERATION
            "speed_percent_of_max": round(self.speed_percentage, 1),
            "acceleration_rpm_per_sec": round(self.speed_rate_of_change, 2),
            
            # CURRENT ANALYSIS
            "current_ripple_percent": round(self.current_ripple, 1),
            "current_smoothness_0_to_1": round(self.current_smoothness, 2),
            "load_condition": self.load_condition.value,
            "load_percent": round(self.load_percentage, 1),
            
            # THERMAL
            "temp_rise_from_ambient_c": round(self.temp_rise_from_ambient, 1),
            "thermal_power_w": round(self.thermal_power, 2),
            "temp_rate_of_change_c_per_sec": round(self.temp_rate_of_change, 3),
            "heat_dissipation_w": round(self.heat_dissipation_rate, 2),
            
            # TORQUE & MECHANICAL
            "estimated_torque_nm": round(self.estimated_torque, 3),
            "estimated_mechanical_power_w": round(self.estimated_mechanical_power, 2),
            
            # HEALTH
            "faults": [f.value for f in self.faults],
            "fault_history": {k.value: v for k, v in self.fault_history.items() if v > 0},
            "critical_fault": self.critical_fault_detected,
            
            # STATE TRACKING
            "state_duration_seconds": round(time.time() - self.state_entry_time, 2),
            "powered_on": self.powered_on,
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
        self._log("LOGS CLEARED", level="info")

    def reset_faults(self):
        """Clear fault history (manual intervention)"""
        self.faults.clear()
        self.fault_history = {ft: 0 for ft in FaultType}
        self.critical_fault_detected = False
        self._log("FAULT HISTORY RESET", level="info")

    def get_health_score(self) -> float:
        """
        Calculate overall motor health (0-100%).
        
        Factors:
        - Temperature (20% weight)
        - Efficiency (20% weight)
        - Current ripple (15% weight)
        - Thermal drift (15% weight)
        - Active faults (30% weight)
        """
        score = 100.0

        # Temperature penalty
        if self.temperature > self.thresholds["temp_critical"]:
            temp_penalty = 100.0
        elif self.temperature > self.thresholds["temp_warning"]:
            temp_penalty = 20.0 + (self.temperature - self.thresholds["temp_warning"]) / 15.0 * 30.0
        else:
            temp_penalty = 0.0
        score -= temp_penalty * 0.20

        # Efficiency penalty
        if self.efficiency < self.thresholds["efficiency_min"]:
            eff_penalty = 20.0
        elif self.efficiency < 60.0:
            eff_penalty = (60.0 - self.efficiency) / 20.0 * 15.0
        else:
            eff_penalty = 0.0
        score -= eff_penalty * 0.20

        # Current ripple penalty (commutator wear indicator)
        ripple_penalty = min(15.0, self.current_ripple / self.thresholds["current_ripple_threshold"] * 15.0)
        score -= ripple_penalty * 0.15

        # Thermal drift penalty
        drift_penalty = min(15.0, max(0.0, self.temp_rate_of_change / self.thresholds["temp_rise_rate_max"] * 15.0))
        score -= drift_penalty * 0.15

        # Fault penalty
        fault_penalty = min(30.0, len(self.faults) * 10.0)
        score -= fault_penalty * 0.30

        return max(0.0, min(100.0, score))


# =========================
# TEST SCENARIOS
# =========================
if __name__ == "__main__":
    print("=" * 80)
    print("12V BRUSHED DC MOTOR CONTROL SYSTEM - COMPREHENSIVE TEST SUITE")
    print("=" * 80)

    motor = DCMotor("Test-Motor", update_rate=10.0)

    # TEST 1: Power-on and idle
    print("\n[TEST 1] Power ON → IDLE State")
    print("-" * 80)
    motor.power_on()
    for i in range(5):
        motor.update(12.0, 0.5, 0, 25.0)
        status = motor.status()
        print(f"  Cycle {i}: {status['state']:15s} | V={status['voltage']:.1f}V | I={status['current']:.2f}A | T={status['temperature']:.1f}°C")

    # TEST 2: Startup and acceleration
    print("\n[TEST 2] START Command → ACCELERATING → RUNNING")
    print("-" * 80)
    motor.start_motor()
    for i in range(60):
        ramp = i / 60.0
        rpm = int(3000 * ramp)
        current = 0.5 + (4.0 * ramp)
        motor.update(12.0, current, rpm, 25.0 + 30.0 * ramp)
        if i % 15 == 0:
            status = motor.status()
            print(f"  Cycle {i:2d}: {status['state']:15s} | RPM={status['speed']:4d} | I={status['current']:.2f}A | Eff={status['efficiency_percent']:.1f}% | T={status['temperature']:.1f}°C")

    # TEST 3: Stall detection
    print("\n[TEST 3] Sudden STALL Condition (High Current, Zero RPM)")
    print("-" * 80)
    for i in range(30):
        if i < 20:
            # Normal running
            motor.update(12.0, 2.0, 2000, 60.0)
        else:
            # Sudden jam (load applied)
            motor.update(12.0, 6.0, 10, 75.0)
        
        if i >= 15:
            status = motor.status()
            print(f"  Cycle {i:2d}: {status['state']:15s} | RPM={status['speed']:4d} | I={status['current']:.2f}A | Faults={status['faults']}")

    # TEST 4: Thermal runaway
    print("\n[TEST 4] OVERHEAT WARNING → THERMAL RUNAWAY")
    print("-" * 80)
    motor.reset_faults()
    motor.power_on()
    motor.start_motor()
    for i in range(50):
        temp = 25.0 + (70.0 * (i / 50.0))
        motor.update(12.0, 3.5, 2500, temp)
        if i % 10 == 0:
            status = motor.status()
            print(f"  Cycle {i:2d}: {status['state']:15s} | T={status['temperature']:.1f}°C | Critical={status['critical_fault']} | Faults={status['faults']}")

    # TEST 5: Voltage instability
    print("\n[TEST 5] VOLTAGE INSTABILITY Detection")
    print("-" * 80)
    motor.reset_faults()
    motor.power_on()
    motor.start_motor()
    voltages = [12.0, 11.8, 13.2, 11.5, 12.0, 12.3, 11.0, 12.8, 12.0]  # Fluctuating
    for i, v in enumerate(voltages * 3):
        motor.update(v, 2.5, 2000, 55.0)
        status = motor.status()
        print(f"  Cycle {i:2d}: V={status['voltage']:.2f}V | Faults={status['faults']}")

    # TEST 6: Health score tracking
    print("\n[TEST 6] HEALTH SCORE Degradation")
    print("-" * 80)
    motor.reset_faults()
    motor.power_on()
    motor.start_motor()
    for i in range(40):
        # Simulate aging: increasing current ripple, temp drift
        temp = 25.0 + (50.0 * (i / 40.0))
        current = 2.0 + (2.0 * (i / 40.0))
        motor.update(12.0, current, 2000 - (500 * i / 40.0), temp)
        if i % 10 == 0:
            status = motor.status()
            health = motor.get_health_score()
            print(f"  Cycle {i:2d}: Health={health:5.1f}% | T={status['temperature']:.1f}°C | Eff={status['efficiency_percent']:.1f}% | Ripple={status['current_ripple_percent']:.1f}%")

    # Final Summary
    print("\n" + "=" * 80)
    print("TEST SUITE COMPLETE")
    print("=" * 80)
    final_status = motor.status()
    print(f"Final State: {final_status['state']}")
    print(f"Health Score: {motor.get_health_score():.1f}%")
    print(f"Active Faults: {final_status['faults']}")
    print(f"Total Log Entries: {len(motor.log)}")