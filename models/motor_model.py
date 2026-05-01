from enum import Enum
from collections import deque
import time
from typing import Dict, List


# =========================
# MOTOR STATES
# =========================
class MotorState(Enum):
    """Simple, practical states for 12V DC motor"""
    OFF = "OFF"                    # No power
    IDLE = "IDLE"                  # Power on, stationary
    RUNNING = "RUNNING"            # Motor spinning normally
    FAULT = "FAULT"                # Critical issue, stop needed


# =========================
# FAULT TYPES
# =========================
class FaultType(Enum):
    """Practical faults for 12V brushed DC"""
    # Critical - must stop
    STALL = "STALL"                        # High current + zero/very low speed
    THERMAL_RUNAWAY = "THERMAL_RUNAWAY"    # Temp > 90°C
    WINDING_SHORT = "WINDING_SHORT"        # High current but very low back-EMF
    
    # Severe - escalate
    OVERHEAT = "OVERHEAT"                  # Temp 75-90°C
    HIGH_CURRENT = "HIGH_CURRENT"          # Current spike > 6.5A
    VOLTAGE_DROP = "VOLTAGE_DROP"          # Supply voltage too low (< 10V)
    
    # Warning - monitor
    LOW_EFFICIENCY = "LOW_EFFICIENCY"      # Efficiency < 50% (bearing friction)
    BRUSH_WEAR = "BRUSH_WEAR"              # High current ripple (poor commutation)


class DCMotor12V:
    """
    12V Brushed DC Motor Control Model
    ──────────────────────────────────
    
    SIMPLE. PRACTICAL. COLLEGE-READY.
    
    Sensors: V (voltage), I (current), T (temperature), N (RPM)
    No load = Normal. Motor will mostly run at no-load.
    
    Derived quantities:
    - Power (input, mechanical, loss)
    - Efficiency
    - Back-EMF & motor constant
    - Torque estimate
    - Current ripple (brush condition)
    - Thermal analysis
    """

    def __init__(self, name: str = "12V-Motor", update_rate: float = 10.0):
        """
        Args:
            name: Motor identifier
            update_rate: Control loop frequency (Hz)
        """
        self.name = name
        self.update_rate = update_rate

        # =========================
        # SENSOR INPUTS
        # =========================
        self.voltage = 12.0        # V (supply voltage)
        self.current = 0.0         # A (motor current)
        self.speed = 0             # RPM (motor speed)
        self.temperature = 25.0    # °C (winding temperature)

        # =========================
        # DERIVED QUANTITIES
        # =========================
        # Power
        self.power_input = 0.0          # W (V × I)
        self.power_loss = 0.0           # W (I²R + friction)
        self.power_mechanical = 0.0     # W (output power)
        self.efficiency = 0.0           # % (mechanical/input)

        # Back-EMF & Motor Constants
        self.back_emf = 0.0             # V (calculated)
        self.ke = 0.0                   # Back-EMF constant (V⋅s/rad)
        self.km = 0.075                 # Torque constant (N⋅m/A) - standard for 12V

        # Load & Speed
        self.speed_pct = 0.0            # % of max (3000 RPM nominal)
        self.load_pct = 0.0             # % based on current (0-5A = 0-100%)
        self.torque = 0.0               # N⋅m (estimated)

        # Current Analysis
        self.current_ripple = 0.0       # % (variance indicator)
        self.current_avg = 0.0          # A (moving average)

        # Thermal
        self.temp_rise = 0.0            # °C (above 25°C ambient)
        self.temp_rise_rate = 0.0       # °C/s (rate of change)

        # =========================
        # MOTOR SPECS (12V brushed typical)
        # =========================
        self.R = 2.4                    # Ohms (winding resistance, from datasheet)
        self.rpm_max = 3000             # RPM (no-load max speed)
        self.current_nominal = 4.0      # A (rated continuous current)
        self.current_max = 6.5          # A (absolute max safe)

        # =========================
        # STATE MACHINE
        # =========================
        self.state = MotorState.OFF
        self.running = False
        self.powered_on = False
        self.command_start = False
        self.command_stop = False
        self.state_entry_time = time.time()

        # =========================
        # HISTORY (for ripple, trends)
        # =========================
        self.current_history = deque(maxlen=15)  # Last 15 current samples
        self.speed_history = deque(maxlen=15)
        self.temp_history = deque(maxlen=20)

        # =========================
        # FAULTS
        # =========================
        self.faults = []
        self.fault_counts = {ft: 0 for ft in FaultType}  # Persistence tracking
        self.fault_persistence_threshold = 3  # Must occur 3+ cycles to trigger

        # =========================
        # THRESHOLDS (tuned for 12V motor)
        # =========================
        self.thresholds = {
            # Current (A)
            "current_no_load": 0.5,      # Typical no-load current
            "current_light": 1.5,        # Light load
            "current_nominal": 4.0,      # Normal operating
            "current_spike": 6.5,        # Hard limit
            
            # Speed (RPM)
            "speed_min_running": 50,     # Motor considered "running"
            "speed_nominal": 2500,       # Comfortable operating speed
            
            # Temperature (°C)
            "temp_ambient": 25.0,
            "temp_warning": 75.0,        # Warm
            "temp_critical": 90.0,       # Dangerous
            
            # Voltage (V)
            "voltage_nominal": 12.0,
            "voltage_low": 10.0,         # Too low, motor struggles
            
            # Current ripple (%)
            "ripple_healthy": 15.0,      # Good brush contact
            "ripple_worn": 30.0,         # Worn brushes
            
            # Efficiency (%)
            "efficiency_good": 70.0,     # Healthy
            "efficiency_low": 50.0,      # Bearing friction/wear
        }

        # =========================
        # LOGGING
        # =========================
        self.logs = deque(maxlen=300)
        self._log("Initialized", level="info")

    # =====================================================
    # COMMANDS
    # =====================================================
    def power_on(self):
        """Enable power supply"""
        if not self.powered_on:
            self.powered_on = True
            self.state = MotorState.IDLE
            self._log("Power ON", level="info")

    def power_off(self):
        """Disable power supply"""
        self.powered_on = False
        self.running = False
        self.state = MotorState.OFF
        self._log("Power OFF", level="info")

    def start(self):
        """Command motor to run"""
        if self.state == MotorState.IDLE:
            self.command_start = True
            self._log("Start command", level="info")

    def stop(self):
        """Command motor to stop"""
        if self.running:
            self.command_stop = True
            self._log("Stop command", level="info")

    def emergency_stop(self):
        """Force immediate stop"""
        self.powered_on = False
        self.running = False
        self.state = MotorState.OFF
        self.faults.append(FaultType.STALL)
        self._log("⚠️  EMERGENCY STOP", level="critical")

    # =====================================================
    # MAIN UPDATE CYCLE
    # =====================================================
    def update(self, voltage: float, current: float, speed: int, temperature: float):
        """
        Single control cycle.
        
        Args:
            voltage: Supply voltage (V)
            current: Motor current (A)
            speed: Motor speed (RPM)
            temperature: Winding temperature (°C)
        """
        # Validate inputs
        self._validate_inputs(voltage, current, speed, temperature)

        # Compute derived quantities
        self._compute_metrics()

        # Update history
        self._update_history()

        # State machine
        self._update_state()

        # Fault detection
        self._detect_faults()

    def _validate_inputs(self, voltage: float, current: float, speed: int, temperature: float):
        """Safe input validation"""
        # Voltage
        try:
            self.voltage = max(0.0, min(20.0, float(voltage)))
        except:
            self.voltage = 12.0

        # Current
        try:
            self.current = max(0.0, min(15.0, float(current)))
        except:
            self.current = 0.0

        # Speed
        try:
            self.speed = max(0, min(3500, int(speed)))
        except:
            self.speed = 0

        # Temperature
        try:
            self.temperature = max(-10.0, min(120.0, float(temperature)))
        except:
            self.temperature = 25.0

    # =====================================================
    # COMPUTE DERIVED QUANTITIES
    # =====================================================
    def _compute_metrics(self):
        """Calculate all derived metrics from V, I, T, N"""

        # 1. POWER ANALYSIS
        # ─────────────────────────────────────────────────
        self.power_input = self.voltage * self.current

        # Resistive loss: I²R (Joule heating in winding)
        i_squared_r = (self.current ** 2) * self.R

        # Friction loss (bearing, air resistance)
        # Rough estimate: 20% of input power at nominal load
        friction_loss = max(0.0, 0.20 * self.power_input)

        self.power_loss = i_squared_r + friction_loss
        self.power_mechanical = max(0.0, self.power_input - self.power_loss)

        # Efficiency
        if self.power_input > 0.1:
            self.efficiency = (self.power_mechanical / self.power_input) * 100.0
        else:
            self.efficiency = 0.0

        # 2. BACK-EMF
        # ─────────────────────────────────────────────────
        # Back-EMF = V - I×R
        self.back_emf = max(0.0, self.voltage - (self.current * self.R))

        # Back-EMF constant: Ke = Vb / ω
        if self.speed > 100:
            omega = (self.speed * 2 * 3.14159) / 60  # rad/s
            self.ke = self.back_emf / omega
        else:
            self.ke = 0.0

        # 3. LOAD & SPEED
        # ─────────────────────────────────────────────────
        self.speed_pct = min(100.0, (self.speed / self.rpm_max) * 100.0)
        self.load_pct = min(100.0, (self.current / self.current_nominal) * 100.0)

        # 4. TORQUE ESTIMATION
        # ─────────────────────────────────────────────────
        # τ = Km × I
        self.torque = self.km * self.current

        # 5. CURRENT RIPPLE (Brush condition indicator)
        # ─────────────────────────────────────────────────
        if len(self.current_history) >= 5 and self.current > 0.5:
            avg_i = sum(self.current_history) / len(self.current_history)
            variance = sum((i - avg_i) ** 2 for i in self.current_history) / len(self.current_history)
            std_dev = variance ** 0.5
            self.current_ripple = (std_dev / avg_i * 100.0) if avg_i > 0 else 0.0
        else:
            self.current_ripple = 0.0

        self.current_avg = sum(self.current_history) / len(self.current_history) if self.current_history else self.current

        # 6. THERMAL ANALYSIS
        # ─────────────────────────────────────────────────
        self.temp_rise = self.temperature - self.thresholds["temp_ambient"]

        if len(self.temp_history) >= 2:
            temp_delta = self.temperature - self.temp_history[-2]
            self.temp_rise_rate = temp_delta * self.update_rate
        else:
            self.temp_rise_rate = 0.0

    def _update_history(self):
        """Maintain sliding window for trend analysis"""
        self.current_history.append(self.current)
        self.speed_history.append(self.speed)
        self.temp_history.append(self.temperature)

    # =====================================================
    # STATE MACHINE
    # =====================================================
    def _update_state(self):
        """Simple state transitions"""
        elapsed = time.time() - self.state_entry_time

        # Power off → OFF
        if not self.powered_on:
            if self.state != MotorState.OFF:
                self.state = MotorState.OFF
                self.running = False
            return

        # OFF → IDLE (when powered on)
        if self.state == MotorState.OFF and self.powered_on:
            self.state = MotorState.IDLE
            self.state_entry_time = time.time()

        # IDLE + start command → RUNNING
        if self.state == MotorState.IDLE and self.command_start:
            self.running = True
            self.state = MotorState.RUNNING
            self.command_start = False
            self.state_entry_time = time.time()
            self._log("→ RUNNING", level="info")

        # RUNNING + stop command → IDLE
        if self.state == MotorState.RUNNING and self.command_stop:
            self.running = False
            self.state = MotorState.IDLE
            self.command_stop = False
            self.state_entry_time = time.time()
            self._log("→ IDLE", level="info")

        # FAULT → manual recovery
        if self.state == MotorState.FAULT:
            # Can only recover by power cycle
            if not self.powered_on:
                self.state = MotorState.OFF

    # =====================================================
    # FAULT DETECTION
    # =====================================================
    def _detect_faults(self):
        """Practical fault detection for 12V motor"""
        current_faults = []

        # ─────────────────────────────────────────────────
        # CRITICAL FAULTS
        # ─────────────────────────────────────────────────

        # 1. STALL: High current + zero speed = rotor locked
        if self.running and self.current > 5.0 and self.speed < 50:
            current_faults.append(FaultType.STALL)
            self._log(f"🔴 STALL: {self.current:.2f}A @ {self.speed} RPM", level="critical")

        # 2. THERMAL RUNAWAY: Temp > 90°C
        if self.temperature > self.thresholds["temp_critical"]:
            current_faults.append(FaultType.THERMAL_RUNAWAY)
            self._log(f"🔴 THERMAL RUNAWAY: {self.temperature:.1f}°C", level="critical")

        # 3. WINDING SHORT: High current but low back-EMF
        # Indicator: Current > 5A but back-EMF < 2V (motor not generating voltage)
        if (self.running and
            self.current > 5.0 and
            self.speed > 1000 and
            self.back_emf < 2.0):
            current_faults.append(FaultType.WINDING_SHORT)
            self._log(f"🔴 WINDING SHORT: I={self.current:.2f}A, Back-EMF={self.back_emf:.2f}V", level="critical")

        # ─────────────────────────────────────────────────
        # SEVERE FAULTS
        # ─────────────────────────────────────────────────

        # 4. OVERHEAT WARNING: 75-90°C
        if (self.thresholds["temp_warning"] <= self.temperature <
            self.thresholds["temp_critical"]):
            current_faults.append(FaultType.OVERHEAT)
            self._log(f"⚠️  OVERHEAT: {self.temperature:.1f}°C", level="warning")

        # 5. HIGH CURRENT SPIKE: > 6.5A
        if self.current > self.thresholds["current_spike"]:
            current_faults.append(FaultType.HIGH_CURRENT)
            self._log(f"⚠️  HIGH CURRENT: {self.current:.2f}A", level="warning")

        # 6. LOW VOLTAGE: < 10V (motor struggling)
        if self.voltage < self.thresholds["voltage_low"] and self.running:
            current_faults.append(FaultType.VOLTAGE_DROP)
            self._log(f"⚠️  VOLTAGE DROP: {self.voltage:.2f}V", level="warning")

        # ─────────────────────────────────────────────────
        # WARNING FAULTS
        # ─────────────────────────────────────────────────

        # 7. LOW EFFICIENCY: < 50% (bearing friction or internal damage)
        if (self.running and
            self.speed > 500 and
            self.current > 1.0 and
            self.efficiency < self.thresholds["efficiency_low"]):
            current_faults.append(FaultType.LOW_EFFICIENCY)
            self._log(f"⚠️  LOW EFFICIENCY: {self.efficiency:.1f}%", level="warning")

        # 8. BRUSH WEAR: High current ripple (> 30%)
        if (self.running and
            self.current > 1.0 and
            self.current_ripple > self.thresholds["ripple_worn"]):
            current_faults.append(FaultType.BRUSH_WEAR)
            self._log(f"⚠️  BRUSH WEAR: Ripple {self.current_ripple:.1f}%", level="warning")

        # ─────────────────────────────────────────────────
        # FAULT PERSISTENCE LOGIC
        # ─────────────────────────────────────────────────

        # Update fault counts
        for fault in FaultType:
            if fault in current_faults:
                self.fault_counts[fault] += 1
            else:
                self.fault_counts[fault] = max(0, self.fault_counts[fault] - 1)

        # Finalize fault list (persistent faults only)
        self.faults = [
            fault for fault, count in self.fault_counts.items()
            if count >= self.fault_persistence_threshold
        ]

        # Escalate critical faults immediately
        critical_faults = [FaultType.STALL, FaultType.THERMAL_RUNAWAY, FaultType.WINDING_SHORT]
        if any(f in self.faults for f in critical_faults):
            if self.state != MotorState.FAULT:
                self.state = MotorState.FAULT
                self.running = False
                self._log(f"🔴 FAULT STATE: {[f.value for f in self.faults]}", level="critical")

    # =====================================================
    # LOGGING
    # =====================================================
    def _log(self, msg: str, level: str = "info"):
        """Log entry"""
        self.logs.append({
            "time": time.time(),
            "level": level,
            "state": self.state.value,
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
        """Complete motor status snapshot"""
        return {
            "name": self.name,
            "timestamp": time.time(),
            "state": self.state.value,
            "powered_on": self.powered_on,
            "running": self.running,
            
            # Sensors (raw)
            "voltage_v": round(self.voltage, 2),
            "current_a": round(self.current, 2),
            "speed_rpm": self.speed,
            "temperature_c": round(self.temperature, 1),
            
            # Derived quantities
            "power_input_w": round(self.power_input, 2),
            "power_mechanical_w": round(self.power_mechanical, 2),
            "power_loss_w": round(self.power_loss, 2),
            "efficiency_pct": round(self.efficiency, 1),
            
            # Back-EMF & Motor
            "back_emf_v": round(self.back_emf, 2),
            "back_emf_constant_ke": round(self.ke, 4),
            "estimated_torque_nm": round(self.torque, 3),
            
            # Load & Speed
            "speed_pct_of_max": round(self.speed_pct, 1),
            "load_pct": round(self.load_pct, 1),
            
            # Current Analysis
            "current_ripple_pct": round(self.current_ripple, 1),
            "current_avg_a": round(self.current_avg, 2),
            
            # Thermal
            "temp_rise_from_ambient_c": round(self.temp_rise, 1),
            "temp_rise_rate_c_per_sec": round(self.temp_rise_rate, 3),
            
            # Health
            "faults": [f.value for f in self.faults],
            "health_score": self.get_health_score(),
        }

    def get_health_score(self) -> float:
        """Health score 0-100%"""
        score = 100.0

        # Temperature penalty
        if self.temperature > self.thresholds["temp_critical"]:
            score -= 100.0
        elif self.temperature > self.thresholds["temp_warning"]:
            penalty = (self.temperature - self.thresholds["temp_warning"]) / 15.0 * 30.0
            score -= penalty

        # Efficiency penalty
        if self.efficiency < self.thresholds["efficiency_low"]:
            score -= 20.0
        elif self.efficiency < 60.0:
            penalty = (60.0 - self.efficiency) / 10.0 * 15.0
            score -= penalty

        # Ripple penalty
        if self.current_ripple > self.thresholds["ripple_worn"]:
            penalty = min(15.0, (self.current_ripple - self.thresholds["ripple_worn"]) / 20.0 * 15.0)
            score -= penalty

        # Fault penalty
        score -= len(self.faults) * 10.0

        return max(0.0, min(100.0, score))

    def get_logs(self, limit: int = 50) -> List[Dict]:
        """Get recent logs"""
        return list(self.logs)[-limit:]


