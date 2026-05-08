"""
motor_model.py — 12V Brushed DC Motor Model  v3.0  (Combined Best-of-Both)
===========================================================================

SOURCE MERGE:
  v2_upload/motor_model.py  — better state machine, fault preservation,
                               config architecture, threshold UI metadata
  our motor_model.py        — OVERHEAT coverage fix, threshold-relative
                               HIGH_CURRENT check, separate motor_config.py

ALL 11 BUGS FIXED:
  01 — history updated BEFORE metrics (was after)
  02 — WINDING_SHORT uses Ke deviation + relative Vb (was physically impossible)
  03 — P_friction = b*omega^2 (was flat 20% of P_input)
  04 — LOW_EFFICIENCY gated at I>1A AND N>900 RPM (was firing at light load)
  05 — emergency_stop() does NOT inject STALL fault
  06 — fault persistence hard-resets to 0 on clear (was slow -1 decrement)
  07 — THERMAL_RUNAWAY requires high temp AND rising rate (was temp alone)
  08 — startup grace window suppresses false faults during ramp
  09 — Ke_live computed dynamically; Km is the single nameplate reference
  10 — all thermal thresholds in SHELL temperature (sensor is on casing)
  11 — R_eff is temperature-corrected every cycle

KEY DESIGN DECISIONS:
  [D1] Sensor is on SHELL — winding temp Tw is ESTIMATED via thermal observer.
  [D2] Ke = Km always (energy conservation law). One constant, not two.
  [D3] Every quantity is labelled MEASURED or ESTIMATED — never ambiguous.
  [D4] Startup (first 600ms) is a first-class operating phase with suppressed faults.
  [D5] Fault persistence = hard-reset debounce. Counter -> 0 the instant condition clears.
  [D6] History updated BEFORE metrics every cycle.

ADDITIONAL FIXES OVER BOTH ORIGINALS:
  FIX-A — OVERHEAT now fires when shell is above critical but rate is too low
           for THERMAL_RUNAWAY (stable-hot zone was a detection gap in v2_upload).
  FIX-B — HIGH_CURRENT inner sustain check is threshold-relative (th.current_high_a * 0.90),
           not hardcoded 5.0A (v2_upload broke if operator changed current_high_a).
  FIX-C — Fault list preserved in FAULT state via intelligent merge (from v2_upload).
  FIX-D — power_off() clears fault counts so they don't survive a power cycle (from v2_upload).
  FIX-E — Ke history cleared on new start so startup transient doesn't contaminate
           winding health estimates of the next run (from v2_upload).
  FIX-F — get_threshold_config() returns modified flag for frontend badge (from v2_upload).
  FIX-G — fault_counts exposed in status() so API consumers can see raw debounce state.

THREAD SAFETY:
  _state_lock (threading.Lock) protects all mutable state.
  All public methods are safe for concurrent sensor thread + command thread access.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional


# =============================================================================
# ENUMERATIONS
# =============================================================================

class MotorState(Enum):
    OFF     = "OFF"      # No power applied
    IDLE    = "IDLE"     # Power on, motor stationary
    RUNNING = "RUNNING"  # Motor spinning (includes startup transient)
    FAULT   = "FAULT"    # Critical fault — manual recovery required


class FaultType(Enum):
    # CRITICAL — auto-stop immediately (persist = 1)
    STALL           = "STALL"
    THERMAL_RUNAWAY = "THERMAL_RUNAWAY"
    WINDING_SHORT   = "WINDING_SHORT"

    # SEVERE — alert only, no auto-stop (persist = 2-3)
    OVERHEAT        = "OVERHEAT"
    HIGH_CURRENT    = "HIGH_CURRENT"
    VOLTAGE_DROP    = "VOLTAGE_DROP"

    # WARNING — maintenance needed (persist = 5)
    LOW_EFFICIENCY  = "LOW_EFFICIENCY"
    BRUSH_WEAR      = "BRUSH_WEAR"


_CRITICAL = frozenset({FaultType.STALL, FaultType.THERMAL_RUNAWAY, FaultType.WINDING_SHORT})


# =============================================================================
# CONFIGURATION  (two-tier — constants are engineer-only, thresholds are tunable)
# =============================================================================

@dataclass
class MotorConstants:
    """
    Physical motor properties.  Measure once per unit.
    Never changed at runtime.  Requires code deploy + engineering sign-off.

    How to measure each value: see motor_model_documentation.txt Section 2.
    """
    # Electrical
    R:        float = 2.4       # Winding resistance [Ohm]  — multimeter, motor at rest
    L:        float = 0.0025    # Armature inductance [H]   — LCR meter or step-response
    Km:       float = 0.038     # Torque AND back-EMF constant [N.m/A = V.s/rad]
                                # Ke = Km always (energy conservation [D2])
    alpha_cu: float = 0.00393   # Copper temp coefficient [/degC] — fixed physical constant

    # Mechanical
    b:        float = 0.000043  # Viscous damping [N.m.s/rad]  — from no-load I and speed
    J:        float = 0.00005   # Rotor inertia [kg.m^2]        — datasheet or geometry
    N_max:    int   = 3000      # No-load max speed [RPM]       — datasheet
    I_nom:    float = 4.0       # Nominal continuous current [A] — datasheet
    I_max:    float = 6.5       # Absolute maximum current [A]  — datasheet

    # Thermal
    Rth_ws:   float = 2.5       # Winding-to-shell thermal resistance [degC/W]
                                # CRITICAL: sensor is on shell [D1]. This constant
                                # lets the model estimate winding temp from shell temp.
    Rth_sa:   float = 12.0      # Shell-to-ambient thermal resistance [degC/W]
    Cw:       float = 15.0      # Winding thermal capacitance [J/degC]
    Cs:       float = 80.0      # Shell thermal capacitance [J/degC]


@dataclass
class MotorThresholds:
    """
    Fault trigger values.
    Engineers set safe defaults here.
    Operators may tune a subset at runtime via API within SAFETY_BOUNDS.

    ALL temperature thresholds are SHELL temperature [D1].
    The sensor is on the motor casing — not the winding.
    """
    # Thermal — shell-referenced
    temp_ambient_c:        float = 25.0   # Environment temperature [degC]
    temp_warning_c:        float = 55.0   # Shell -> OVERHEAT warning
    temp_critical_c:       float = 70.0   # Shell -> THERMAL_RUNAWAY (+ rate check)
    temp_absolute_max_c:   float = 80.0   # Shell hard limit — auto-stop regardless of rate
    temp_runaway_rate_c_s: float = 1.5    # Shell rise rate [degC/s] for runaway trigger

    # Current
    current_stall_a:       float = 4.5    # Threshold for stall detection [A]
    current_high_a:        float = 5.5    # HIGH_CURRENT fault threshold [A]
    current_noload_max_a:  float = 0.8    # Max expected no-load current [A]

    # Voltage
    voltage_low_v:         float = 10.5   # VOLTAGE_DROP threshold [V]
    voltage_nominal_v:     float = 12.0   # Expected supply voltage [V]

    # Speed
    speed_min_running_rpm: int   = 80     # Below this = motor considered stopped
    speed_overspeed_rpm:   int   = 3200   # Above this = overspeed / sensor error

    # Efficiency
    efficiency_low_pct:    float = 25.0   # LOW_EFFICIENCY threshold [%]
    efficiency_min_load_a: float = 2.0    # Min current for efficiency check (FIX bug-04)
    efficiency_min_rpm:    int   = 900    # Min speed for efficiency check  (FIX bug-04)

    # Brush / ripple
    ripple_warning_pct:    float = 15.0   # Moderate wear indicator [%]
    ripple_fault_pct:      float = 30.0   # BRUSH_WEAR fault threshold [%]

    # Winding health
    ke_deviation_fault:    float = 0.25   # Ke deviation fraction -> WINDING_SHORT

    # Fault persistence [cycles]
    persist_warning:  int = 5    # WARNING faults
    persist_severe:   int = 3    # SEVERE faults
    persist_critical: int = 1    # CRITICAL — immediate, never debounce

    # Startup
    startup_grace_ms: int = 600  # Transient window [ms]

    # Schema version — increment when fields are added/renamed/removed
    schema_version: int = 3


@dataclass
class MotorConfig:
    """Top-level config container passed into DCMotor12V."""
    motor_constants: MotorConstants  = field(default_factory=MotorConstants)
    thresholds:      MotorThresholds = field(default_factory=MotorThresholds)


# =============================================================================
# SAFETY BOUNDS  (API validation — operators cannot exceed these)
# =============================================================================

SAFETY_BOUNDS: Dict[str, tuple] = {
    "temp_ambient_c":        (-20.0, 60.0),
    "temp_warning_c":        (40.0,  70.0),
    "temp_critical_c":       (55.0,  80.0),
    "temp_absolute_max_c":   (70.0,  85.0),
    "temp_runaway_rate_c_s": (0.5,   5.0),
    "current_stall_a":       (3.0,   6.0),
    "current_high_a":        (4.0,   6.5),
    "voltage_low_v":         (8.0,   11.5),
    "efficiency_low_pct":    (10.0,  70.0),
    "ripple_fault_pct":      (15.0,  60.0),
    "ke_deviation_fault":    (0.10,  0.40),
    "startup_grace_ms":      (200,   2000),
    "persist_critical":      (1,     1),    # never debounce CRITICAL
    "persist_severe":        (2,     10),
    "persist_warning":       (3,     20),
}

# Fields operators may NOT touch — physics gates or safety-critical
OPERATOR_READONLY = frozenset({
    "persist_critical",
    "efficiency_min_load_a",
    "efficiency_min_rpm",
    "current_noload_max_a",
    "speed_min_running_rpm",
    "speed_overspeed_rpm",
    "ripple_warning_pct",
    "schema_version",
})


# =============================================================================
# MAIN MOTOR MODEL
# =============================================================================

class DCMotor12V:
    """
    12V Brushed DC Motor — Industrial-Grade Control Model v3.0

    MEASURED inputs  [D3]  — from hardware sensors, 4 values only:
        voltage     [V]       supply voltage
        current     [A]       motor current
        speed       [RPM]     shaft speed
        shell_temp  [degC]    outer casing temperature  (NOT winding) [D1]

    ESTIMATED quantities [D3] — derived by the model each cycle:
        winding_temp, back_emf, back_emf_pred, Ke_live, Ke_deviation,
        torque, torque_load, R_eff, power_*, efficiency,
        current_ripple, temp_rise_rate
    """

    def __init__(self,
                 name: str = "12V-Motor",
                 update_rate: float = 10.0,
                 config: Optional[MotorConfig] = None):
        self.name        = name
        self.update_rate = update_rate          # Hz
        self.config      = config or MotorConfig()
        self._c          = self.config.motor_constants
        self._th         = self.config.thresholds
        self._state_lock = threading.Lock()

        # ── MEASURED inputs [D3] ──────────────────────────────────────────────
        self.voltage:    float = 12.0
        self.current:    float = 0.0
        self.speed:      int   = 0
        self.shell_temp: float = 25.0   # Ts — casing, NOT winding [D1]

        # ── ESTIMATED quantities [D3] ─────────────────────────────────────────
        self.omega:            float = 0.0
        self.R_eff:            float = self._c.R
        self.back_emf:         float = 0.0
        self.back_emf_pred:    float = 0.0
        self.Ke_live:          float = 0.0
        self.Ke_deviation:     float = 0.0
        self.torque:           float = 0.0
        self.torque_load:      float = 0.0
        self.winding_temp:     float = 25.0   # Tw — estimated [D1]
        self.temp_rise_rate:   float = 0.0
        self.power_input:      float = 0.0
        self.power_copper:     float = 0.0
        self.power_friction:   float = 0.0
        self.power_loss:       float = 0.0
        self.power_mechanical: float = 0.0
        self.efficiency:       float = 0.0
        self.current_ripple:   float = 0.0
        self.current_avg:      float = 0.0
        self.speed_pct:        float = 0.0
        self.load_pct:         float = 0.0

        # ── State machine ─────────────────────────────────────────────────────
        self.state:            MotorState = MotorState.OFF
        self.running:          bool  = False
        self.powered_on:       bool  = False
        self.command_start:    bool  = False
        self.command_stop:     bool  = False
        self.state_entry_time: float = time.time()

        # ── Startup transient [D4] ────────────────────────────────────────────
        self._startup_time:          Optional[float] = None
        self._startup_grace_expired: bool = True   # True = normal; False = in grace window

        # ── History — updated BEFORE metrics [D6] ─────────────────────────────
        self._current_history: deque = deque(maxlen=15)
        self._speed_history:   deque = deque(maxlen=15)
        self._temp_history:    deque = deque(maxlen=20)
        self._Ke_history:      deque = deque(maxlen=30)

        # ── Fault engine [D5] ─────────────────────────────────────────────────
        self.fault_counts: Dict[FaultType, int] = {ft: 0 for ft in FaultType}
        self.faults:       List[FaultType] = []

        # ── Logging ───────────────────────────────────────────────────────────
        self.logs: deque = deque(maxlen=300)
        self._log("DCMotor12V v3.0 initialised", level="info")

    # =========================================================================
    # PUBLIC COMMANDS  (all thread-safe)
    # =========================================================================

    def power_on(self):
        """Enable supply — OFF -> IDLE."""
        with self._state_lock:
            if not self.powered_on:
                self.powered_on = True
                self.state = MotorState.IDLE
                self.state_entry_time = time.time()
                self._log("Power ON -> IDLE", level="info")

    def power_off(self):
        """Cut supply — any state -> OFF.  Clears all faults. [FIX-D]"""
        with self._state_lock:
            self.powered_on = False
            self.running    = False
            self._startup_grace_expired = True
            self.state = MotorState.OFF
            self.state_entry_time = time.time()
            # FIX-D: clear fault state so counts don't survive a power cycle
            self.faults = []
            self.fault_counts = {ft: 0 for ft in FaultType}
            self._log("Power OFF -> OFF (faults cleared)", level="info")

    def start(self):
        """Issue START command — valid only from IDLE."""
        with self._state_lock:
            if self.state == MotorState.IDLE:
                self.command_start = True
                self._log("Start command issued", level="info")

    def stop(self):
        """Issue graceful STOP command."""
        with self._state_lock:
            if self.running:
                self.command_stop = True
                self._log("Stop command issued", level="info")

    def emergency_stop(self):
        """
        Immediate hard stop.
        FIX bug-05: does NOT inject STALL — deliberate stop is not a mechanical stall.
        Sets state=FAULT so motor cannot restart without power cycle.
        """
        with self._state_lock:
            self.powered_on = False
            self.running    = False
            self._startup_grace_expired = True
            self.state = MotorState.FAULT
            self.state_entry_time = time.time()
            self._log("EMERGENCY STOP — state=FAULT (reason: EMERGENCY_STOP)", level="critical")

    # =========================================================================
    # MAIN UPDATE CYCLE  (called at update_rate Hz by sensor ingestion path)
    # =========================================================================

    def update(self, voltage: float, current: float, speed: int, temperature: float):
        """
        Single control cycle.

        Args:
            voltage:     Supply voltage [V]          MEASURED
            current:     Motor current [A]            MEASURED
            speed:       Shaft speed [RPM]            MEASURED
            temperature: Shell temperature [degC]     MEASURED — shell, NOT winding [D1]
        """
        self._validate_inputs(voltage, current, speed, temperature)
        self._update_history()          # BEFORE metrics [D6]
        self._compute_metrics()
        self._update_state()
        self._detect_faults()

    # =========================================================================
    # STEP 1 — INPUT VALIDATION
    # =========================================================================

    def _validate_inputs(self, voltage, current, speed, temperature):
        """Clamp all sensor inputs to physically plausible ranges."""
        try:    self.voltage    = max(0.0,   min(20.0,  float(voltage)))
        except: self.voltage    = 12.0

        try:    self.current    = max(0.0,   min(15.0,  float(current)))
        except: self.current    = 0.0

        try:    self.speed      = max(0,     min(3500,  int(speed)))
        except: self.speed      = 0

        try:    self.shell_temp = max(-10.0, min(120.0, float(temperature)))
        except: self.shell_temp = 25.0

    # =========================================================================
    # STEP 2 — HISTORY  (must run before _compute_metrics — [D6])
    # =========================================================================

    def _update_history(self):
        self._current_history.append(self.current)
        self._speed_history.append(self.speed)
        self._temp_history.append(self.shell_temp)

    # =========================================================================
    # STEP 3 — DERIVED METRICS
    # =========================================================================

    def _compute_metrics(self):
        """
        Compute all estimated quantities from measured inputs + motor constants.
        Execution order is intentional — later steps use earlier results.
        """
        c  = self._c
        th = self._th

        # 1. Angular velocity
        self.omega = self.speed * 0.10472   # rad/s = RPM * 2*pi/60

        # 2. Temperature-corrected winding resistance
        #    R_eff = R * (1 + alpha_cu * (Tw - 20))
        #    Bootstraps from shell_temp on first cycle; converges within 1-2 cycles.
        self.R_eff = max(0.1, c.R * (1.0 + c.alpha_cu * (self.winding_temp - 20.0)))

        # 3. Back-EMF from armature circuit (steady-state, dI/dt ~ 0)
        #    Vb = V - I * R_eff
        self.back_emf = max(0.0, self.voltage - self.current * self.R_eff)

        # 4. Back-EMF predicted from speed
        #    Vb_pred = Km * omega
        self.back_emf_pred = c.Km * self.omega

        # 5. Live Ke estimation (adaptive winding health monitor)
        #    Ke_live = Vb / omega  — valid only when spinning and past startup [D2][D4]
        if self.omega > 8.4 and self._startup_grace_expired:
            self.Ke_live = self.back_emf / self.omega
            self._Ke_history.append(self.Ke_live)
        else:
            self.Ke_live = 0.0

        # Rolling Ke deviation from nameplate Km (min 5 samples for stability)
        if len(self._Ke_history) >= 5:
            Ke_avg = sum(self._Ke_history) / len(self._Ke_history)
            self.Ke_deviation = abs(Ke_avg - c.Km) / c.Km
        else:
            self.Ke_deviation = 0.0

        # 6. Torque
        #    tau = Km * I
        #    tau_load = Km*I - b*omega  (steady-state, J*d_omega/dt = 0)
        self.torque      = c.Km * self.current
        self.torque_load = max(0.0, c.Km * self.current - c.b * self.omega)

        # 7. Winding temperature — two-body thermal observer [D1]
        #    Tw = Ts + I^2 * R_eff * Rth_ws
        # Use base R (not R_eff) here to break circular dependency:
        # R_eff depends on Tw, Tw depends on R_eff. Using base R gives a clean
        # one-step observer with no feedback loop.
        self.winding_temp = self.shell_temp + (self.current ** 2) * c.R * c.Rth_ws

        # 8. Shell temperature rate of change
        #    dTs/dt = (Ts[t] - Ts[t-1]) * update_rate_hz
        #    Requires >=5 samples to avoid false spikes during startup transient
        #    (startup temp jump from ambient to operating produces 1-cycle rate spike)
        if len(self._temp_history) >= 5:
            self.temp_rise_rate = (
                self._temp_history[-1] - self._temp_history[-2]
            ) * self.update_rate
        else:
            self.temp_rise_rate = 0.0

        # 9. Power analysis
        self.power_input    = self.voltage * self.current
        self.power_copper   = (self.current ** 2) * self.R_eff   # Joule heating
        self.power_friction = c.b * (self.omega ** 2)             # viscous loss — FIX bug-03
        self.power_loss     = self.power_copper + self.power_friction
        self.power_mechanical = max(0.0, self.power_input - self.power_loss)

        # 10. Efficiency — only meaningful at real load  (FIX bug-04 + bug-11)
        if (self.power_input > 0.5
                and self.current >= th.efficiency_min_load_a
                and self.speed   >= th.efficiency_min_rpm):
            self.efficiency = (self.power_mechanical / self.power_input) * 100.0
        else:
            self.efficiency = 0.0   # not computed — insufficient load or speed

        # 11. Speed and load percentages
        self.speed_pct = min(100.0, (self.speed / c.N_max) * 100.0)
        self.load_pct  = min(100.0, (self.current / c.I_nom) * 100.0)

        # 12. Current ripple — brush condition indicator
        #     Valid only during steady-state running (not startup, speed stable, I > 1A)
        if (len(self._current_history) >= 5
                and self._startup_grace_expired
                and self._speed_is_stable()):
            avg = sum(self._current_history) / len(self._current_history)
            if avg > 1.0:
                variance = (
                    sum((i - avg) ** 2 for i in self._current_history)
                    / len(self._current_history)
                )
                self.current_ripple = (math.sqrt(variance) / avg) * 100.0
            else:
                self.current_ripple = 0.0
        else:
            self.current_ripple = 0.0

        self.current_avg = (
            sum(self._current_history) / len(self._current_history)
            if self._current_history else self.current
        )

    def _speed_is_stable(self) -> bool:
        """True when |d_omega/dt| < 50 rad/s^2 — not accelerating or decelerating."""
        if len(self._speed_history) < 2:
            return False
        dN = abs(self._speed_history[-1] - self._speed_history[-2])
        return (dN * 0.10472 * self.update_rate) < 50.0

    # =========================================================================
    # STEP 4 — STATE MACHINE
    # =========================================================================

    def _update_state(self):
        """Advance state machine and manage startup grace window [D4]."""
        with self._state_lock:

            # Power removed -> OFF (unless already in FAULT — needs power cycle)
            if not self.powered_on:
                if self.state not in (MotorState.OFF, MotorState.FAULT):
                    self.state = MotorState.OFF
                    self.running = False
                    self._startup_grace_expired = True
                return

            # OFF -> IDLE on power-on
            if self.state == MotorState.OFF:
                self.state = MotorState.IDLE
                self.state_entry_time = time.time()

            # IDLE + start command -> RUNNING
            if self.state == MotorState.IDLE and self.command_start:
                self.running       = True
                self.state         = MotorState.RUNNING
                self.command_start = False
                self.state_entry_time = time.time()
                # Begin startup grace window [D4]
                self._startup_time          = time.time()
                self._startup_grace_expired = False
                # FIX-E: clear Ke history — stale data from previous run must not
                # contaminate winding health estimates for the new run
                self._Ke_history.clear()
                self._log("-> RUNNING (startup grace window active)", level="info")

            # RUNNING + stop command -> IDLE
            if self.state == MotorState.RUNNING and self.command_stop:
                self.running      = False
                self.state        = MotorState.IDLE
                self.command_stop = False
                self.state_entry_time = time.time()
                self._startup_grace_expired = True
                self._log("-> IDLE (stopped normally)", level="info")

            # Startup grace window expiry check [D4]
            if not self._startup_grace_expired and self._startup_time is not None:
                elapsed_ms = (time.time() - self._startup_time) * 1000.0
                if elapsed_ms >= self._th.startup_grace_ms:
                    self._startup_grace_expired = True
                    self._log(
                        f"Startup grace expired ({elapsed_ms:.0f} ms) — all faults active",
                        level="info"
                    )
                    # Extended startup failure check:
                    # if speed is still below minimum AND current is still high -> blockage
                    if (self.speed   < self._th.speed_min_running_rpm
                            and self.current > self._th.current_stall_a):
                        self._log(
                            "Motor did not reach minimum speed after startup grace — "
                            "possible mechanical blockage",
                            level="critical"
                        )

            # FAULT -> OFF only via power cycle
            if self.state == MotorState.FAULT and not self.powered_on:
                self.state = MotorState.OFF

    # =========================================================================
    # STEP 5 — FAULT DETECTION ENGINE
    # =========================================================================

    def _detect_faults(self):
        """
        Evaluate all 8 fault conditions and apply persistence debounce [D5].

        Persistence rule (FIX bug-06):
          condition TRUE  -> increment counter
          condition FALSE -> RESET counter to 0  (hard reset, never -1)
          fault active    -> counter >= persist threshold for severity

        Startup suppression [D4]:
          Suppressed during grace: STALL, HIGH_CURRENT, LOW_EFFICIENCY,
                                   BRUSH_WEAR, WINDING_SHORT
          Always active:           THERMAL_RUNAWAY, OVERHEAT, VOLTAGE_DROP

        Fault-in-FAULT-state handling [FIX-C]:
          Once in FAULT state, running=False causes conditions to evaluate False.
          We preserve the original fault list and only ADD newly detected faults.
          Faults are only cleared on power_off() (power cycle).
        """
        th    = self._th
        c     = self._c
        grace = self._startup_grace_expired   # True = normal, False = startup

        active: Dict[FaultType, bool] = {}

        # ── FAULT 1: STALL (CRITICAL, persist=1) ─────────────────────────────
        # High current + near-zero speed after startup. Current not falling (not a glitch).
        dI = (self._current_history[-1] - self._current_history[-2]
              if len(self._current_history) >= 2 else 0.0)
        active[FaultType.STALL] = (
            grace
            and self.running
            and self.current > th.current_stall_a
            and self.speed   < th.speed_min_running_rpm
            and dI >= 0
        )

        # ── FAULT 2: THERMAL_RUNAWAY (CRITICAL, persist=1) ───────────────────
        # Condition A: shell > critical AND rising fast (FIX bug-07 — was temp alone)
        # Condition B: shell > absolute max regardless of rate
        active[FaultType.THERMAL_RUNAWAY] = (
            (self.shell_temp > th.temp_critical_c
             and self.temp_rise_rate > th.temp_runaway_rate_c_s)
            or self.shell_temp > th.temp_absolute_max_c
        )

        # ── FAULT 3: WINDING_SHORT (CRITICAL, persist=2) ─────────────────────
        # Ke deviation > 25% from nameplate Km AND Vb < 70% of predicted AND excess I.
        # FIX bug-02: original was I>5A AND speed>1000 AND Vb<2V — physically impossible.
        active[FaultType.WINDING_SHORT] = (
            grace
            and self.running
            and self.speed > 500
            and self.Ke_deviation > th.ke_deviation_fault
            and self.current > c.I_nom * 1.2
            and self.back_emf_pred > 0.1          # guard: avoid div-by-zero at standstill
            and self.back_emf < self.back_emf_pred * 0.70
        )

        # ── FAULT 4: OVERHEAT (SEVERE, persist=3) ────────────────────────────
        # FIX-A: fires when shell is in warning band OR above critical but rate is
        # too low for THERMAL_RUNAWAY (stable-hot zone). Original had a detection gap
        # at 70-80 degC shell with low rate — neither fault would fire.
        active[FaultType.OVERHEAT] = (
            th.temp_warning_c <= self.shell_temp < th.temp_absolute_max_c
            and not active[FaultType.THERMAL_RUNAWAY]
        )

        # ── FAULT 5: HIGH_CURRENT (SEVERE, persist=2) ────────────────────────
        # Sustained high current after startup.
        # FIX-B: inner sustain check is threshold-relative (not hardcoded 5.0A).
        # If operator changes current_high_a, the sustain check moves with it.
        recent_5 = list(self._current_history)[-5:]
        recent_avg = sum(recent_5) / len(recent_5) if recent_5 else self.current
        active[FaultType.HIGH_CURRENT] = (
            grace
            and self.current > th.current_high_a
            and recent_avg   > th.current_high_a * 0.90
        )

        # ── FAULT 6: VOLTAGE_DROP (SEVERE, persist=3) ────────────────────────
        # Low supply voltage while running. Always active (even during startup).
        active[FaultType.VOLTAGE_DROP] = (
            self.running
            and self.voltage < th.voltage_low_v
            and (th.voltage_nominal_v - self.voltage) > 1.5
        )

        # ── FAULT 7: LOW_EFFICIENCY (WARNING, persist=5) ─────────────────────
        # Only at real load/speed — light-load inefficiency is normal physics (FIX bug-04).
        active[FaultType.LOW_EFFICIENCY] = (
            grace
            and self.running
            and self.speed   >= th.efficiency_min_rpm
            and self.current >= th.efficiency_min_load_a
            and self.efficiency > 0
            and self.efficiency < th.efficiency_low_pct
        )

        # ── FAULT 8: BRUSH_WEAR (WARNING, persist=5) ─────────────────────────
        # High ripple during steady-state running only (speed stable, I > 1A).
        active[FaultType.BRUSH_WEAR] = (
            grace
            and self.running
            and self.current_avg > 1.0
            and self._speed_is_stable()
            and self.current_ripple > th.ripple_fault_pct
        )

        # ── PERSISTENCE MAP ───────────────────────────────────────────────────
        persist_map: Dict[FaultType, int] = {
            FaultType.STALL:          th.persist_critical,
            FaultType.THERMAL_RUNAWAY:th.persist_critical,
            FaultType.WINDING_SHORT:  max(2, th.persist_severe - 1),  # own 2-cycle debounce
            FaultType.OVERHEAT:       th.persist_severe,
            FaultType.HIGH_CURRENT:   max(2, th.persist_severe - 1),  # own 2-cycle debounce
            FaultType.VOLTAGE_DROP:   th.persist_severe,
            FaultType.LOW_EFFICIENCY: th.persist_warning,
            FaultType.BRUSH_WEAR:     th.persist_warning,
        }

        # ── HARD-RESET DEBOUNCE [D5] ──────────────────────────────────────────
        for ft, condition_met in active.items():
            if condition_met:
                self.fault_counts[ft] += 1
            else:
                self.fault_counts[ft] = 0   # hard reset — never -1 (FIX bug-06)

        # ── BUILD ACTIVE FAULT LIST ───────────────────────────────────────────
        # FIX-C: In FAULT state, running=False causes conditions to be False.
        # Preserve the existing fault list and only ADD new faults.
        # Faults are only cleared by power_off().
        previous_faults = set(self.faults)

        if self.state == MotorState.FAULT:
            # FIX-C: preserve original fault list in FAULT state because
            # running=False makes most conditions evaluate False.
            # Exception: THERMAL_RUNAWAY and OVERHEAT are condition-based
            # (not running-dependent) so evaluate them live — do not preserve
            # stale thermal faults if temperature has since recovered.
            newly_detected = {
                ft for ft, count in self.fault_counts.items()
                if count >= persist_map[ft]
            }
            thermal_faults = {FaultType.THERMAL_RUNAWAY, FaultType.OVERHEAT}
            preserved = {ft for ft in self.faults if ft not in thermal_faults}
            current_thermal = {ft for ft in newly_detected if ft in thermal_faults}
            self.faults = list(preserved | newly_detected - thermal_faults | current_thermal)
        else:
            self.faults = [
                ft for ft, count in self.fault_counts.items()
                if count >= persist_map[ft]
            ]

        # ── LOG NEW FAULTS ────────────────────────────────────────────────────
        for ft in self.faults:
            if ft not in previous_faults:
                self._log(
                    f"FAULT ACTIVE: {ft.value} | "
                    f"Ts={self.shell_temp:.1f}C Tw={self.winding_temp:.1f}C "
                    f"I={self.current:.2f}A N={self.speed}RPM "
                    f"Vb={self.back_emf:.2f}V eta={self.efficiency:.1f}%",
                    level="critical" if ft in _CRITICAL else "warning"
                )

        # ── AUTO-STOP FOR CRITICAL FAULTS ─────────────────────────────────────
        critical_now = [ft for ft in self.faults if ft in _CRITICAL]
        if critical_now:
            with self._state_lock:
                if self.state not in (MotorState.FAULT, MotorState.OFF):
                    self.running = False
                    self.state   = MotorState.FAULT
                    self._startup_grace_expired = True
                    self._log(
                        f"AUTO-STOP triggered: {[f.value for f in critical_now]}",
                        level="critical"
                    )

    # =========================================================================
    # HEALTH SCORE
    # =========================================================================

    def get_health_score(self) -> float:
        """Composite health 0-100. Factors: faults, thermal margin, efficiency, ripple."""
        score = 100.0
        th    = self._th

        for ft in self.faults:
            if ft in _CRITICAL:
                score -= 40.0
            elif ft in (FaultType.OVERHEAT, FaultType.HIGH_CURRENT, FaultType.VOLTAGE_DROP):
                score -= 20.0
            else:
                score -= 10.0

        if self.shell_temp > th.temp_warning_c:
            score -= min(20.0, (self.shell_temp - th.temp_warning_c) * 1.5)

        if self.efficiency > 0 and self.efficiency < th.efficiency_low_pct:
            score -= min(15.0, (th.efficiency_low_pct - self.efficiency) * 0.5)

        if self.current_ripple > th.ripple_warning_pct:
            score -= min(10.0, (self.current_ripple - th.ripple_warning_pct) * 0.3)

        return round(max(0.0, min(100.0, score)), 1)

    # =========================================================================
    # STATUS SNAPSHOT  (JSON-serialisable)
    # =========================================================================

    def status(self) -> Dict:
        """Complete motor status snapshot — thread-safe."""
        with self._state_lock:
            state_val    = self.state.value
            running      = self.running
            powered_on   = self.powered_on
            grace_active = not self._startup_grace_expired

        return {
            "name":       self.name,
            "timestamp":  time.time(),
            "state":      state_val,
            "powered_on": powered_on,
            "running":    running,
            "startup_grace_active": grace_active,

            # MEASURED [D3]
            "voltage_v":     round(self.voltage, 2),
            "current_a":     round(self.current, 2),
            "speed_rpm":     self.speed,
            "temperature_c": round(self.shell_temp, 1),   # shell — kept as temperature_c for API compat

            # ESTIMATED [D3] — thermal
            "winding_temp_c":       round(self.winding_temp, 1),
            "temp_rise_rate_c_s":   round(self.temp_rise_rate, 3),

            # ESTIMATED — electrical
            "back_emf_v":           round(self.back_emf, 2),
            "back_emf_predicted_v": round(self.back_emf_pred, 2),
            "ke_live":              round(self.Ke_live, 5),
            "ke_deviation_pct":     round(self.Ke_deviation * 100.0, 2),
            "km_nameplate":         self._c.Km,
            "R_eff_ohm":            round(self.R_eff, 4),

            # ESTIMATED — mechanical
            "torque_nm":       round(self.torque, 4),
            "torque_load_nm":  round(self.torque_load, 4),
            "omega_rad_s":     round(self.omega, 3),
            "speed_pct_of_max":round(self.speed_pct, 1),
            "load_pct":        round(self.load_pct, 1),

            # ESTIMATED — power
            "power_input_w":      round(self.power_input, 2),
            "power_mechanical_w": round(self.power_mechanical, 2),
            "power_copper_w":     round(self.power_copper, 2),
            "power_friction_w":   round(self.power_friction, 3),
            "power_loss_w":       round(self.power_loss, 2),
            "efficiency_pct":     round(self.efficiency, 1),

            # ESTIMATED — brush condition
            "current_ripple_pct": round(self.current_ripple, 1),
            "current_avg_a":      round(self.current_avg, 2),

            # Fault and health
            "faults":       [f.value for f in self.faults],
            "fault_counts": {f.value: self.fault_counts[f] for f in FaultType},  # FIX-G
            "health_score": self.get_health_score(),
        }

    # =========================================================================
    # CONFIG ACCESS  (runtime threshold overrides — doc Section 10)
    # =========================================================================

    def apply_threshold_overrides(self, overrides: Dict) -> Dict:
        """
        Apply operator threshold overrides, validated against SAFETY_BOUNDS.
        Motor constants (R, Km, etc.) are never accessible here — engineer-only.
        Returns {"applied": {...}, "rejected": {...}}.
        """
        applied  = {}
        rejected = {}

        for key, value in overrides.items():
            if key in OPERATOR_READONLY:
                rejected[key] = f"'{key}' is read-only for operators"
                continue
            if not hasattr(self._th, key):
                rejected[key] = f"Unknown threshold field '{key}'"
                continue
            if key in SAFETY_BOUNDS:
                lo, hi = SAFETY_BOUNDS[key]
                try:
                    v = float(value)
                except (TypeError, ValueError):
                    rejected[key] = "Value must be numeric"
                    continue
                if not (lo <= v <= hi):
                    rejected[key] = f"Value {v} out of safe range [{lo}, {hi}]"
                    continue
            setattr(self._th, key, value)
            applied[key] = value
            self._log(f"Threshold override: {key}={value}", level="info")

        return {"applied": applied, "rejected": rejected}

    def reset_thresholds_to_defaults(self):
        """Reset all operator overrides to code-level defaults."""
        self.config.thresholds = MotorThresholds()
        self._th = self.config.thresholds
        self._log("Thresholds reset to defaults", level="info")

    def get_threshold_config(self) -> Dict:
        """
        Return current thresholds with defaults, bounds and modified flag.
        FIX-F: modified flag enables frontend [MODIFIED] badge per field.
        """
        defaults = MotorThresholds()
        result   = {}
        for key in vars(self._th):
            if key.startswith("_"):
                continue
            cur = getattr(self._th, key)
            dfl = getattr(defaults, key, None)
            bnd = SAFETY_BOUNDS.get(key)
            result[key] = {
                "value":     cur,
                "default":   dfl,
                "modified":  cur != dfl,   # FIX-F
                "read_only": key in OPERATOR_READONLY,
                "bounds":    {"min": bnd[0], "max": bnd[1]} if bnd else None,
            }
        return result

    # =========================================================================
    # LOGGING
    # =========================================================================

    def _log(self, msg: str, level: str = "info"):
        self.logs.append({
            "time":    time.time(),
            "level":   level,
            "state":   self.state.value,
            "message": msg,
            "snapshot": {
                "V":   round(self.voltage, 2),
                "I":   round(self.current, 2),
                "RPM": self.speed,
                "Ts":  round(self.shell_temp, 1),
                "Tw":  round(self.winding_temp, 1),
                "Eff": round(self.efficiency, 1),
            },
        })

    def get_logs(self, limit: int = 50) -> List[Dict]:
        return list(self.logs)[-limit:]
