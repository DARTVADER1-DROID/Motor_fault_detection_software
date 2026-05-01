from enum import Enum
from collections import deque
import time


# =========================
# FSM STATES
# =========================
class MotorState(Enum):
    STOPPED = "STOPPED"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    NO_LOAD = "NO_LOAD"
    OVERLOAD = "OVERLOAD"
    FAULT = "FAULT"


# =========================
# FAULT TYPES (REAL SYSTEM)
# =========================
class FaultType(Enum):
    OVERHEAT = "OVERHEAT"
    ELECTRICAL_OVERLOAD = "ELECTRICAL_OVERLOAD"
    STALL_CONDITION = "STALL_CONDITION"
    ABNORMAL_RPM = "ABNORMAL_RPM"
    SENSOR_FAILURE = "SENSOR_FAILURE"
    OVER_VOLTAGE = "OVER_VOLTAGE"
    UNDER_VOLTAGE = "UNDER_VOLTAGE"


class Motor:

    def __init__(self, name="Motor-1"):

        # =========================
        # DIRECT DATA (INPUT ONLY)
        # =========================
        self.voltage = 0
        self.current = 0
        self.rpm = 0
        self.temperature = 0

        # =========================
        # DERIVED DATA (AUTO)
        # =========================
        self.power = 0
        self.load_index = 0

        # =========================
        # FSM STATE
        # =========================
        self.state = MotorState.STOPPED
        self.running = False

        # =========================
        # THRESHOLDS
        # =========================
        self.thresholds = {
            # voltage range
            "min_voltage": 200,
            "max_voltage": 260,

            # current
            "min_current_no_load": 0.4,
            "max_current_overload": 5.0,

            # rpm
            "min_rpm_running": 100,
            "stall_rpm": 20,

            # temperature
            "max_temp": 80
        }

        # =========================
        # FAULT SYSTEM
        # =========================
        self.faults = []
        self.log = deque(maxlen=200)

    # =====================================================
    # START MOTOR (FSM CONTROLLED)
    # =====================================================
    def start_motor(self):

        if self.state in [MotorState.STOPPED, MotorState.NO_LOAD]:
            self.running = True
            self.state = MotorState.STARTING
            self._log("START INITIATED")
        else:
            self._log(f"START BLOCKED: {self.state.value}")

    # =====================================================
    # STOP MOTOR
    # =====================================================
    def stop_motor(self):

        self.running = False
        self.state = MotorState.STOPPED
        self._log("MOTOR STOPPED")

    # =====================================================
    # UPDATE PIPELINE
    # =====================================================
    def update_data(self, voltage, current, rpm, temperature):

        if not self.running:
            return

        # 1. DIRECT DATA
        self.voltage = voltage
        self.current = current
        self.rpm = rpm
        self.temperature = temperature

        # 2. DERIVED DATA
        self._compute_derived()

        # 3. FSM UPDATE
        self._fsm_update()

        # 4. FAULT ENGINE
        self._fault_engine()

    # =====================================================
    # DERIVED DATA
    # =====================================================
    def _compute_derived(self):

        self.power = self.voltage * self.current

        self.load_index = min(1.0, max(0.0,
            (self.current / self.thresholds["max_current_overload"]) * 0.7 +
            (self.rpm / 1500) * 0.3
        ))

    # =====================================================
    # FSM LOGIC (BEHAVIOR ONLY)
    # =====================================================
    def _fsm_update(self):

        if self.state == MotorState.STOPPED:
            return

        # START → RUNNING
        if self.state == MotorState.STARTING and self.rpm > 0:
            self.state = MotorState.RUNNING

        # NO LOAD (valid state)
        if self.current < self.thresholds["min_current_no_load"] and self.rpm > 0:
            self.state = MotorState.NO_LOAD

        # OVERLOAD (stress state)
        elif self.current > self.thresholds["max_current_overload"]:
            self.state = MotorState.OVERLOAD

        # NORMAL RUN
        elif self.running:
            self.state = MotorState.RUNNING

    # =====================================================
    # FAULT ENGINE (STRICT ABNORMAL CONDITIONS ONLY)
    # =====================================================
    def _fault_engine(self):

        self.faults.clear()

        # 1. Overheat
        if self.temperature > self.thresholds["max_temp"]:
            self.faults.append(FaultType.OVERHEAT)

        # 2. Electrical overload (severe)
        if self.current > self.thresholds["max_current_overload"] * 1.2:
            self.faults.append(FaultType.ELECTRICAL_OVERLOAD)

        # 3. Stall condition
        if self.current > 0 and self.rpm < self.thresholds["stall_rpm"]:
            self.faults.append(FaultType.STALL_CONDITION)

        # 4. Abnormal RPM
        if self.state == MotorState.RUNNING and self.rpm < self.thresholds["min_rpm_running"]:
            self.faults.append(FaultType.ABNORMAL_RPM)

        # 5. Sensor failure
        if self.voltage <= 0 or self.current < 0:
            self.faults.append(FaultType.SENSOR_FAILURE)

        # 6. Over-voltage
        if self.voltage > self.thresholds["max_voltage"]:
            self.faults.append(FaultType.OVER_VOLTAGE)

        # 7. Under-voltage
        if 0 < self.voltage < self.thresholds["min_voltage"]:
            self.faults.append(FaultType.UNDER_VOLTAGE)

        # ESCALATION RULE
        if self.faults:
            self.state = MotorState.FAULT
            self._log(f"FAULTS: {[f.value for f in self.faults]}")
        else:
            self._log(f"OK: {self.state.value}")

    # =====================================================
    # LOGGING
    # =====================================================
    def _log(self, msg):
        self.log.append({
            "time": time.time(),
            "state": self.state.value,
            "message": msg
        })

    # =====================================================
    # STATUS
    # =====================================================
    def status(self):
        return {
            "state": self.state.value,
            "voltage": self.voltage,
            "current": self.current,
            "rpm": self.rpm,
            "temperature": self.temperature,
            "power": self.power,
            "load_index": self.load_index,
            "faults": [f.value for f in self.faults]
        }