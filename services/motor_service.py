from typing import Dict, List, Protocol
from collections import defaultdict
import threading
import time

from models.motor_model import DCMotor12V, MotorState


# =====================================================
# CONTRACT (REMOVES IDE + TYPE ISSUES FOREVER)
# =====================================================
class MotorContract(Protocol):
    state: MotorState
    voltage: float
    current: float
    speed: int
    temperature: float
    efficiency: float
    faults: List

    def power_on(self): ...
    def power_off(self): ...
    def start(self): ...
    def stop(self): ...
    def emergency_stop(self): ...
    def update(self, voltage, current, speed, temperature): ...
    def status(self) -> dict: ...
    def _log(self, msg: str, level: str = "info"): ...


# =====================================================
# SERVICE LAYER (IMPROVED)
# =====================================================
class MotorService:
    """
    Industrial-grade Motor Service Layer

    Improvements:
    ─────────────────────────────
    - Strong contract typing (Protocol)
    - Thread-safe motor operations
    - Safe state transitions
    - Better fault aggregation
    - Monitoring-ready architecture
    """

    def __init__(self):
        self.motors: Dict[str, MotorContract] = {}

        # global lock for atomic operations
        self.lock = threading.Lock()

        self.running = False

    # =====================================================
    # REGISTRATION
    # =====================================================
    def register_motor(self, motor_id: str, motor: MotorContract):
        with self.lock:
            self.motors[motor_id] = motor

    def remove_motor(self, motor_id: str):
        with self.lock:
            self.motors.pop(motor_id, None)

    # =====================================================
    # LIFECYCLE CONTROL (ATOMIC SAFE)
    # =====================================================
    def power_on(self, motor_id: str):
        with self.lock:
            self._get_motor(motor_id).power_on()

    def power_off(self, motor_id: str):
        with self.lock:
            self._get_motor(motor_id).power_off()

    def start_motor(self, motor_id: str):
        with self.lock:
            motor = self._get_motor(motor_id)

            if self._is_safe_to_start(motor):
                motor.start()
            else:
                motor._log("START BLOCKED (service safety gate)", level="warning")

    def stop_motor(self, motor_id: str):
        with self.lock:
            self._get_motor(motor_id).stop()

    def emergency_stop(self, motor_id: str):
        with self.lock:
            self._get_motor(motor_id).emergency_stop()

    # =====================================================
    # SENSOR PIPELINE (LIGHTWEIGHT + SAFE)
    # =====================================================
    def update_motor(self, motor_id: str, v: float, i: float, s: int, t: float):
        motor = self._get_motor(motor_id)

        if motor.state == MotorState.OFF:
            return

        motor.update(v, i, s, t)

    # =====================================================
    # SMART SAFETY GATE (IMPROVED LOGIC)
    # =====================================================
    def _is_safe_to_start(self, motor: MotorContract) -> bool:
        """
        Instead of hard thresholds only,
        we use a weighted safety score idea.
        """

        score = 100

        # Fault penalty
        score -= len(motor.faults) * 25

        # Thermal penalty
        if motor.temperature > 80:
            score -= 50
        elif motor.temperature > 70:
            score -= 20

        # Voltage penalty
        if motor.voltage < 10:
            score -= 40

        # Efficiency penalty
        if motor.efficiency < 50:
            score -= 20

        return score >= 60

    # =====================================================
    # STATUS APIs
    # =====================================================
    def get_motor_status(self, motor_id: str) -> Dict:
        return self._get_motor(motor_id).status()

    def get_all_status(self) -> Dict[str, Dict]:
        return {mid: m.status() for mid, m in self.motors.items()}

    # =====================================================
    # FAULT ANALYTICS (IMPROVED)
    # =====================================================
    def get_all_faults(self) -> Dict[str, List]:
        return {mid: m.faults for mid, m in self.motors.items()}

    def get_fault_summary(self) -> Dict:
        summary = defaultdict(int)

        for motor in self.motors.values():
            for f in motor.faults:
                summary[f.value] += 1

        return dict(summary)

    # =====================================================
    # MONITORING LOOP (NON-BLOCKING READY)
    # =====================================================
    def run_monitoring_loop(self, interval: float = 1.0):
        self.running = True

        while self.running:
            with self.lock:
                for motor in self.motors.values():
                    self._health_check(motor)

            time.sleep(interval)

    def stop_monitoring(self):
        self.running = False

    # =====================================================
    # HEALTH OVERSIGHT (SERVICE INTELLIGENCE)
    # =====================================================
    def _health_check(self, motor: MotorContract):

        if motor.efficiency < 40:
            motor._log("SERVICE: critical efficiency drop", "warning")

        if motor.state == MotorState.RUNNING and motor.speed < 30:
            motor._log("SERVICE: stall trend detected", "warning")

        if motor.temperature > 85:
            motor._log("SERVICE: thermal stress high", "warning")

    # =====================================================
    # INTERNAL
    # =====================================================
    def _get_motor(self, motor_id: str) -> MotorContract:
        if motor_id not in self.motors:
            raise ValueError(f"Motor '{motor_id}' not registered")
        return self.motors[motor_id]