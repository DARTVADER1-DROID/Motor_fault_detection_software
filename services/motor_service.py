from typing import Dict, List
from collections import defaultdict
import threading
import time

from models.motor_model import DCMotor12V, MotorState


class MotorService:
    """
    Service Layer for DC Motor System

    Responsibilities:
    ─────────────────────────────
    - Motor lifecycle control
    - Safety guardrails
    - Multi-motor orchestration
    - Fault aggregation
    - Monitoring loop (SCADA style)
    """

    def __init__(self):
        # Type-safe motor registry
        self.motors: Dict[str, DCMotor12V] = {}

        # Thread safety (future-proof for FastAPI/WebSocket)
        self.lock = threading.Lock()

        self.running = False

    # =====================================================
    # REGISTRATION
    # =====================================================
    def register_motor(self, motor_id: str, motor: DCMotor12V):
        self.motors[motor_id] = motor

    def remove_motor(self, motor_id: str):
        if motor_id in self.motors:
            del self.motors[motor_id]

    # =====================================================
    # LIFECYCLE CONTROL (MATCHING MODEL)
    # =====================================================
    def power_on(self, motor_id: str):
        motor = self._get_motor(motor_id)
        motor.power_on()

    def power_off(self, motor_id: str):
        motor = self._get_motor(motor_id)
        motor.power_off()

    def start_motor(self, motor_id: str):
        motor = self._get_motor(motor_id)

        if self._is_safe_to_start(motor):
            motor.start()   # ✅ MATCHES MODEL
        else:
            motor._log("START BLOCKED by service layer", level="warning")

    def stop_motor(self, motor_id: str):
        motor = self._get_motor(motor_id)
        motor.stop()        # ✅ MATCHES MODEL

    def emergency_stop(self, motor_id: str):
        motor = self._get_motor(motor_id)
        motor.emergency_stop()

    # =====================================================
    # SENSOR UPDATE PIPELINE
    # =====================================================
    def update_motor(
        self,
        motor_id: str,
        voltage: float,
        current: float,
        speed: int,
        temperature: float
    ):
        motor = self._get_motor(motor_id)

        # service-level safety gate
        if motor.state == MotorState.OFF:
            return

        motor.update(voltage, current, speed, temperature)

    # =====================================================
    # SAFETY GATE (SERVICE OVERRIDE LAYER)
    # =====================================================
    def _is_safe_to_start(self, motor: DCMotor12V) -> bool:

        # block if motor is already in fault state
        if motor.state == MotorState.FAULT:
            return False

        # thermal protection
        if motor.temperature > 70:
            return False

        # undervoltage protection
        if motor.voltage < 10:
            return False

        return True

    # =====================================================
    # STATUS APIs
    # =====================================================
    def get_motor_status(self, motor_id: str) -> Dict:
        motor = self._get_motor(motor_id)
        return motor.status()

    def get_all_status(self) -> Dict[str, Dict]:
        return {mid: m.status() for mid, m in self.motors.items()}

    # =====================================================
    # FAULT AGGREGATION
    # =====================================================
    def get_all_faults(self) -> Dict[str, List]:
        return {
            mid: m.status()["faults"]
            for mid, m in self.motors.items()
        }

    def get_fault_summary(self) -> Dict:
        summary = defaultdict(int)

        for motor in self.motors.values():
            for fault in motor.faults:
                summary[fault.value] += 1

        return dict(summary)

    # =====================================================
    # MONITORING LOOP (SCADA STYLE)
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
    # SERVICE-LEVEL HEALTH OVERSIGHT
    # =====================================================
    def _health_check(self, motor: DCMotor12V):

        # efficiency degradation detection
        if motor.efficiency < 40:
            motor._log("SERVICE WARNING: Low efficiency", level="warning")

        # stall trend detection
        if motor.state == MotorState.RUNNING and motor.speed < 30:
            motor._log("SERVICE WARNING: Possible stall trend", level="warning")

    # =====================================================
    # INTERNAL GETTER
    # =====================================================
    def _get_motor(self, motor_id: str) -> DCMotor12V:
        if motor_id not in self.motors:
            raise ValueError(f"Motor '{motor_id}' not registered")
        return self.motors[motor_id]