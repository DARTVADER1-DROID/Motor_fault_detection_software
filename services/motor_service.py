from typing import Dict, List
import threading
import time
from collections import defaultdict

from models.motor_model import DCMotor12V


# =====================================================
# CONNECTION STATUS
# =====================================================
class ConnectionStatus:
    CONNECTED = "CONNECTED"
    TIMEOUT = "TIMEOUT"


# =====================================================
# WRAPPER (SERVICE METADATA ONLY)
# =====================================================
class MotorWrapper:
    def __init__(self, motor: DCMotor12V, timeout_sec: float):
        self.motor = motor
        self.timeout_sec = timeout_sec

        self.last_update_time = time.time()
        self.connection_status = ConnectionStatus.CONNECTED

    def heartbeat(self):
        """Called when ESP32 sends data"""
        self.last_update_time = time.time()
        self.connection_status = ConnectionStatus.CONNECTED

    def update_connection(self):
        """Check timeout"""
        if time.time() - self.last_update_time > self.timeout_sec:
            self.connection_status = ConnectionStatus.TIMEOUT
        else:
            self.connection_status = ConnectionStatus.CONNECTED


# =====================================================
# FLEET SERVICE
# =====================================================
class MotorFleetService:
    """
    🔥 Fleet Supervisor Layer

    Responsibilities:
    - Manage multiple motors
    - Track ESP32 connectivity
    - Route sensor data
    - Enforce command safety
    - Provide fleet insights

    DOES NOT:
    ❌ Override motor state machine
    ❌ Recalculate physics
    ❌ Detect faults (model does it)
    """

    def __init__(self, timeout_sec: float = 3.0, monitor_interval: float = 1.0):
        self.motors: Dict[str, MotorWrapper] = {}

        self.timeout_sec = timeout_sec
        self.monitor_interval = monitor_interval

        self.lock = threading.Lock()
        self.running = False

    # =====================================================
    # REGISTRATION
    # =====================================================
    def register_motor(self, motor_id: str, motor: DCMotor12V):
        with self.lock:
            self.motors[motor_id] = MotorWrapper(motor, self.timeout_sec)

    def remove_motor(self, motor_id: str):
        with self.lock:
            if motor_id in self.motors:
                del self.motors[motor_id]

    # =====================================================
    # ESP32 DATA INGESTION
    # =====================================================
    def ingest_sensor_data(
        self,
        motor_id: str,
        voltage: float,
        current: float,
        speed: int,
        temperature: float
    ):
        with self.lock:
            wrapper = self._get(motor_id)

            # update connection heartbeat
            wrapper.heartbeat()

            # forward to model
            wrapper.motor.update(voltage, current, speed, temperature)

    # =====================================================
    # COMMAND CONTROL (SAFE)
    # =====================================================
    def power_on(self, motor_id: str):
        with self.lock:
            wrapper = self._get(motor_id)
            wrapper.motor.power_on()

    def power_off(self, motor_id: str):
        with self.lock:
            wrapper = self._get(motor_id)
            wrapper.motor.power_off()

    def start_motor(self, motor_id: str) -> Dict:
        with self.lock:
            wrapper = self._get(motor_id)
            motor = wrapper.motor

            # CONNECTION CHECK (CRITICAL)
            if wrapper.connection_status != ConnectionStatus.CONNECTED:
                return {"error": "Motor not connected"}

            # FAULT CHECK
            if motor.state.value == "FAULT":
                return {"error": "Motor in FAULT state"}

            motor.start()
            return {"status": "start command sent"}

    def stop_motor(self, motor_id: str):
        with self.lock:
            wrapper = self._get(motor_id)
            wrapper.motor.stop()

    def emergency_stop(self, motor_id: str):
        with self.lock:
            wrapper = self._get(motor_id)
            wrapper.motor.emergency_stop()

    # =====================================================
    # FLEET CONTROL
    # =====================================================
    def fleet_power_on(self):
        with self.lock:
            for w in self.motors.values():
                w.motor.power_on()

    def fleet_shutdown(self):
        with self.lock:
            for w in self.motors.values():
                w.motor.emergency_stop()

    def fleet_start(self):
        results = {}
        with self.lock:
            for mid, wrapper in self.motors.items():
                if wrapper.connection_status != ConnectionStatus.CONNECTED:
                    results[mid] = "NOT CONNECTED"
                    continue

                if wrapper.motor.state.value == "FAULT":
                    results[mid] = "FAULT"
                    continue

                wrapper.motor.start()
                results[mid] = "STARTED"

        return results

    # =====================================================
    # STATUS
    # =====================================================
    def get_motor_status(self, motor_id: str) -> Dict:
        with self.lock:
            wrapper = self._get(motor_id)

            data = wrapper.motor.status()
            data["connection_status"] = wrapper.connection_status
            data["last_update_sec"] = round(
                time.time() - wrapper.last_update_time, 2
            )

            return data

    def get_fleet_status(self) -> Dict[str, Dict]:
        with self.lock:
            return {
                mid: self.get_motor_status(mid)
                for mid in self.motors
            }

    def get_connection_map(self) -> Dict[str, str]:
        with self.lock:
            return {
                mid: w.connection_status
                for mid, w in self.motors.items()
            }

    def get_fault_summary(self) -> Dict:
        summary = defaultdict(int)

        with self.lock:
            for wrapper in self.motors.values():
                for f in wrapper.motor.faults:
                    summary[f.value] += 1

        return dict(summary)

    # =====================================================
    # MONITOR LOOP (CONNECTION WATCHER)
    # =====================================================
    def start_monitoring(self):
        self.running = True
        thread = threading.Thread(target=self._monitor_loop, daemon=True)
        thread.start()

    def stop_monitoring(self):
        self.running = False

    def _monitor_loop(self):
        while self.running:
            with self.lock:
                for wrapper in self.motors.values():
                    wrapper.update_connection()

            time.sleep(self.monitor_interval)

    # =====================================================
    # UTIL
    # =====================================================
    def _get(self, motor_id: str) -> MotorWrapper:
        if motor_id not in self.motors:
            raise ValueError(f"Motor '{motor_id}' not registered")
        return self.motors[motor_id]