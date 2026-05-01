from typing import Dict, List, Callable, Any, Optional
from collections import defaultdict, deque
import threading
import time
from dataclasses import dataclass

from models.motor_model import DCMotor12V, MotorState, FaultType


# =====================================================
# EVENT SYSTEM
# =====================================================
@dataclass
class MotorEvent:
    motor_id: str
    event_type: str
    payload: Dict[str, Any]
    timestamp: float


# =====================================================
# RULE ENGINE (SAFETY + VALIDATION)
# =====================================================
class RuleEngine:
    """
    Central safety validation engine.
    All motor actions pass through here.
    """

    def __init__(self):
        self.rules: List[Callable[[DCMotor12V], Optional[str]]] = []

    def add_rule(self, rule: Callable[[DCMotor12V], Optional[str]]):
        self.rules.append(rule)

    def evaluate(self, motor: DCMotor12V) -> List[str]:
        violations = []
        for rule in self.rules:
            result = rule(motor)
            if result:
                violations.append(result)
        return violations


# =====================================================
# MOTOR SERVICE (FLEET CONTROLLER)
# =====================================================
class MotorService:
    """
    Fleet-level motor control system.

    Responsibilities:
    ─────────────────────────────────────────
    1. Manage multiple motors (fleet)
    2. Process ESP32 updates (high frequency)
    3. Maintain real-time cache (low latency reads)
    4. Async DB queue (batch persistence)
    5. Rule-based safety system
    6. Fault aggregation + system health
    7. Event-driven architecture
    8. Monitoring loop (SCADA-style)
    """

    def __init__(self):
        # =========================
        # REGISTRY
        # =========================
        self.motors: Dict[str, DCMotor12V] = {}

        # =========================
        # THREAD SAFETY
        # =========================
        self.lock = threading.Lock()

        # =========================
        # CACHE (FAST READ LAYER)
        # =========================
        self.cache: Dict[str, Dict] = {}

        # =========================
        # DB QUEUE (ASYNC WRITE)
        # =========================
        self.db_queue = deque(maxlen=5000)

        # =========================
        # EVENT SUBSCRIBERS
        # =========================
        self.subscribers: List[Callable[[MotorEvent], None]] = []

        # =========================
        # RULE ENGINE
        # =========================
        self.rules = RuleEngine()

        # =========================
        # MONITORING
        # =========================
        self.monitoring = False

        # =========================
        # FLEET HEALTH
        # =========================
        self.fleet_health_score = 100.0

        # =========================
        # FAULT AGGREGATION
        # =========================
        self.global_faults = defaultdict(int)

        # =========================
        # HISTORY STORAGE (LIGHTWEIGHT)
        # =========================
        self.history: Dict[str, deque] = defaultdict(lambda: deque(maxlen=200))

        # =========================
        # INIT DEFAULT RULES
        # =========================
        self._init_default_rules()

    # =====================================================
    # RULES SETUP
    # =====================================================
    def _init_default_rules(self):

        # Voltage safety
        self.rules.add_rule(
            lambda m: "LOW_VOLTAGE" if m.voltage < 10 and m.running else None
        )

        # Thermal safety
        self.rules.add_rule(
            lambda m: "OVERHEAT" if m.temperature > 80 else None
        )

        # Current spike
        self.rules.add_rule(
            lambda m: "OVERCURRENT" if m.current > m.current_max else None
        )

        # Speed anomaly (stall detection)
        self.rules.add_rule(
            lambda m: "STALL" if m.running and m.speed < 30 and m.current > 3 else None
        )

        # Efficiency degradation
        self.rules.add_rule(
            lambda m: "LOW_EFFICIENCY" if m.efficiency < 40 else None
        )

    # =====================================================
    # SUBSCRIPTIONS
    # =====================================================
    def subscribe(self, callback: Callable[[MotorEvent], None]):
        self.subscribers.append(callback)

    def _emit(self, motor_id: str, event_type: str, payload: Dict):
        event = MotorEvent(motor_id, event_type, payload, time.time())

        for sub in self.subscribers:
            try:
                sub(event)
            except:
                pass

    # =====================================================
    # MOTOR REGISTRATION
    # =====================================================
    def register_motor(self, motor_id: str, motor: DCMotor12V):
        with self.lock:
            self.motors[motor_id] = motor
            self.cache[motor_id] = motor.status()

        self._emit(motor_id, "REGISTERED", motor.status())

    def remove_motor(self, motor_id: str):
        with self.lock:
            self.motors.pop(motor_id, None)
            self.cache.pop(motor_id, None)

        self._emit(motor_id, "REMOVED", {})

    # =====================================================
    # LIFECYCLE CONTROL
    # =====================================================
    def power_on(self, motor_id: str):
        m = self._get(motor_id)
        m.power_on()
        self._emit(motor_id, "POWER_ON", m.status())

    def power_off(self, motor_id: str):
        m = self._get(motor_id)
        m.power_off()
        self._emit(motor_id, "POWER_OFF", m.status())

    def start_motor(self, motor_id: str):
        m = self._get(motor_id)

        if self._safe_to_start(m):
            m.start()
            self._emit(motor_id, "START", m.status())
        else:
            self._emit(motor_id, "START_BLOCKED", m.status())

    def stop_motor(self, motor_id: str):
        m = self._get(motor_id)
        m.stop()
        self._emit(motor_id, "STOP", m.status())

    def emergency_stop(self, motor_id: str):
        m = self._get(motor_id)
        m.emergency_stop()
        self._emit(motor_id, "EMERGENCY_STOP", m.status())

    # =====================================================
    # BATCH OPERATIONS (FLEET CONTROL)
    # =====================================================
    def start_all(self):
        for mid in list(self.motors.keys()):
            self.start_motor(mid)

    def stop_all(self):
        for mid in list(self.motors.keys()):
            self.stop_motor(mid)

    def emergency_stop_all(self):
        for mid in list(self.motors.keys()):
            self.emergency_stop(mid)

    # =====================================================
    # SENSOR UPDATE PIPELINE (ESP32 ENTRY POINT)
    # =====================================================
    def update_motor(self, motor_id: str, v: float, i: float, s: int, t: float):
        m = self._get(motor_id)

        # update model
        m.update(v, i, s, t)

        # cache update (FAST READ LAYER)
        self.cache[motor_id] = m.status()

        # history
        self.history[motor_id].append(m.status())

        # rule evaluation
        violations = self.rules.evaluate(m)

        if violations:
            for v in violations:
                self.global_faults[v] += 1
                self._emit(motor_id, "RULE_VIOLATION", {"rule": v})

        # DB queue (ASYNC persistence)
        self.db_queue.append(m.status())

        # telemetry event
        self._emit(motor_id, "UPDATE", m.status())

    # =====================================================
    # SAFE START LOGIC
    # =====================================================
    def _safe_to_start(self, m: DCMotor12V) -> bool:
        if m.state == MotorState.FAULT:
            return False
        if m.temperature > 75:
            return False
        if m.voltage < 10:
            return False
        return True

    # =====================================================
    # MONITORING LOOP (SCADA STYLE)
    # =====================================================
    def start_monitoring(self, interval: float = 1.0):
        if self.monitoring:
            return

        self.monitoring = True
        threading.Thread(target=self._monitor_loop, args=(interval,), daemon=True).start()

    def _monitor_loop(self, interval: float):
        while self.monitoring:
            self._compute_fleet_health()
            self._detect_fleet_anomalies()
            time.sleep(interval)

    # =====================================================
    # FLEET HEALTH ENGINE
    # =====================================================
    def _compute_fleet_health(self):
        if not self.motors:
            self.fleet_health_score = 100
            return

        total = 0
        score = 0

        for m in self.motors.values():
            total += 1
            score += m.get_health_score()

        self.fleet_health_score = score / total

    # =====================================================
    # ANOMALY DETECTION (FLEET LEVEL)
    # =====================================================
    def _detect_fleet_anomalies(self):
        for mid, m in self.motors.items():

            if m.state == MotorState.RUNNING and m.speed < 20:
                self._emit(mid, "STALL_WARNING", m.status())

            if m.temperature > 85:
                self._emit(mid, "THERMAL_WARNING", m.status())

            if m.efficiency < 35:
                self._emit(mid, "EFFICIENCY_CRITICAL", m.status())

    # =====================================================
    # STATUS APIs
    # =====================================================
    def get_motor_status(self, motor_id: str):
        return self.cache.get(motor_id)

    def get_all_status(self):
        return self.cache

    def get_fleet_health(self):
        return {
            "fleet_health": self.fleet_health_score,
            "motor_count": len(self.motors),
            "faults": dict(self.global_faults),
        }

    def get_motor_history(self, motor_id: str):
        return list(self.history[motor_id])

    # =====================================================
    # UTIL
    # =====================================================
    def _get(self, motor_id: str) -> DCMotor12V:
        if motor_id not in self.motors:
            raise ValueError(f"Motor {motor_id} not found")
        return self.motors[motor_id]