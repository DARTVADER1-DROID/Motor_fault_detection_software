"""
Cache & Persistence Layer
=========================

Cache:
  Pure in-memory, TTL-based, thread-safe.  No external dependencies.
  Always available — zero configuration required.

Persistence:
  PostgreSQL via SQLAlchemy.  Fully optional.
  Only activated when DATABASE_URL env var is set to a real connection string.
  Every DB method catches its own exceptions and returns False/[] on failure,
  so a DB outage never crashes the service.

Data flow:
  ESP32  →  /update  →  ingest_sensor_data()  →  cache invalidated
                                               →  DB sensor reading (if DB set)
  Query  →  cache hit  →  return immediately
         →  cache miss →  compute from service  →  cache  →  return
  Flush  →  background thread every 60s  →  DB motor state + fault events
"""

from typing import Dict, List, Optional, Any
import threading
import time
import json
import os
import logging
from datetime import datetime, timedelta

log = logging.getLogger(__name__)

_sqlalchemy = None   # lazily imported so missing package doesn't break startup


def _try_import_sqlalchemy():
    global _sqlalchemy
    if _sqlalchemy is None:
        try:
            import sqlalchemy
            _sqlalchemy = sqlalchemy
        except ImportError:
            _sqlalchemy = False
    return _sqlalchemy if _sqlalchemy is not False else None


# ─────────────────────────────────────────────────────────────────────────────
# IN-MEMORY CACHE
# Thread-safe dict with per-key TTL expiry.  No serialisation overhead.
# Keys are plain strings; values are any Python object.
# ─────────────────────────────────────────────────────────────────────────────

class CacheLayer:
    """
    Pure in-memory, TTL-aware cache.

    Operations are O(1).  Expired entries are evicted lazily on access
    and proactively by a lightweight background reaper thread (every 30s).
    """

    # Default TTLs (seconds)
    TTL_MOTOR_STATUS   = 5
    TTL_CONNECTION_MAP = 2
    TTL_FAULT_SUMMARY  = 10

    def __init__(self):
        self._store: Dict[str, Any]   = {}   # key → value
        self._exp:   Dict[str, float] = {}   # key → expiry epoch
        self._lock   = threading.Lock()

        # Background reaper — keeps memory tidy without impacting hot path
        self._reaper = threading.Thread(
            target=self._reap_loop, daemon=True, name="CacheReaper")
        self._reaper.start()

        log.info("Cache: in-memory cache ready")

    # ── Public API ────────────────────────────────────────────────────────────

    def get(self, key: str) -> Optional[Any]:
        """Return cached value or None if missing/expired."""
        with self._lock:
            exp = self._exp.get(key)
            if exp is None:
                return None
            if time.monotonic() > exp:
                self._evict(key)
                return None
            return self._store.get(key)

    def set(self, key: str, value: Any, ttl: int) -> bool:
        """Store value with TTL (seconds).  Always succeeds."""
        with self._lock:
            self._store[key] = value
            self._exp[key]   = time.monotonic() + ttl
        return True

    def delete(self, key: str) -> bool:
        """Remove a key.  Silently ignores missing keys."""
        with self._lock:
            self._evict(key)
        return True

    def flush_all(self) -> int:
        """Clear the entire cache. Returns number of entries removed."""
        with self._lock:
            n = len(self._store)
            self._store.clear()
            self._exp.clear()
        return n

    def stats(self) -> Dict:
        """Return snapshot of cache size."""
        with self._lock:
            now   = time.monotonic()
            total = len(self._store)
            alive = sum(1 for exp in self._exp.values() if exp > now)
        return {"total_keys": total, "live_keys": alive, "backend": "in-memory"}

    @property
    def backend(self) -> str:
        """Always 'in-memory' — kept for compatibility."""
        return "in-memory"

    # ── Internal ──────────────────────────────────────────────────────────────

    def _evict(self, key: str):
        """Must be called with self._lock held."""
        self._store.pop(key, None)
        self._exp.pop(key, None)

    def _reap_loop(self):
        """Background thread: evict all expired keys every 30 s."""
        while True:
            time.sleep(30)
            try:
                now = time.monotonic()
                with self._lock:
                    expired = [k for k, exp in self._exp.items() if exp <= now]
                    for k in expired:
                        self._evict(k)
                if expired:
                    log.debug("Cache reaper evicted %d expired keys", len(expired))
            except Exception as e:
                log.debug("Cache reaper error: %s", e)


# ─────────────────────────────────────────────────────────────────────────────
# DATABASE LAYER  — PostgreSQL, fully optional
# ─────────────────────────────────────────────────────────────────────────────

class DatabaseLayer:
    """
    PostgreSQL persistence via SQLAlchemy.

    Only instantiated when DATABASE_URL env var contains a real URL.
    Every public method is wrapped in try/except and returns False/[] on failure —
    a DB outage never propagates to the endpoint layer.
    """

    def __init__(self, db_url: str):
        if _try_import_sqlalchemy() is None:
            raise RuntimeError("sqlalchemy is not installed (pip install sqlalchemy psycopg2-binary)")

        from sqlalchemy import (create_engine, Column, String, Float,
                                Integer, DateTime, Text)
        from sqlalchemy.orm import declarative_base, sessionmaker

        Base = declarative_base()

        class MotorRecord(Base):
            __tablename__ = "motors"
            motor_id     = Column(String(50), primary_key=True)
            name         = Column(String(100))
            state        = Column(String(20))
            powered_on   = Column(Integer)
            running      = Column(Integer)
            health_score = Column(Float)
            updated_at   = Column(DateTime, default=datetime.utcnow,
                                  onupdate=datetime.utcnow)

        class SensorReading(Base):
            __tablename__ = "sensor_readings"
            id               = Column(Integer, primary_key=True, autoincrement=True)
            motor_id         = Column(String(50), index=True)
            voltage          = Column(Float)
            current          = Column(Float)
            temperature      = Column(Float)
            speed            = Column(Integer)
            efficiency       = Column(Float)
            power_input      = Column(Float)
            power_mechanical = Column(Float)
            health_score     = Column(Float)
            timestamp        = Column(DateTime, default=datetime.utcnow, index=True)

        class FaultLog(Base):
            __tablename__ = "fault_logs"
            id          = Column(Integer, primary_key=True, autoincrement=True)
            motor_id    = Column(String(50), index=True)
            fault_type  = Column(String(50))
            severity    = Column(String(20))
            occurred_at = Column(DateTime, default=datetime.utcnow, index=True)
            details     = Column(Text)

        class CommandLog(Base):
            __tablename__ = "command_logs"
            id        = Column(Integer, primary_key=True, autoincrement=True)
            motor_id  = Column(String(50), index=True)
            command   = Column(String(50))
            status    = Column(String(20))
            queued_at = Column(DateTime, default=datetime.utcnow)

        engine = create_engine(
            db_url,
            pool_size=5,
            max_overflow=10,
            pool_pre_ping=True,
            echo=False,
        )
        Base.metadata.create_all(engine)

        self._Session     = sessionmaker(bind=engine)
        self._MotorRecord = MotorRecord
        self._Sensor      = SensorReading
        self._FaultLog    = FaultLog
        self._CommandLog  = CommandLog
        log.info("DB: connected — %s", db_url.split("@")[-1])

    def _s(self):
        return self._Session()

    # ── Motor state ───────────────────────────────────────────────────────────

    def save_motor_state(self, motor_id: str, motor) -> bool:
        try:
            s = self._s()
            r = s.query(self._MotorRecord).filter_by(motor_id=motor_id).first()
            if not r:
                r = self._MotorRecord(motor_id=motor_id, name=motor.name)
            r.state        = motor.state.value
            r.powered_on   = int(motor.powered_on)
            r.running      = int(motor.running)
            r.health_score = motor.get_health_score()
            r.updated_at   = datetime.utcnow()
            s.add(r); s.commit(); s.close()
            return True
        except Exception as e:
            log.warning("DB.save_motor_state(%s): %s", motor_id, e)
            return False

    # ── Sensor readings ───────────────────────────────────────────────────────

    def save_sensor_reading(self, motor_id: str, data: Dict) -> bool:
        try:
            s = self._s()
            s.add(self._Sensor(
                motor_id         = motor_id,
                voltage          = data.get("voltage", 0),
                current          = data.get("current", 0),
                temperature      = data.get("temperature", 25),
                speed            = data.get("speed", 0),
                efficiency       = data.get("efficiency", 0),
                power_input      = data.get("power_input", 0),
                power_mechanical = data.get("power_mechanical", 0),
                health_score     = data.get("health_score", 100),
                timestamp        = datetime.utcnow(),
            ))
            s.commit(); s.close()
            return True
        except Exception as e:
            log.warning("DB.save_sensor_reading(%s): %s", motor_id, e)
            return False

    def get_sensor_history(self, motor_id: str, hours: int = 1) -> List[Dict]:
        try:
            s     = self._s()
            since = datetime.utcnow() - timedelta(hours=min(hours, 24))
            rows  = (s.query(self._Sensor)
                     .filter(self._Sensor.motor_id == motor_id,
                             self._Sensor.timestamp >= since)
                     .order_by(self._Sensor.timestamp.desc())
                     .limit(1000).all())
            s.close()
            return [{"timestamp": r.timestamp.isoformat(), "voltage": r.voltage,
                     "current": r.current, "temperature": r.temperature,
                     "speed": r.speed, "efficiency": r.efficiency,
                     "health_score": r.health_score} for r in rows]
        except Exception as e:
            log.warning("DB.get_sensor_history(%s): %s", motor_id, e)
            return []

    # ── Fault log ─────────────────────────────────────────────────────────────

    def log_fault(self, motor_id: str, fault_type: str,
                  severity: str, details: Dict) -> bool:
        try:
            s = self._s()
            s.add(self._FaultLog(
                motor_id    = motor_id,
                fault_type  = fault_type,
                severity    = severity,
                occurred_at = datetime.utcnow(),
                details     = json.dumps(details, default=str),
            ))
            s.commit(); s.close()
            return True
        except Exception as e:
            log.warning("DB.log_fault(%s): %s", motor_id, e)
            return False

    def get_fault_history(self, motor_id: str, limit: int = 100) -> List[Dict]:
        try:
            s    = self._s()
            rows = (s.query(self._FaultLog)
                    .filter_by(motor_id=motor_id)
                    .order_by(self._FaultLog.occurred_at.desc())
                    .limit(limit).all())
            s.close()
            return [{"id": r.id, "fault_type": r.fault_type,
                     "severity": r.severity,
                     "occurred_at": r.occurred_at.isoformat(),
                     "details": json.loads(r.details) if r.details else {}}
                    for r in rows]
        except Exception as e:
            log.warning("DB.get_fault_history(%s): %s", motor_id, e)
            return []

    # ── Command log ───────────────────────────────────────────────────────────

    def log_command(self, motor_id: str, command: str,
                    status: str = "QUEUED") -> bool:
        try:
            s = self._s()
            s.add(self._CommandLog(motor_id=motor_id, command=command,
                                   status=status, queued_at=datetime.utcnow()))
            s.commit(); s.close()
            return True
        except Exception as e:
            log.warning("DB.log_command(%s,%s): %s", motor_id, command, e)
            return False

    def get_command_history(self, motor_id: str, limit: int = 100) -> List[Dict]:
        try:
            s    = self._s()
            rows = (s.query(self._CommandLog)
                    .filter_by(motor_id=motor_id)
                    .order_by(self._CommandLog.queued_at.desc())
                    .limit(limit).all())
            s.close()
            return [{"id": r.id, "command": r.command, "status": r.status,
                     "queued_at": r.queued_at.isoformat()} for r in rows]
        except Exception as e:
            log.warning("DB.get_command_history(%s): %s", motor_id, e)
            return []


# ─────────────────────────────────────────────────────────────────────────────
# CACHED MOTOR SERVICE
# Wraps MotorFleetService with cache + optional DB writes.
# ─────────────────────────────────────────────────────────────────────────────

class CachedMotorService:
    def __init__(self, motor_service, cache: CacheLayer,
                 db: Optional[DatabaseLayer]):
        self.motor_service = motor_service
        self.cache         = cache
        self.db            = db       # None when DB not configured
        self._running      = False
        self._flush_thread: Optional[threading.Thread] = None

    # ── Sensor ingestion ──────────────────────────────────────────────────────

    def update_motor_cached(self, motor_id: str, voltage: float,
                            current: float, speed: int, temperature: float):
        """
        Feed new sensor data into the state machine, invalidate cache,
        and (if DB configured) persist the reading.
        Raises ValueError if the motor is not registered.
        """
        self.motor_service.update_motor(motor_id, voltage, current,
                                        speed, temperature)
        if self.db is not None:
            try:
                wrapper = self.motor_service.motors.get(motor_id)
                if wrapper:
                    m = wrapper.motor
                    self.db.save_sensor_reading(motor_id, {
                        "voltage":          voltage,
                        "current":          current,
                        "temperature":      temperature,
                        "speed":            speed,
                        "efficiency":       m.efficiency,
                        "power_input":      m.power_input,
                        "power_mechanical": m.power_mechanical,
                        "health_score":     m.get_health_score(),
                    })
            except Exception as e:
                log.warning("CachedMotorService DB write on update: %s", e)

    # ── History queries (DB-backed; return [] when DB absent) ─────────────────

    def get_motor_history(self, motor_id: str, hours: int = 1) -> List[Dict]:
        if self.db is None:
            return []
        return self.db.get_sensor_history(motor_id, hours)

    def get_fault_history(self, motor_id: str) -> List[Dict]:
        if self.db is None:
            return []
        return self.db.get_fault_history(motor_id)

    def get_command_history(self, motor_id: str) -> List[Dict]:
        if self.db is None:
            return []
        return self.db.get_command_history(motor_id)

    # ── Periodic DB flush ─────────────────────────────────────────────────────

    def start_periodic_flush(self, interval: int = 60):
        if self.db is None:
            log.info("Periodic DB flush disabled — no database configured")
            return
        self._running = True
        self._flush_thread = threading.Thread(
            target=self._flush_loop, args=(interval,),
            daemon=True, name="DBFlush")
        self._flush_thread.start()
        log.info("Periodic DB flush started (every %ds)", interval)

    def stop(self):
        self._running = False

    def _flush_loop(self, interval: int):
        while self._running:
            try:
                with self.motor_service.lock:
                    snapshot = list(self.motor_service.motors.items())
                for motor_id, wrapper in snapshot:
                    self.db.save_motor_state(motor_id, wrapper.motor)
                    for fault in wrapper.motor.faults:
                        sev = ("CRITICAL"
                               if fault.value in ("STALL", "THERMAL_RUNAWAY", "WINDING_SHORT")
                               else "WARNING")
                        self.db.log_fault(motor_id, fault.value, sev, {
                            "state":       wrapper.motor.state.value,
                            "speed":       wrapper.motor.speed,
                            "temperature": wrapper.motor.temperature,
                        })
            except Exception as e:
                log.warning("DBFlush error: %s", e)
            time.sleep(interval)


# ─────────────────────────────────────────────────────────────────────────────
# FACTORY
# ─────────────────────────────────────────────────────────────────────────────

_PLACEHOLDER_URLS = {
    "postgresql://user:password@localhost/motor_db",
    "postgresql://user:password@localhost:5432/motor_db",
}


def init_cache_persistence(motor_service, db_url: str):
    """
    Initialise cache + optional DB + CachedMotorService.

    Cache:   always created, always works.
    DB:      created only when DATABASE_URL is a non-empty, non-placeholder URL.
             Any connection error is logged as a warning and db is set to None.

    Returns (cached_service, cache, db) — db may be None.
    """
    cache = CacheLayer()

    db: Optional[DatabaseLayer] = None
    url = (db_url or "").strip()

    if url and url not in _PLACEHOLDER_URLS:
        try:
            db = DatabaseLayer(url)
        except Exception as e:
            log.warning("DB unavailable — running without persistence: %s", e)
    else:
        log.info("DATABASE_URL not set — running without persistence")

    cs = CachedMotorService(motor_service, cache, db)
    cs.start_periodic_flush(interval=60)

    return cs, cache, db
