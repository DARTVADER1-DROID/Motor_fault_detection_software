# Motor Control System v3.0

Industrial-grade 12V Brushed DC Motor fleet management system.
Built for ESP32 microcontrollers with a FastAPI backend and web dashboard.

## What's in this release

- **motor_model.py v3.0** — Physics-correct DC motor model. All 11 original bugs fixed.
  Shell-sensor thermal observer, adaptive Ke estimation, startup grace window,
  hard-reset fault persistence, and correct viscous friction model.
- **motor_service.py v3.0** — Full fleet management with complete fleet control suite
  (fleet_power_on, fleet_start, fleet_stop, fleet_power_off, fleet_emergency).
- **endpoints.py v3.0** — Full REST API including runtime config management
  (GET/PATCH/DELETE /motor/{id}/config) and all fleet control endpoints.
- **cache_persistence.py** — In-memory TTL cache + optional PostgreSQL persistence.

---

## Project Structure

```
motor_control_system_v3/
├── backend/
│   ├── main.py                      # Uvicorn entry point
│   ├── api/
│   │   └── endpoints.py             # All FastAPI routes
│   ├── models/
│   │   └── motor_model.py           # DCMotor12V physics model + fault engine
│   ├── services/
│   │   └── motor_service.py         # Fleet manager, command queue, diagnostics
│   └── core/
│       └── cache_persistence.py     # In-memory cache + PostgreSQL (optional)
├── frontend/
│   ├── index.html                   # Industrial control dashboard
│   ├── css/styles.css
│   └── js/app.js
├── esp32_firmware/
│   └── motor_controller.ino         # Arduino sketch for ESP32
├── config/
│   └── .env.example                 # Environment variable template
├── requirements.txt
└── README.md
```

---

## Quick Start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure environment

```bash
cp config/.env.example .env
# Edit .env as needed (DATABASE_URL is optional)
```

### 3. Run the backend

```bash
python -m backend.main
```

API docs available at: `http://localhost:8000/docs`

---

## API Overview

### Single-Motor Control
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | /motor/create | Register a motor |
| POST | /motor/{id}/power_on | Power on |
| POST | /motor/{id}/start | Start motor |
| POST | /motor/{id}/stop | Graceful stop |
| POST | /motor/{id}/power_off | Power off (clears faults) |
| POST | /motor/{id}/emergency | Emergency stop |
| GET  | /motor/{id} | Full status snapshot |
| GET  | /motor/{id}/faults | Fault diagnosis |

### Fleet Control (new in v3.0)
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | /fleet/power_on | Power on ALL motors |
| POST | /fleet/start | Start ALL IDLE motors |
| POST | /fleet/stop | Gracefully stop ALL RUNNING motors |
| POST | /fleet/power_off | Power off ALL motors |
| POST | /fleet/emergency | Emergency stop entire fleet |
| GET  | /fleet/status | Status of all motors |
| GET  | /fleet/faults | Fleet-wide fault diagnosis |

### Runtime Configuration
| Method | Endpoint | Description |
|--------|----------|-------------|
| GET    | /motor/{id}/config | Current effective config |
| PATCH  | /motor/{id}/config | Apply threshold overrides |
| DELETE | /motor/{id}/config | Reset to engineering defaults |
| GET    | /config/bounds | Valid ranges per field |

### ESP32 Interface
| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | /update | Ingest sensor readings |
| GET  | /motor/{id}/command | Poll next command |

---

## Sensor Inputs

The model accepts exactly 4 raw sensor values per control cycle:

| Input | Unit | Normal Range |
|-------|------|--------------|
| voltage | V | 11.5 – 12.5 V |
| current | A | 0.3 – 4.0 A |
| speed | RPM | 0 – 3000 RPM |
| temperature | °C | 25 – 55 °C (shell) |

> **Important:** The temperature sensor reads the motor **outer casing (shell)**,
> not the winding. All thermal thresholds are in shell temperature.
> Winding temperature is estimated internally by the thermal observer.

---

## Fault Types

| Fault | Severity | Auto-Stop | Description |
|-------|----------|-----------|-------------|
| STALL | CRITICAL | Yes | Rotor locked — high current, zero speed |
| THERMAL_RUNAWAY | CRITICAL | Yes | Shell temp rising fast past critical |
| WINDING_SHORT | CRITICAL | Yes | Ke deviation indicates shorted coil |
| OVERHEAT | SEVERE | No | Shell temp in warning band |
| HIGH_CURRENT | SEVERE | No | Sustained overcurrent |
| VOLTAGE_DROP | SEVERE | No | Supply voltage below threshold |
| LOW_EFFICIENCY | WARNING | No | Efficiency below threshold at real load |
| BRUSH_WEAR | WARNING | No | High current ripple in steady-state |

---

## Motor Constants

Physical motor properties are in `MotorThresholds` and `MotorConstants`
inside `motor_model.py`. Engineers update these in code (requires redeploy).
Operators can tune fault thresholds at runtime via `PATCH /motor/{id}/config`.

| Constant | Default | Unit | How to measure |
|----------|---------|------|----------------|
| R | 2.4 | Ω | Multimeter, motor at rest |
| Km | 0.038 | N·m/A | Spin at known RPM, read open-circuit voltage |
| b | 0.000043 | N·m·s/rad | From no-load I and speed |
| Rth_ws | 2.5 | °C/W | Thermal step test |

---

## Database (optional)

Set `DATABASE_URL` in `.env` to enable PostgreSQL persistence:

```
DATABASE_URL=postgresql://user:password@host:5432/motor_db
```

Without a database URL the system runs fully in-memory with TTL caching.
All endpoints work identically — history queries return empty arrays.
