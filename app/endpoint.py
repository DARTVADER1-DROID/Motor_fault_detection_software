from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from services.motor_service import MotorFleetService as MotorService
from models.motor_model import DCMotor12V

app = FastAPI()

# =========================
# SINGLETON SERVICE
# =========================
motor_service = MotorService()


# =========================
# REQUEST SCHEMAS
# =========================
class MotorCreate(BaseModel):
    motor_id: str


class MotorUpdate(BaseModel):
    motor_id: str
    voltage: float
    current: float
    speed: int
    temperature: float


# =========================
# MOTOR MANAGEMENT
# =========================
@app.post("/motor/create")
def create_motor(data: MotorCreate):
    if motor_service.exists(data.motor_id):
        raise HTTPException(400, "Motor already exists")

    motor = DCMotor12V(name=data.motor_id)
    motor_service.register_motor(data.motor_id, motor)

    return {"message": "Motor created"}


@app.delete("/motor/{motor_id}")
def delete_motor(motor_id: str):
    motor_service.remove_motor(motor_id)
    return {"message": "Motor removed"}


# =========================
# CONTROL (COMMAND WRITE)
# =========================
@app.post("/motor/{motor_id}/power_on")
def power_on(motor_id: str):
    motor_service.queue_command(motor_id, "POWER_ON")
    return {"message": "POWER_ON queued"}


@app.post("/motor/{motor_id}/start")
def start_motor(motor_id: str):
    motor_service.queue_command(motor_id, "START")
    return {"message": "START queued"}


@app.post("/motor/{motor_id}/stop")
def stop_motor(motor_id: str):
    motor_service.queue_command(motor_id, "STOP")
    return {"message": "STOP queued"}


@app.post("/motor/{motor_id}/power_off")
def power_off(motor_id: str):
    motor_service.queue_command(motor_id, "POWER_OFF")
    return {"message": "POWER_OFF queued"}


@app.post("/motor/{motor_id}/emergency")
def emergency(motor_id: str):
    motor_service.queue_command(motor_id, "EMERGENCY_STOP")
    return {"message": "EMERGENCY_STOP queued"}


# =========================
# ESP32 POLLING ENDPOINT
# =========================
@app.get("/motor/{motor_id}/command")
def get_command(motor_id: str):
    try:
        cmd = motor_service.get_next_command(motor_id)
        return {"command": cmd}
    except ValueError as e:
        raise HTTPException(404, str(e))


# =========================
# SENSOR UPDATE (ESP32 → BACKEND)
# =========================
@app.post("/update")
def update_motor(data: MotorUpdate):
    try:
        motor_service.update_motor(
            motor_id=data.motor_id,
            voltage=data.voltage,
            current=data.current,
            speed=data.speed,
            temperature=data.temperature,
        )
        return {"status": "updated"}

    except ValueError as e:
        raise HTTPException(404, str(e))


# =========================
# STATUS
# =========================
@app.get("/motor/{motor_id}")
def get_status(motor_id: str):
    return motor_service.get_motor_status(motor_id)


@app.get("/fleet/status")
def fleet_status():
    return motor_service.get_all_status()


@app.get("/fleet/faults")
def fleet_faults():
    return motor_service.get_fault_summary()