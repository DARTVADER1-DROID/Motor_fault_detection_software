/**
 * ============================================================
 *  MOTOR CONTROL SYSTEM v3.0 — ESP32 Firmware
 *  One ESP32 — One Motor — Relay Switched
 * ============================================================
 *
 *  HARDWARE:
 *    - ESP32 (any WiFi variant)
 *    - Relay module          (active HIGH — HIGH = motor ON)
 *    - Voltage sensor module (25V max, 5:1 resistor divider)
 *    - ACS712-20A            (100 mV/A, analog current sensor)
 *    - DHT11                 (shell temperature, digital)
 *    - LM393 slotted disk    (20 slots/rev, digital speed pulses)
 *
 *  REGISTRATION RULE:
 *    This firmware does NOT register the motor. Registration is
 *    done exclusively from the frontend dashboard.
 *    1. Flash this firmware with your MOTOR_ID
 *    2. Register that same ID from the frontend
 *    3. Reboot the ESP32
 *    If the motor ID is not found on the backend, the ESP32 halts
 *    and prints clear instructions on the serial monitor.
 *
 *  BOOT SEQUENCE:
 *    WiFi connect
 *    → GET  /health                  confirm backend is alive
 *    → POST /motor/{id}/power_on     confirm motor is registered
 *      → 404: HALT — print instructions, retry every 10s
 *      → 200: proceed to main loop
 *
 *  MAIN LOOP:
 *    Every SENSOR_INTERVAL_MS  → read all sensors → POST /update
 *    Every COMMAND_POLL_MS     → GET /motor/{id}/command → execute
 *    Every HEARTBEAT_MS        → GET /health → update watchdog timer
 *    Every loop()              → WiFi watchdog
 *                              → Backend watchdog (relay OFF if silent)
 *
 *  COMMANDS FROM BACKEND:
 *    START          — relay ON
 *    STOP           — relay OFF (graceful)
 *    POWER_ON       — backend sync acknowledge (relay unchanged)
 *    POWER_OFF      — relay OFF + POST /motor/{id}/power_off
 *    EMERGENCY_STOP — relay OFF immediately + POST /motor/{id}/emergency
 *
 * ============================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "DHT.h"


// ================================================================
//  SECTION 1 — USER CONFIGURATION
//  Only edit this section. Nothing else needs to change.
// ================================================================

// ── WiFi ─────────────────────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ── Motor Identity ───────────────────────────────────────────────
// Must exactly match the Motor ID registered from the frontend.
// Case-sensitive. Alphanumeric, hyphens and underscores only.
// Example: "MOTOR-01", "MOTOR_LINE_A", "PUMP_03"
const char* MOTOR_ID = "MOTOR-01";

// ── Backend URL ──────────────────────────────────────────────────
// Local PC  : "http://192.168.1.100:8000"    → set USE_HTTPS 0
// Railway   : "https://your-app.railway.app" → set USE_HTTPS 1
// No trailing slash.
const char* BACKEND_HOST = "https://motorfaultdetectionsoftware-production.up.railway.app";

// ── HTTPS ────────────────────────────────────────────────────────
// 0 = plain HTTP  (local PC / LAN)
// 1 = HTTPS       (Railway or any TLS-enabled server)
// When USE_HTTPS=1 certificate is not verified (setInsecure).
// Suitable for Railway public certs.
#define USE_HTTPS 1

// ── Polling & Timing (milliseconds) ─────────────────────────────
const unsigned long SENSOR_INTERVAL_MS   = 500;   // Send sensor data to backend
const unsigned long COMMAND_POLL_MS      = 500;   // Poll backend for commands
const unsigned long HEARTBEAT_MS         = 5000;  // Ping GET /health
const unsigned long WATCHDOG_MS          = 10000; // Relay OFF if backend silent
const unsigned long WIFI_RECONNECT_DELAY = 5000;  // Between WiFi retry attempts
const unsigned long DHT_INTERVAL         = 1000;  // DHT11 minimum read interval
const unsigned long RELAY_MIN_SWITCH_MS  = 500;   // Min time between relay toggles

// ── HTTP Request Timeout ─────────────────────────────────────────
const int HTTP_TIMEOUT_MS = 5000;

// ── ADC Averaging ────────────────────────────────────────────────
// Samples averaged per ADC reading to reduce ESP32 ADC noise.
const int ADC_SAMPLES = 10;


// ================================================================
//  SECTION 2 — PIN CONFIGURATION
//  Change pin numbers here to match your wiring.
// ================================================================

// ── Relay ────────────────────────────────────────────────────────
// Active HIGH: HIGH = relay energised = motor ON
//              LOW  = relay off       = motor OFF
//
// Valid safe output GPIOs (avoid ADC2 pins when WiFi is active,
// avoid strapping pins which affect boot mode):
//   Safe for relay: 25, 26, 27, 32, 33
//   Avoid (strapping): 0, 2, 5, 12, 15
//   Avoid (ADC2/WiFi conflict): 4, 13, 14, 25, 26, 27 when reading ADC
//   Note: 25, 26, 27 are safe OUTPUT-only relay pins.
const int PIN_RELAY = 27;

// ── Voltage Sensor — Analog (25V module, 5:1 divider) ────────────
// Uses ESP32 ADC1 only — ADC2 is disabled when WiFi is active.
// Valid ADC1 input-only pins: 34, 35, 36 (VP), 39 (VN)
// These pins have no internal pullup/pulldown — safe for analog.
// Recommended: 34, 35, 36, 39
const int PIN_VOLTAGE = 36;

// ── Current Sensor — ACS712-20A, Analog ─────────────────────────
// Same ADC1 restriction as voltage sensor.
// Do NOT use ADC2 pins (0,2,4,12,13,14,15,25,26,27) — WiFi disables them.
// Recommended: 35, 39  (36 used for voltage above)
const int PIN_CURRENT = 35;

// ── DHT11 Temperature Sensor — Digital ──────────────────────────
// Mounted on the motor SHELL (outer casing), not on the winding.
// Backend estimates winding temperature from shell reading.
// Valid digital I/O: 4, 13, 16, 17, 19, 21, 22, 23
// Avoid strapping pins: 0, 2, 5, 12, 15
// Recommended: 4, 13, 16, 17
#define DHTPIN  4
#define DHTTYPE DHT11

// ── LM393 Speed Sensor — Digital Interrupt ───────────────────────
// Slotted disk on motor shaft. LM393 comparator outputs clean
// digital pulses — one pulse per slot passing the sensor.
// All ESP32 GPIOs support external interrupts.
// Avoid strapping pins: 0, 2, 5, 12, 15
// Recommended: 13, 14, 16, 17, 32, 33
const int PIN_ENCODER = 13;


// ================================================================
//  SECTION 3 — SENSOR CALIBRATION
//  Adjust these constants if readings drift from actual values.
// ================================================================

// ── ADC Reference Voltage ────────────────────────────────────────
const float ADC_REF = 3.3;

// ── Voltage Sensor ───────────────────────────────────────────────
// 25V module uses 5:1 fixed resistor divider.
// VOLTAGE_CAL: fine-tune trim. Compare against a multimeter.
// If ESP32 reads 11.8V but multimeter shows 12.0V → VOLTAGE_CAL = 12.0/11.8
const float VOLTAGE_RATIO = 5.0;
float       VOLTAGE_CAL   = 1.0;

// ── ACS712-20A Current Sensor ────────────────────────────────────
// ACS712-20A sensitivity = 100 mV/A.
// Offset (midpoint at zero current) is auto-calibrated at boot.
// Do not change ACS_SENSITIVITY unless you swap the ACS712 variant:
//   ACS712-05B →  5A max → 0.185 V/A
//   ACS712-20A → 20A max → 0.100 V/A  ← this firmware
//   ACS712-30A → 30A max → 0.066 V/A
const float ACS_SENSITIVITY = 0.100;
float       ACS_OFFSET      = 0.0;   // Auto-set by calibrateACS() at boot

// ── LM393 Speed Sensor ───────────────────────────────────────────
// Slots on the disk mounted on the motor shaft.
const int ENCODER_PPR = 20;   // Pulses per revolution (20-slot disk)


// ================================================================
//  INTERNALS — do not edit below this line
// ================================================================

DHT dht(DHTPIN, DHTTYPE);

#if USE_HTTPS
WiFiClientSecure wifiClient;
#endif

// ── Live sensor values ───────────────────────────────────────────
float voltage     = 0.0;
float current_a   = 0.0;
float temperature = 25.0;
int   speed_rpm   = 0;

// ── Encoder (interrupt-driven) ───────────────────────────────────
volatile uint32_t encoderPulses    = 0;
uint32_t          lastPulseSnapshot = 0;
unsigned long     lastRpmCalc      = 0;

// ── Relay ────────────────────────────────────────────────────────
bool          relayOn           = false;
unsigned long lastRelaySwitch   = 0;

// ── Timing ───────────────────────────────────────────────────────
unsigned long lastSensorSend     = 0;
unsigned long lastCommandPoll    = 0;
unsigned long lastHeartbeatCheck = 0;
unsigned long lastWifiAttempt    = 0;
unsigned long lastDHTRead        = 0;
unsigned long lastBackendContact = 0;

// ── Boot state ───────────────────────────────────────────────────
bool backendReachable = false;
bool motorRegistered  = false;
bool watchdogFired    = false;


// ================================================================
//  ENCODER ISR
// ================================================================

void IRAM_ATTR encoderISR() {
  encoderPulses++;
}


// ================================================================
//  SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("============================================================");
  Serial.println("  MOTOR CONTROL SYSTEM v3.0 — ESP32 Firmware");
  Serial.println("  One ESP32 — One Motor");
  Serial.println("============================================================");
  Serial.printf ("[BOOT] Motor ID   : %s\n",   MOTOR_ID);
  Serial.printf ("[BOOT] Backend    : %s\n",   BACKEND_HOST);
  Serial.printf ("[BOOT] Protocol   : %s\n",   USE_HTTPS ? "HTTPS (TLS)" : "HTTP");
  Serial.printf ("[BOOT] Sensor Hz  : %.1f Hz (every %lu ms)\n",
                 1000.0 / SENSOR_INTERVAL_MS, SENSOR_INTERVAL_MS);
  Serial.printf ("[BOOT] Command Hz : %.1f Hz (every %lu ms)\n",
                 1000.0 / COMMAND_POLL_MS, COMMAND_POLL_MS);
  Serial.printf ("[BOOT] Watchdog   : %lu ms\n", WATCHDOG_MS);
  Serial.println("------------------------------------------------------------");

  // ── Relay: ensure motor is OFF before anything else ──────────
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  relayOn = false;
  Serial.printf("[RELAY] Initialised → OFF  (GPIO %d, active HIGH)\n", PIN_RELAY);

  // ── Encoder interrupt ────────────────────────────────────────
  pinMode(PIN_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), encoderISR, RISING);
  Serial.printf("[ENCODER] Interrupt attached → GPIO %d, PPR=%d\n",
                PIN_ENCODER, ENCODER_PPR);

  // ── DHT11 ────────────────────────────────────────────────────
  dht.begin();
  Serial.printf("[DHT11] Initialised → GPIO %d\n", DHTPIN);

  // ── ACS712 calibration ───────────────────────────────────────
  // Must run BEFORE WiFi.begin() — ADC2 shares resources with WiFi
  // and becomes unreliable after WiFi starts. ADC1 pins (34,35,36,39)
  // remain usable during WiFi, but calibration is most accurate
  // before WiFi radio activates.
  calibrateACS();

  // ── HTTPS client setup ───────────────────────────────────────
#if USE_HTTPS
  wifiClient.setInsecure();
  Serial.println("[TLS] WiFiClientSecure ready (certificate not verified)");
#endif

  // ── WiFi connect ─────────────────────────────────────────────
  connectWiFi();

  // ── Boot sequence: health → power_on ─────────────────────────
  if (WiFi.status() == WL_CONNECTED) {
    checkBackendHealth();
    if (backendReachable) {
      powerOnMotor();
    }
  }

  Serial.println("------------------------------------------------------------");
  if (backendReachable && motorRegistered) {
    lastBackendContact = millis();
    Serial.println("[BOOT] ✓ Ready — entering main loop");
  } else {
    Serial.println("[BOOT] ✗ HALTED — resolve errors above before continuing");
  }
  Serial.println("============================================================");
}


// ================================================================
//  MAIN LOOP
// ================================================================

void loop() {

  // ── Halt state: retry until resolved ────────────────────────
  if (!backendReachable || !motorRegistered) {
    static unsigned long lastHaltMsg = 0;
    if (millis() - lastHaltMsg >= 10000) {
      Serial.println();
      Serial.println("[HALT] ============================================================");
      if (!backendReachable) {
        Serial.printf("[HALT] Backend unreachable at: %s\n", BACKEND_HOST);
        Serial.println("[HALT] → Check BACKEND_HOST, WiFi, and that backend is running");
      }
      if (!motorRegistered) {
        Serial.printf("[HALT] Motor '%s' is NOT registered on the backend\n", MOTOR_ID);
        Serial.println("[HALT] → Open the frontend dashboard");
        Serial.printf("[HALT] → Register a motor with ID: %s\n", MOTOR_ID);
        Serial.println("[HALT] → Then reboot the ESP32");
      }
      Serial.println("[HALT] Retrying now...");
      Serial.println("[HALT] ============================================================");
      lastHaltMsg = millis();

      checkWiFiConnection();
      if (WiFi.status() == WL_CONNECTED) {
        if (!backendReachable) checkBackendHealth();
        if (backendReachable && !motorRegistered) powerOnMotor();
      }
    }
    return;
  }

  unsigned long now = millis();

  // ── WiFi watchdog ────────────────────────────────────────────
  checkWiFiConnection();

  // ── Backend watchdog ─────────────────────────────────────────
  if (now - lastBackendContact > WATCHDOG_MS) {
    if (!watchdogFired) {
      watchdogFired = true;
      Serial.println();
      Serial.println("[WATCHDOG] *** Backend silent — cutting relay for safety ***");
      Serial.printf ("[WATCHDOG] No contact for %lu ms (threshold %lu ms)\n",
                     now - lastBackendContact, WATCHDOG_MS);
      Serial.printf ("[WATCHDOG] Free heap: %lu bytes\n", ESP.getFreeHeap());
      setRelay(false, "WATCHDOG-TIMEOUT");
    }
  } else {
    if (watchdogFired) {
      watchdogFired = false;
      Serial.println("[WATCHDOG] Backend contact restored — resuming polling");
    }
  }

  // ── Skip network calls if WiFi down ─────────────────────────
  if (WiFi.status() != WL_CONNECTED) return;

  // ── Sensor read + send ───────────────────────────────────────
  if (now - lastSensorSend >= SENSOR_INTERVAL_MS) {
    readSensors();
    sendSensorData();
    lastSensorSend = now;
  }

  // ── Command poll ─────────────────────────────────────────────
  if (now - lastCommandPoll >= COMMAND_POLL_MS) {
    pollCommand();
    lastCommandPoll = now;
  }

  // ── Heartbeat ────────────────────────────────────────────────
  if (now - lastHeartbeatCheck >= HEARTBEAT_MS) {
    checkBackendHealth();
    lastHeartbeatCheck = now;
  }
}


// ================================================================
//  WIFI
// ================================================================

void connectWiFi() {
  Serial.println();
  Serial.printf("[WIFI] Connecting to: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WIFI] ✓ Connected");
    Serial.printf ("[WIFI]   IP Address  : %s\n",   WiFi.localIP().toString().c_str());
    Serial.printf ("[WIFI]   Gateway     : %s\n",   WiFi.gatewayIP().toString().c_str());
    Serial.printf ("[WIFI]   Subnet Mask : %s\n",   WiFi.subnetMask().toString().c_str());
    Serial.printf ("[WIFI]   DNS Server  : %s\n",   WiFi.dnsIP().toString().c_str());
    Serial.printf ("[WIFI]   RSSI        : %d dBm\n", WiFi.RSSI());
    Serial.printf ("[WIFI]   MAC Address : %s\n",   WiFi.macAddress().c_str());
    Serial.printf ("[WIFI]   Channel     : %d\n",   WiFi.channel());
  } else {
    Serial.println("[WIFI] ✗ Connection FAILED");
    Serial.printf ("[WIFI]   Status code : %d\n",   (int)WiFi.status());
    Serial.println("[WIFI]   Will retry automatically");
  }

  lastWifiAttempt = millis();
}

void checkWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiAttempt < WIFI_RECONNECT_DELAY) return;

  Serial.println("[WIFI] ✗ Disconnected");
  Serial.printf ("[WIFI]   Reason code : %d\n", (int)WiFi.disconnectReasonCode());
  Serial.println("[WIFI]   Attempting reconnect...");
  connectWiFi();
}


// ================================================================
//  SENSOR READING
// ================================================================

void readSensors() {

  // ── Voltage (10-sample average) ──────────────────────────────
  long vSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    vSum += analogRead(PIN_VOLTAGE);
    delayMicroseconds(200);
  }
  float vADC = ((float)vSum / ADC_SAMPLES / 4095.0) * ADC_REF;
  voltage    = vADC * VOLTAGE_RATIO * VOLTAGE_CAL;

  // ── Current (10-sample average) ──────────────────────────────
  long cSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    cSum += analogRead(PIN_CURRENT);
    delayMicroseconds(200);
  }
  float cADC  = ((float)cSum / ADC_SAMPLES / 4095.0) * ADC_REF;
  current_a   = (cADC - ACS_OFFSET) / ACS_SENSITIVITY;
  if (current_a < 0.0) current_a = 0.0;   // Clamp noise below zero

  // ── Temperature — DHT11 (max 1 read/sec) ─────────────────────
  if (millis() - lastDHTRead >= DHT_INTERVAL) {
    float t = dht.readTemperature();
    if (!isnan(t)) {
      temperature = t;
    } else {
      Serial.println("[DHT11] ✗ Read failed — retaining last value");
    }
    lastDHTRead = millis();
  }

  // ── Speed — RPM from LM393 pulse count ───────────────────────
  unsigned long now     = millis();
  unsigned long elapsed = now - lastRpmCalc;
  if (elapsed >= SENSOR_INTERVAL_MS) {
    uint32_t currentPulses = encoderPulses;               // Atomic read
    uint32_t delta         = currentPulses - lastPulseSnapshot;
    lastPulseSnapshot      = currentPulses;
    lastRpmCalc            = now;
    // RPM = (pulses / PPR) * (60000 / elapsed_ms)
    speed_rpm = (int)((delta * 60000UL) / ((unsigned long)elapsed * ENCODER_PPR));
  }

  // ── Clamp to backend-accepted ranges ─────────────────────────
  voltage     = constrain(voltage,     0.0,   20.0);
  current_a   = constrain(current_a,   0.0,   15.0);
  temperature = constrain(temperature, -40.0, 120.0);
  speed_rpm   = constrain(speed_rpm,   0,     3500);

  // ── Serial diagnostic ─────────────────────────────────────────
  Serial.printf("[SENSOR] V: %.3fV  I: %.3fA  T: %.1f°C  RPM: %d  Relay: %s\n",
                voltage, current_a, temperature, speed_rpm,
                relayOn ? "ON" : "OFF");
}


// ================================================================
//  ACS712 AUTO CALIBRATION
// ================================================================

void calibrateACS() {
  Serial.println("[ACS712] Calibrating — motor must be OFF with NO LOAD...");

  float sum    = 0;
  int   count  = 200;

  for (int i = 0; i < count; i++) {
    int   raw = analogRead(PIN_CURRENT);
    float v   = (raw / 4095.0) * ADC_REF;
    sum += v;
    delay(5);
  }

  ACS_OFFSET = sum / count;
  Serial.printf("[ACS712] ✓ Calibration done — zero-current offset: %.4fV\n", ACS_OFFSET);
}


// ================================================================
//  HTTP HELPERS — single GET/POST handles HTTP and HTTPS
// ================================================================

int doGET(const String& path, String& responseBody) {
  String url  = String(BACKEND_HOST) + path;
  int    code = -1;

#if USE_HTTPS
  HTTPClient http;
  http.begin(wifiClient, url);
#else
  HTTPClient http;
  http.begin(url);
#endif

  http.setTimeout(HTTP_TIMEOUT_MS);
  code = http.GET();

  if (code > 0) {
    responseBody = http.getString();
  } else {
    responseBody = "";
    Serial.printf("[HTTP] GET  %s → %s\n",
                  path.c_str(), http.errorToString(code).c_str());
  }
  http.end();
  return code;
}

int doPOST(const String& path, const String& body, String& responseBody) {
  String url  = String(BACKEND_HOST) + path;
  int    code = -1;

#if USE_HTTPS
  HTTPClient http;
  http.begin(wifiClient, url);
#else
  HTTPClient http;
  http.begin(url);
#endif

  http.setTimeout(HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");
  code = http.POST(body);

  if (code > 0) {
    responseBody = http.getString();
  } else {
    responseBody = "";
    Serial.printf("[HTTP] POST %s → %s\n",
                  path.c_str(), http.errorToString(code).c_str());
  }
  http.end();
  return code;
}


// ================================================================
//  BOOT CALLS
// ================================================================

void checkBackendHealth() {
  Serial.println("[HEALTH] Pinging backend...");

  String response;
  int code = doGET("/health", response);

  if (code == 200) {
    backendReachable   = true;
    lastBackendContact = millis();

    // Parse and print backend details
    Serial.println("[HEALTH] ✓ Backend healthy");
    Serial.printf ("[HEALTH]   URL      : %s/health\n", BACKEND_HOST);
    Serial.printf ("[HEALTH]   Response : %s\n", response.c_str());
    Serial.printf ("[HEALTH]   Uptime   : %lu s\n", millis() / 1000);
    Serial.printf ("[HEALTH]   Free heap: %lu bytes\n", ESP.getFreeHeap());
  } else {
    backendReachable = false;
    Serial.printf("[HEALTH] ✗ Backend unreachable — HTTP %d\n", code);
    Serial.printf("[HEALTH]   URL: %s/health\n", BACKEND_HOST);
    Serial.println("[HEALTH]   → Check BACKEND_HOST and that the server is running");
  }
}

void powerOnMotor() {
  String path     = String("/motor/") + MOTOR_ID + "/power_on";
  String response;

  Serial.printf("[BOOT] Sending power_on → motor '%s'...\n", MOTOR_ID);

  int code = doPOST(path, "{}", response);

  if (code == 200) {
    motorRegistered    = true;
    lastBackendContact = millis();
    Serial.printf("[BOOT] ✓ Motor '%s' registered and powered on\n", MOTOR_ID);
  } else if (code == 404) {
    motorRegistered = false;
    Serial.println("[BOOT] ============================================================");
    Serial.printf ("[BOOT] ✗ Motor '%s' is NOT registered on the backend\n", MOTOR_ID);
    Serial.println("[BOOT]");
    Serial.println("[BOOT]   ACTION REQUIRED:");
    Serial.println("[BOOT]   1. Open the frontend dashboard in your browser");
    Serial.printf ("[BOOT]   2. Create a motor with ID: %s\n",  MOTOR_ID);
    Serial.println("[BOOT]   3. Reboot this ESP32 after registration");
    Serial.println("[BOOT]");
    Serial.println("[BOOT]   The ESP32 will retry automatically every 10 seconds.");
    Serial.println("[BOOT] ============================================================");
  } else {
    motorRegistered = false;
    Serial.printf("[BOOT] ✗ power_on request failed — HTTP %d\n", code);
    Serial.printf("[BOOT]   Response: %s\n", response.c_str());
  }
}


// ================================================================
//  SENSOR DATA → BACKEND
// ================================================================

void sendSensorData() {
  // Matches MotorUpdate schema: motor_id, voltage, current, speed, temperature
  char body[220];
  snprintf(body, sizeof(body),
    "{\"motor_id\":\"%s\","
     "\"voltage\":%.3f,"
     "\"current\":%.3f,"
     "\"speed\":%d,"
     "\"temperature\":%.2f}",
    MOTOR_ID, voltage, current_a, speed_rpm, temperature
  );

  String response;
  int code = doPOST("/update", String(body), response);

  if (code == 200) {
    lastBackendContact = millis();
    Serial.println("[UPDATE] ✓ Sensor data accepted by backend");
  } else {
    Serial.printf("[UPDATE] ✗ Failed — HTTP %d\n", code);
    Serial.printf("[UPDATE]   Body     : %s\n", body);
    Serial.printf("[UPDATE]   Response : %s\n", response.c_str());
  }
}


// ================================================================
//  COMMAND POLLING
// ================================================================

void pollCommand() {
  String path = String("/motor/") + MOTOR_ID + "/command";
  String response;

  int code = doGET(path, response);

  if (code == 200) {
    lastBackendContact = millis();

    String cmd = extractJsonString(response, "command");

    if (cmd.length() == 0 || cmd == "null") {
      // No pending command — normal idle, suppress serial noise
      return;
    }

    Serial.printf("[CMD] ← Backend sent: %s\n", cmd.c_str());
    executeCommand(cmd);

  } else if (code == 404) {
    Serial.printf("[CMD] ✗ Motor '%s' not found — HTTP 404\n", MOTOR_ID);
  } else {
    Serial.printf("[CMD] ✗ Poll failed — HTTP %d\n", code);
    Serial.printf("[CMD]   URL: %s%s\n", BACKEND_HOST, path.c_str());
  }
}


// ================================================================
//  COMMAND EXECUTION
// ================================================================

void executeCommand(const String& cmd) {

  if (cmd == "START") {
    Serial.println("[CMD] Executing START");
    setRelay(true, "START");

  } else if (cmd == "STOP") {
    Serial.println("[CMD] Executing STOP");
    setRelay(false, "STOP");

  } else if (cmd == "POWER_ON") {
    // Backend is syncing its model state — relay is already on.
    // Acknowledge without changing relay.
    Serial.println("[CMD] Executing POWER_ON → backend sync, relay unchanged");

  } else if (cmd == "POWER_OFF") {
    Serial.println("[CMD] Executing POWER_OFF");
    setRelay(false, "POWER_OFF");

    String response;
    int code = doPOST(String("/motor/") + MOTOR_ID + "/power_off", "{}", response);
    if (code == 200) {
      Serial.println("[CMD] ✓ Backend acknowledged power_off");
    } else {
      Serial.printf("[CMD] ✗ power_off notify failed — HTTP %d\n", code);
    }

  } else if (cmd == "EMERGENCY_STOP") {
    // Hard cut — bypass relay guard, immediate
    Serial.println("[CMD] *** EMERGENCY_STOP *** — cutting relay immediately");
    digitalWrite(PIN_RELAY, LOW);
    relayOn         = false;
    lastRelaySwitch = millis();
    Serial.println("[RELAY] *** EMERGENCY — OFF (guard bypassed) ***");

    String response;
    int code = doPOST(String("/motor/") + MOTOR_ID + "/emergency", "{}", response);
    if (code == 200) {
      Serial.println("[CMD] ✓ Backend acknowledged emergency stop");
    } else {
      Serial.printf("[CMD] ✗ emergency notify failed — HTTP %d\n", code);
    }

  } else {
    Serial.printf("[CMD] ✗ Unknown command: '%s' — ignored\n", cmd.c_str());
  }
}


// ================================================================
//  RELAY CONTROL
// ================================================================

void setRelay(bool on, const char* reason) {
  // Enforce minimum switching delay to protect relay and motor
  unsigned long timeSinceSwitch = millis() - lastRelaySwitch;
  if (timeSinceSwitch < RELAY_MIN_SWITCH_MS) {
    unsigned long wait = RELAY_MIN_SWITCH_MS - timeSinceSwitch;
    Serial.printf("[RELAY] Switch guard active — waiting %lu ms (reason: %s)\n",
                  wait, reason);
    delay(wait);
  }

  if (on == relayOn) {
    Serial.printf("[RELAY] Already %s — no change (reason: %s)\n",
                  on ? "ON" : "OFF", reason);
    return;
  }

  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  relayOn         = on;
  lastRelaySwitch = millis();

  Serial.printf("[RELAY] → %s  (reason: %s)  uptime: %lu s  heap: %lu bytes\n",
                on ? "ON" : "OFF", reason,
                millis() / 1000, ESP.getFreeHeap());
}


// ================================================================
//  JSON STRING EXTRACTOR
//  Lightweight parser — extracts one string or null field from
//  a flat JSON response. No library dependency needed.
// ================================================================

String extractJsonString(const String& json, const String& key) {
  String searchKey = "\"" + key + "\"";
  int    keyIdx    = json.indexOf(searchKey);
  if (keyIdx == -1) return "";

  int colonIdx = json.indexOf(':', keyIdx + searchKey.length());
  if (colonIdx == -1) return "";

  int valueStart = colonIdx + 1;
  while (valueStart < (int)json.length() && json[valueStart] == ' ') valueStart++;

  // null value
  if (json.substring(valueStart, valueStart + 4) == "null") return "null";

  // quoted string value
  if (json[valueStart] == '"') {
    int valueEnd = json.indexOf('"', valueStart + 1);
    if (valueEnd == -1) return "";
    return json.substring(valueStart + 1, valueEnd);
  }

  return "";
}
