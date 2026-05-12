/*
 * ============================================================
 *  CleanBot Firmware  v2.0
 *  Board   : ESP32 DevKit V1 (38-pin)
 *  Core    : ESP32 Arduino Core 3.x (3.3.8 tested)
 *  Driver  : L298N (drive wheels)
 *  Relay   : 1-channel active-low — controls ELECTROMAGNET
 *  BLDC    : PWM via ESC on GPIO 22
 *  Servo   : Sweep arm 0 to 90 degrees on GPIO 21
 * ============================================================
 *
 *  WIRING
 *  ──────────────────────────────────────────────────────────
 *  L298N IN1   → GPIO 26   Left motor direction 1
 *  L298N IN2   → GPIO 27   Left motor direction 2
 *  L298N ENA   → GPIO 14   Left motor PWM speed
 *  L298N IN3   → GPIO 25   Right motor direction 1
 *  L298N IN4   → GPIO 33   Right motor direction 2
 *  L298N ENB   → GPIO 32   Right motor PWM speed
 *
 *  Relay IN    → GPIO 18   LOW=ON  HIGH=OFF (active-low)
 *  Relay COM   → Battery +
 *  Relay NO    → Electromagnet +
 *  Electromagnet - → GND
 *
 *  ESC signal  → GPIO 22   BLDC PWM (1000–2000us)
 *  ESC power   → Battery directly
 *  ESC GND     → Common GND
 *
 *  Servo SIG   → GPIO 21
 *  Servo VCC   → 5V
 *  Servo GND   → GND
 *
 *  All GND rails tied together
 * ============================================================
 *
 *  COLLECTION CYCLE (30 seconds)
 *  ──────────────────────────────────────────────────────────
 *  T+0s   Robot stops
 *  T+0s   Relay ON  → Electromagnet activates
 *  T+1s   BLDC starts low (~20% throttle)
 *  T+4s   BLDC ramps to full over 5 seconds
 *  T+9s   Servo sweeps 0 → 90 degrees
 *  T+25s  BLDC ramps down over 3 seconds
 *  T+28s  Servo returns to 0 degrees
 *  T+29s  Relay OFF → Electromagnet releases waste into bin
 *  T+30s  Robot resumes to next waypoint
 * ============================================================
 *
 *  CORE 3.x PWM API (different from 2.x)
 *  ──────────────────────────────────────────────────────────
 *  ledcAttach(pin, freq, resolution)   ← no channel numbers
 *  ledcWrite(pin, value)               ← write to PIN not channel
 * ============================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// ── EDIT THESE ────────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* SERVER_URL    = "https://YOUR-APP.onrender.com";
// ─────────────────────────────────────────────────────────


// ════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════════════════════════
#define MOTOR_A_IN1   26
#define MOTOR_A_IN2   27
#define MOTOR_A_ENA   14
#define MOTOR_B_IN3   25
#define MOTOR_B_IN4   33
#define MOTOR_B_ENB   32

#define RELAY_PIN     18
#define BLDC_PIN      22
#define SERVO_PIN     21


// ════════════════════════════════════════════════════════════
//  PWM SETTINGS  (Core 3.x — no channel numbers)
// ════════════════════════════════════════════════════════════
#define PWM_FREQ_MOTOR    1000
#define PWM_RES_MOTOR     8

// BLDC ESC — 50Hz 16-bit
// 1000us = 3276 counts  (stop/arm)
// 1190us = 3900 counts  (~20% throttle)
// 2000us = 6553 counts  (100% throttle)
#define PWM_FREQ_BLDC     50
#define PWM_RES_BLDC      16
#define BLDC_STOP         3276
#define BLDC_LOW          3900
#define BLDC_HIGH         6553
#define BLDC_RAMP_STEPS   50


// ════════════════════════════════════════════════════════════
//  TIMING / TUNING
// ════════════════════════════════════════════════════════════
#define DRIVE_SPEED         200
#define BLDC_LOW_HOLD_MS    3000
#define BLDC_RAMP_UP_MS     5000
#define BLDC_RAMP_DOWN_MS   3000
#define SERVO_MOVE_MS       1000
#define COLLECTION_MS       30000     // 30 second collection cycle
#define SERVO_HOME          0
#define SERVO_SWEEP         90        // Sweep to 90 degrees

// Navigation calibration — measure these on your actual robot
#define MM_PER_UNIT         1500.0f   // physical size of your arena in mm
#define MM_PER_SEC          250.0f    // mm/sec at DRIVE_SPEED=200
#define DEG_PER_SEC         85.0f     // degrees/sec turning at DRIVE_SPEED=200

#define POLL_RUNNING_MS     500
#define POLL_IDLE_MS        2000


// ════════════════════════════════════════════════════════════
//  GLOBALS
// ════════════════════════════════════════════════════════════
Servo sweepServo;
float currentX        = 0.0f;
float currentY        = 0.0f;
float currentHeading  = 0.0f;
bool  isRunning       = false;
unsigned long lastPollTime = 0;


// ════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ════════════════════════════════════════════════════════════
void setupDriveMotors();
void setupRelay();
void setupBLDC();
void setupServo();
void connectWiFi();
void motorsStop();
void setMotorA(int dir, int spd);
void setMotorB(int dir, int spd);
void driveForward(int ms);
void driveBackward(int ms);
void turnRight(int ms);
void turnLeft(int ms);
String fetchServerStatus();
void fetchAndExecuteCommand();
void reportPosition();
void reportComponentState();
void sendStatusMessage(const char* msg);
void runCollectionCycle();
void rampBLDC(int startVal, int endVal, int durationMs);
void sweepServoSmooth(int fromAngle, int toAngle, int durationMs);
void navigateTo(float tx, float ty);
void handleManualCommand(const char* type, int speed, float distance, float degrees);


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n============================================");
  Serial.println("  CleanBot Firmware v2.0");
  Serial.println("  ESP32 Core 3.x");
  Serial.println("============================================\n");

  setupDriveMotors();
  setupRelay();
  setupBLDC();
  setupServo();
  connectWiFi();

  Serial.println("\n[READY] Awaiting commands from server...\n");
}


// ════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Lost connection — reconnecting...");
    connectWiFi();
    delay(2000);
    return;
  }

  unsigned long now      = millis();
  unsigned long interval = isRunning ? POLL_RUNNING_MS : POLL_IDLE_MS;

  if (now - lastPollTime >= interval) {
    lastPollTime = now;
    String status = fetchServerStatus();

    if (status == "running" || status == "collecting") {
      isRunning = true;
      fetchAndExecuteCommand();
    } else {
      if (isRunning) {
        motorsStop();
        isRunning = false;
        Serial.printf("[STATUS] Server: %s\n", status.c_str());
      }
    }
    reportPosition();
  }
}


// ════════════════════════════════════════════════════════════
//  HARDWARE SETUP
// ════════════════════════════════════════════════════════════
void setupDriveMotors() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  ledcAttach(MOTOR_A_ENA, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  ledcAttach(MOTOR_B_ENB, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  motorsStop();
  Serial.println("[MOTORS] L298N ready");
}

void setupRelay() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);   // HIGH = OFF (active-low)
  Serial.println("[RELAY]  Electromagnet relay ready — OFF");
}

void setupBLDC() {
  ledcAttach(BLDC_PIN, PWM_FREQ_BLDC, PWM_RES_BLDC);
  Serial.println("[BLDC]   Arming ESC...");
  ledcWrite(BLDC_PIN, BLDC_STOP);
  delay(2000);
  Serial.println("[BLDC]   ESC armed");
}

void setupServo() {
  sweepServo.attach(SERVO_PIN, 500, 2400);
  sweepServo.write(SERVO_HOME);
  delay(600);
  Serial.println("[SERVO]  Arm at 0 deg");
}

void connectWiFi() {
  Serial.printf("[WIFI]   Connecting to '%s'", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI]   Connected! IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI]   FAILED — restarting...");
    delay(3000);
    ESP.restart();
  }
}


// ════════════════════════════════════════════════════════════
//  SERVER COMMUNICATION
// ════════════════════════════════════════════════════════════
String fetchServerStatus() {
  HTTPClient http;
  http.begin(String(SERVER_URL) + "/api/robot/status");
  http.setTimeout(5000);
  int code = http.GET();
  if (code != 200) { http.end(); return "error"; }
  String body = http.getString();
  http.end();
  JsonDocument doc;
  if (deserializeJson(doc, body)) return "error";
  return String((const char*)doc["robot_status"]);
}

void fetchAndExecuteCommand() {
  HTTPClient http;
  http.begin(String(SERVER_URL) + "/api/robot/next_command");
  http.setTimeout(5000);
  int code = http.GET();
  if (code != 200) { http.end(); return; }
  String body = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, body)) return;

  if (doc["command"].isNull()) {
    String s = doc["status"].as<String>();
    if (s == "completed") {
      Serial.println("[NAV] All commands done!");
      motorsStop();
      isRunning = false;
    }
    return;
  }

  const char* type  = doc["command"]["type"];
  int speed         = doc["command"]["speed"] | DRIVE_SPEED;
  float distance    = doc["command"]["distance"] | 0.0f;
  float degrees     = doc["command"]["degrees"] | 0.0f;
  int remaining     = doc["remaining"] | 0;

  Serial.printf("[CMD] %s  (queue: %d)\n", type, remaining);

  // ── Dispatch to the right handler ──────────────────────
  if      (strcmp(type, "forward")      == 0) handleManualCommand(type, speed, distance, 0);
  else if (strcmp(type, "backward")     == 0) handleManualCommand(type, speed, distance, 0);
  else if (strcmp(type, "turn_right")   == 0) handleManualCommand(type, speed, 0, degrees);
  else if (strcmp(type, "turn_left")    == 0) handleManualCommand(type, speed, 0, degrees);
  else if (strcmp(type, "stop")         == 0) { motorsStop(); }
  else if (strcmp(type, "relay")        == 0) {
    const char* s = doc["command"]["state"];
    bool on = s && strcmp(s, "on") == 0;
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
    Serial.printf("[RELAY] %s\n", on ? "ON" : "OFF");
    reportComponentState();
  }
  else if (strcmp(type, "bldc_ramp")    == 0) {
    int from = doc["command"]["from"] | BLDC_STOP;
    int to   = doc["command"]["to"]   | BLDC_STOP;
    int dur  = doc["command"]["duration_ms"] | 3000;
    rampBLDC(from, to, dur);
  }
  else if (strcmp(type, "servo_sweep")  == 0) {
    int from = doc["command"]["from"] | 0;
    int to   = doc["command"]["to"]   | 90;
    sweepServoSmooth(from, to, SERVO_MOVE_MS);
    reportComponentState();
  }
  else if (strcmp(type, "collect")      == 0) runCollectionCycle();
  else if (strcmp(type, "move")         == 0) {
    float tx = doc["command"]["x"] | currentX;
    float ty = doc["command"]["y"] | currentY;
    navigateTo(tx, ty);
  }
  else if (strcmp(type, "hold")         == 0) {
    int dur = doc["command"]["duration_ms"] | 1000;
    Serial.printf("[HOLD] %d ms\n", dur);
    delay(dur);
  }
}

// Handle a manual drive command from the dashboard
// distance is in mm (0 = use a short default)
void handleManualCommand(const char* type, int spd, float distance_mm, float deg) {
  if (strcmp(type, "forward") == 0) {
    int ms = distance_mm > 0 ? (int)(distance_mm / MM_PER_SEC * 1000) : 200;
    setMotorA(1,  spd);
    setMotorB(1,  spd);
    if (distance_mm > 0) { delay(ms); motorsStop(); }
    // If distance=0 (manual hold), ESP32 keeps driving until next 'stop' command
  }
  else if (strcmp(type, "backward") == 0) {
    int ms = distance_mm > 0 ? (int)(distance_mm / MM_PER_SEC * 1000) : 200;
    setMotorA(-1, spd);
    setMotorB(-1, spd);
    if (distance_mm > 0) { delay(ms); motorsStop(); }
  }
  else if (strcmp(type, "turn_right") == 0) {
    int ms = deg > 0 ? (int)(deg / DEG_PER_SEC * 1000) : 300;
    setMotorA(1,  spd);
    setMotorB(-1, spd);
    delay(ms);
    motorsStop();
  }
  else if (strcmp(type, "turn_left") == 0) {
    int ms = deg > 0 ? (int)(deg / DEG_PER_SEC * 1000) : 300;
    setMotorA(-1, spd);
    setMotorB(1,  spd);
    delay(ms);
    motorsStop();
  }
}

void reportPosition() {
  HTTPClient http;
  http.begin(String(SERVER_URL) + "/api/robot/position");
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);
  JsonDocument doc;
  doc["x"] = currentX;
  doc["y"] = currentY;
  doc["heading"] = currentHeading;
  String body;
  serializeJson(doc, body);
  http.POST(body);
  http.end();
}

void reportComponentState() {
  HTTPClient http;
  http.begin(String(SERVER_URL) + "/api/robot/component_state");
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);
  JsonDocument doc;
  doc["relay"] = (digitalRead(RELAY_PIN) == LOW);   // LOW = relay ON
  doc["servo"] = sweepServo.read();
  String body;
  serializeJson(doc, body);
  http.POST(body);
  http.end();
}

void sendStatusMessage(const char* msg) {
  HTTPClient http;
  http.begin(String(SERVER_URL) + "/api/robot/message");
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);
  JsonDocument doc;
  doc["message"] = msg;
  String body;
  serializeJson(doc, body);
  http.POST(body);
  http.end();
  Serial.printf("[MSG] %s\n", msg);
}


// ════════════════════════════════════════════════════════════
//  COLLECTION CYCLE  (30 seconds)
// ════════════════════════════════════════════════════════════
void runCollectionCycle() {
  unsigned long cycleStart = millis();

  Serial.println("\n+--------------------------------------------+");
  Serial.println("|      COLLECTION CYCLE  (30 seconds)        |");
  Serial.println("+--------------------------------------------+");
  sendStatusMessage("Collecting waste — cycle started");

  // 1. Stop robot
  motorsStop();
  Serial.println("[CYCLE] 1. Stopped");
  delay(400);

  // 2. Relay ON — electromagnet activates
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("[CYCLE] 2. Electromagnet ON");
  reportComponentState();
  delay(600);

  // 3. BLDC starts low
  ledcWrite(BLDC_PIN, BLDC_LOW);
  Serial.println("[CYCLE] 3. BLDC low speed");
  delay(BLDC_LOW_HOLD_MS);

  // 4. BLDC ramps to full speed over 5 seconds
  Serial.println("[CYCLE] 4. Ramping BLDC to full...");
  rampBLDC(BLDC_LOW, BLDC_HIGH, BLDC_RAMP_UP_MS);

  // 5. Servo sweeps 0 → 90 degrees
  Serial.println("[CYCLE] 5. Servo sweep 0 → 90°");
  sweepServoSmooth(SERVO_HOME, SERVO_SWEEP, SERVO_MOVE_MS);
  reportComponentState();

  // 6. Hold for remainder of the 30-second window
  unsigned long elapsed  = millis() - cycleStart;
  unsigned long holdTime = (COLLECTION_MS > elapsed) ? (COLLECTION_MS - elapsed) - 6000 : 0;
  Serial.printf("[CYCLE] 6. Holding %.0f sec...\n", holdTime / 1000.0f);
  unsigned long holdStart = millis();
  while (millis() - holdStart < holdTime) {
    long secsLeft = ((long)holdTime - (long)(millis() - holdStart)) / 1000;
    Serial.printf("[CYCLE]    %ld sec remaining\r", secsLeft);
    delay(1000);
  }
  Serial.println();

  // 7. BLDC ramps down
  Serial.println("[CYCLE] 7. Ramping BLDC down...");
  rampBLDC(BLDC_HIGH, BLDC_STOP, BLDC_RAMP_DOWN_MS);
  ledcWrite(BLDC_PIN, BLDC_STOP);

  // 8. Servo returns home
  Serial.println("[CYCLE] 8. Servo returning 90 → 0°");
  sweepServoSmooth(SERVO_SWEEP, SERVO_HOME, SERVO_MOVE_MS);
  reportComponentState();

  // 9. Relay OFF — waste drops into bin
  delay(300);
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("[CYCLE] 9. Electromagnet OFF — waste released");
  reportComponentState();

  sendStatusMessage("Collection complete — resuming navigation");
  Serial.printf("\n[CYCLE] Done in %.1f sec\n\n", (millis() - cycleStart) / 1000.0f);
  delay(500);
}


// ════════════════════════════════════════════════════════════
//  BLDC RAMP
// ════════════════════════════════════════════════════════════
void rampBLDC(int startVal, int endVal, int durationMs) {
  float step    = (float)(endVal - startVal) / BLDC_RAMP_STEPS;
  int   delayMs = durationMs / BLDC_RAMP_STEPS;
  for (int i = 0; i <= BLDC_RAMP_STEPS; i++) {
    int val = startVal + (int)(i * step);
    val = constrain(val, min(startVal, endVal), max(startVal, endVal));
    ledcWrite(BLDC_PIN, val);
    delay(delayMs);
  }
}


// ════════════════════════════════════════════════════════════
//  SERVO SMOOTH SWEEP
// ════════════════════════════════════════════════════════════
void sweepServoSmooth(int fromAngle, int toAngle, int durationMs) {
  int steps = abs(toAngle - fromAngle);
  if (steps == 0) return;
  int delayMs = max(1, durationMs / steps);
  int dir     = (toAngle > fromAngle) ? 1 : -1;
  for (int a = fromAngle; a != toAngle; a += dir) {
    sweepServo.write(a);
    delay(delayMs);
  }
  sweepServo.write(toAngle);
}


// ════════════════════════════════════════════════════════════
//  NAVIGATION  (waypoint-to-waypoint)
// ════════════════════════════════════════════════════════════
void navigateTo(float tx, float ty) {
  float dx   = tx - currentX;
  float dy   = ty - currentY;
  float dist = sqrt(dx * dx + dy * dy) * MM_PER_UNIT;
  float tHd  = atan2(dx, dy) * 180.0f / PI;
  float turn = tHd - currentHeading;
  while (turn >  180.0f) turn -= 360.0f;
  while (turn < -180.0f) turn += 360.0f;

  Serial.printf("[NAV] → (%.2f,%.2f)  dist=%.0fmm  turn=%.1f°\n", tx, ty, dist, turn);

  if (abs(turn) > 5.0f) {
    int ms = (int)(abs(turn) / DEG_PER_SEC * 1000.0f);
    (turn > 0) ? turnRight(ms) : turnLeft(ms);
    motorsStop();
    currentHeading = tHd;
    delay(150);
  }
  if (dist > 20.0f) {
    int ms = (int)(dist / MM_PER_SEC * 1000.0f);
    driveForward(ms);
    motorsStop();
    delay(150);
  }
  currentX = tx;
  currentY = ty;
  Serial.printf("[NAV] Reached (%.2f, %.2f)\n", tx, ty);
  reportPosition();
}


// ════════════════════════════════════════════════════════════
//  MOTOR PRIMITIVES
// ════════════════════════════════════════════════════════════
void setMotorA(int dir, int spd) {
  digitalWrite(MOTOR_A_IN1, dir ==  1 ? HIGH : LOW);
  digitalWrite(MOTOR_A_IN2, dir == -1 ? HIGH : LOW);
  ledcWrite(MOTOR_A_ENA, spd);
}
void setMotorB(int dir, int spd) {
  digitalWrite(MOTOR_B_IN3, dir ==  1 ? HIGH : LOW);
  digitalWrite(MOTOR_B_IN4, dir == -1 ? HIGH : LOW);
  ledcWrite(MOTOR_B_ENB, spd);
}
void driveForward(int ms)  { setMotorA(1,  DRIVE_SPEED); setMotorB(1,  DRIVE_SPEED); delay(ms); }
void driveBackward(int ms) { setMotorA(-1, DRIVE_SPEED); setMotorB(-1, DRIVE_SPEED); delay(ms); }
void turnRight(int ms)     { setMotorA(1,  DRIVE_SPEED); setMotorB(-1, DRIVE_SPEED); delay(ms); }
void turnLeft(int ms)      { setMotorA(-1, DRIVE_SPEED); setMotorB(1,  DRIVE_SPEED); delay(ms); }
void motorsStop()          { setMotorA(0, 0); setMotorB(0, 0); }
