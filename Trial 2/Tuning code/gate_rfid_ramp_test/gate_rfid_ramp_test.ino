// Gate RFID + ramp tuner.
// Sends the known airlock tag to the server, waits for accepted=true, then uses
// the front ultrasonic sensor to confirm that the door is actually open before
// driving into the ramp. The ramp controller is deliberately soft to avoid wall
// crashes: tune rampBaseSpeed, wallKpUp, wallKpDown, wallKd, and doorOpenCm.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MiniMessenger.h>
#include <Motoron.h>
#include <Wire.h>
#include <math.h>

MotoronI2C mc1(18);
MotoronI2C mc2(17);
MiniMessenger messenger;

void onCommsMessage(const MessageMetadata& meta, const uint8_t* payload, size_t length);

const char* ssid   = "PhaseSpaceNetwork_2.4G";
const char* pass   = "8igMacNet";
const char* broker = "192.168.0.74";
const uint16_t port = 1883;
const char* group  = "6";
const char* board  = "LEAK";

const char* AIRLOCK_TAG_ID = "C2834BF4";

const uint8_t TRIG_F = 37;
const uint8_t ECHO_F = 36;
const uint8_t TRIG_L = 41;
const uint8_t ECHO_L = 40;
const uint8_t TRIG_R = 39;
const uint8_t ECHO_R = 38;
const long ECHO_TIMEOUT = 25000;

const int bumper1Pin = 22;
const int bumper2Pin = 33;
const int redPin     = 46;
const int greenPin   = 47;

const int MAX_MOTOR_SPEED = 660;
const int MIN_FORWARD_SPEED = 200;

// Ramp wall-following tuning
int rampBaseSpeed     = 400;
float wallKpUp        = 25.0f;
float wallKpDown      = 10.0f;
float wallKd          = 3.0f;
int tunnelWallDist    = 20;
int doorDistanceCm    = 8;
int doorOpenCm        = 18;
int downhillReduction = 300;
float tiltThreshold   = 5.0f;
float downhillThreshold = -20.0f;
float compAlpha       = 0.90f;

enum State { ST_WAIT_REPLY, ST_WAIT_DOOR_OPEN, ST_RAMP, ST_DONE };
State state = ST_WAIT_REPLY;

bool airlockAccepted    = false;
bool airlockDenied      = false;
bool mpuOk              = false;
bool wasOnRamp          = false;
bool exitingThroughDoor = false;
bool requestSent        = false;
unsigned long rampOnSinceMs = 0;

float pitch       = 0.0f;
float wfLastError = 0.0f;
unsigned long lastTiltUs      = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastRegisterMs  = 0;
unsigned long airlockWaitMs   = 0;
unsigned long lastPrintMs     = 0;
int doorOpenHits = 0;

const unsigned long HEARTBEAT_TIMEOUT_MS = 3000;
const unsigned long AIRLOCK_RETRY_MS     = 3000;

bool printTelemetry = false;
bool running        = false;
volatile int enable = 1;

Adafruit_MPU6050 mpu;

// ── helpers ───────────────────────────────────────────────────────────────────

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  for (int ch = 1; ch <= 3; ch++) {
    mc.setMaxAcceleration(ch, 300);
    mc.setMaxDeceleration(ch, 500);
  }
}

void setDrive(int left, int right) {
  left  = constrain(left,  -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  right = constrain(right, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  mc1.setSpeed(2, right);
  mc1.setSpeed(3, left);
  mc2.setSpeed(2, right);
  mc2.setSpeed(3, left);
}

void stopMotors() {
  for (int ch = 1; ch <= 3; ch++) {
    mc1.setSpeed(ch, 0);
    mc2.setSpeed(ch, 0);
  }
}

void updateLEDs() {
  bool contact = digitalRead(bumper1Pin) == LOW || digitalRead(bumper2Pin) == LOW;
  digitalWrite(redPin,   contact ? LOW  : HIGH);
  digitalWrite(greenPin, contact ? HIGH : LOW);
}

int readDistance(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, ECHO_TIMEOUT);
  return d == 0 ? 999 : (int)(d / 58L);
}

float getTilt() {
  if (!mpuOk) return 0.0f;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long now = micros();
  float dt = (lastTiltUs == 0) ? 0.0f : (now - lastTiltUs) / 1000000.0f;
  lastTiltUs = now;
  float pitchY = atan2(a.acceleration.y,
                       sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
  pitch = compAlpha * (pitch + g.gyro.x * dt * 180.0f / PI) + (1.0f - compAlpha) * pitchY;
  return pitch;
}

// ── comms ─────────────────────────────────────────────────────────────────────

void onCommsMessage(const MessageMetadata& meta, const uint8_t* payload, size_t length) {
  if (length == 6) {
    if (payload[4]) { enable = 0; running = false; stopMotors(); }
    return;
  }
  if (length == 21) return;

  char msg[MiniMessenger::kMaxPayloadSize + 1];
  size_t n = min(length, (size_t)MiniMessenger::kMaxPayloadSize);
  memcpy(msg, payload, n);
  msg[n] = '\0';

  if (strstr(msg, "type=heartbeat")) {
    lastHeartbeatMs = millis();
    if      (strstr(msg, "enable=1")) enable = 1;
    else if (strstr(msg, "enable=0")) { enable = 0; running = false; stopMotors(); }
    return;
  }

  if (strstr(msg, "type=disable") || strstr(msg, "type=emergency")) {
    enable = 0; running = false; stopMotors();
    return;
  }

  if (strstr(msg, "type=openAirlockReply")) {
    airlockAccepted = strstr(msg, "accepted=true") != nullptr;
    airlockDenied   = !airlockAccepted;
    if (airlockAccepted) {
      Serial.println("[DOOR] Accepted — waiting for ultrasonic door clearance.");
      state = ST_WAIT_DOOR_OPEN;
      running = true;
      doorOpenHits = 0;
    } else {
      Serial.print("[DOOR] Denied");
      char* reason = strstr(msg, "reason=");
      if (reason) { Serial.print(" — "); Serial.println(reason + 7); }
      else Serial.println();
    }
  }
}

void heartbeatCheck() {
  if (lastHeartbeatMs == 0) return;
  if (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS && enable) {
    enable = 0; running = false; stopMotors();
    Serial.println("[COMMS] Heartbeat timeout.");
  }
}

void commsLoop() {
  if (!messenger.isConnected()) return;
  unsigned long now = millis();
  if (now - lastRegisterMs > 10000) {
    lastRegisterMs = now;
    messenger.sendToBoard("server", "type=register");
  }
}

bool requestAirlock() {
  airlockAccepted = false;
  airlockDenied   = false;
  char msg[112];
  snprintf(msg, sizeof(msg),
           "type=openAirlock airlock=A tag_id=%s team_id=%s board_id=%s",
           AIRLOCK_TAG_ID, group, board);
  bool sent = messenger.sendToBoard("server", msg);
  if (!sent) airlockDenied = true;
  Serial.print("[DOOR] Sent tag ");
  Serial.print(AIRLOCK_TAG_ID);
  Serial.println(sent ? " — request sent." : " — SEND FAILED.");
  return sent;
}

bool doorIsOpenStable() {
  int front = readDistance(TRIG_F, ECHO_F);
  if (front >= doorOpenCm) {
    doorOpenHits++;
  } else {
    doorOpenHits = 0;
  }

  if (printTelemetry && millis() - lastPrintMs > 250) {
    lastPrintMs = millis();
    Serial.print("[DOOR] front=");
    Serial.print(front);
    Serial.print(" openCm=");
    Serial.print(doorOpenCm);
    Serial.print(" hits=");
    Serial.println(doorOpenHits);
  }

  return doorOpenHits >= 3;
}

// ── ramp ──────────────────────────────────────────────────────────────────────

bool waitForDoorIfClosed() {
  int front = readDistance(TRIG_F, ECHO_F);
  if (front >= doorDistanceCm) return false;
  stopMotors();
  Serial.print("[RAMP] Door at "); Serial.print(front); Serial.println(" cm — waiting.");
  while (readDistance(TRIG_F, ECHO_F) < doorDistanceCm) {
    updateLEDs();
    messenger.loop();
    delay(100);
  }
  Serial.println("[RAMP] Door open.");
  return true;
}

void rampStep() {
  float tilt      = getTilt();
  int distL       = readDistance(TRIG_L, ECHO_L);
  int distR       = readDistance(TRIG_R, ECHO_R);
  bool tilted     = fabsf(tilt) > tiltThreshold;
  bool wallsClose = distL < tunnelWallDist && distR < tunnelWallDist;

  if (!tilted && !wallsClose) {
    wasOnRamp = false;
    rampOnSinceMs = 0;
    setDrive(rampBaseSpeed, rampBaseSpeed);
    if (printTelemetry && millis() - lastPrintMs > 250) {
      lastPrintMs = millis();
      Serial.print("[RAMP] Searching. tilt="); Serial.print(tilt, 1);
      Serial.print(" L="); Serial.print(distL);
      Serial.print(" R="); Serial.println(distR);
    }
    return;
  }

  if (!wasOnRamp) rampOnSinceMs = millis();
  wasOnRamp = true;
  bool goingDown  = tilt < downhillThreshold;
  bool doorOpened = waitForDoorIfClosed();
  if (goingDown && doorOpened) { exitingThroughDoor = true; Serial.println("[RAMP] Downhill door opened."); }

  int base = rampBaseSpeed;
  if (goingDown && !exitingThroughDoor)
    base = max(MIN_FORWARD_SPEED, rampBaseSpeed - downhillReduction);

  int error      = distL - distR;
  int derivative = error - (int)wfLastError;
  wfLastError    = error;
  float kp       = (goingDown && !exitingThroughDoor) ? wallKpDown : wallKpUp;
  int correction = constrain((int)(kp * error + wallKd * derivative), -220, 220);
  int left       = constrain(base + correction, 0, MAX_MOTOR_SPEED);
  int right      = constrain(base - correction, 0, MAX_MOTOR_SPEED);
  setDrive(left, right);

  if (printTelemetry && millis() - lastPrintMs > 250) {
    lastPrintMs = millis();
    Serial.print("[RAMP] tilt="); Serial.print(tilt, 1);
    Serial.print(" L="); Serial.print(distL);
    Serial.print(" R="); Serial.print(distR);
    Serial.print(" corr="); Serial.print(correction);
    Serial.print(" mL="); Serial.print(left);
    Serial.print(" mR="); Serial.println(right);
  }
}

// ── serial ────────────────────────────────────────────────────────────────────

void printStatus() {
  const char* names[] = {"WAIT_REPLY", "WAIT_DOOR_OPEN", "RAMP", "DONE"};
  Serial.print("state="); Serial.print(names[state]);
  Serial.print(" running="); Serial.print(running ? "true" : "false");
  Serial.print(" requestSent="); Serial.print(requestSent ? "true" : "false");
  Serial.print(" rampBase="); Serial.print(rampBaseSpeed);
  Serial.print(" KpUp="); Serial.print(wallKpUp, 1);
  Serial.print(" KpDown="); Serial.print(wallKpDown, 1);
  Serial.print(" Kd="); Serial.print(wallKd, 1);
  Serial.print(" wallDist="); Serial.print(tunnelWallDist);
  Serial.print(" doorCm="); Serial.print(doorDistanceCm);
  Serial.print(" doorOpenCm="); Serial.print(doorOpenCm);
  Serial.print(" tilt="); Serial.println(getTilt(), 1);
}

void printHelp() {
  Serial.println();
  Serial.println("Gate RFID + ramp test");
  Serial.println("Sends tag C2834BF4 to server, waits for door, drives up ramp.");
  Serial.println("Commands:");
  Serial.println("  g      send airlock request and go");
  Serial.println("  s      stop");
  Serial.println("  r      reset");
  Serial.println("  p      toggle telemetry");
  Serial.println("  + / -  ramp base speed up/down");
  Serial.println("  ] / [  wall KpUp up/down");
  Serial.println("  } / {  wall Kd up/down");
  Serial.println("  u / j  wall dist up/down");
  Serial.println("  d / e  door distance up/down");
  Serial.println("  o / i  door-open distance up/down");
  Serial.println("  c      print status");
  Serial.println("  ?      help");
  printStatus();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      running = true;
      state = ST_WAIT_REPLY;
      requestSent = requestAirlock();
      airlockWaitMs = millis();
      doorOpenHits = 0;
      Serial.println("[GATE] Waiting for server accept...");
    } else if (c == 's' || c == 'S') {
      running = false; stopMotors(); Serial.println("[STOP]");
    } else if (c == 'r' || c == 'R') {
      state = ST_WAIT_REPLY; running = false; requestSent = false;
      airlockAccepted = false; airlockDenied = false;
      wasOnRamp = false; exitingThroughDoor = false;
      doorOpenHits = 0;
      rampOnSinceMs = 0;
      wfLastError = 0.0f; pitch = 0.0f; lastTiltUs = 0;
      stopMotors();
      Serial.println("[RESET]");
    } else if (c == 'p' || c == 'P') {
      printTelemetry = !printTelemetry;
    } else if (c == '+') {
      rampBaseSpeed = constrain(rampBaseSpeed + 25, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED); printStatus();
    } else if (c == '-') {
      rampBaseSpeed = constrain(rampBaseSpeed - 25, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED); printStatus();
    } else if (c == ']') { wallKpUp += 5.0f; printStatus(); }
    else if (c == '[') { wallKpUp = max(5.0f, wallKpUp - 5.0f); printStatus(); }
    else if (c == '}') { wallKd += 1.0f; printStatus(); }
    else if (c == '{') { wallKd = max(0.0f, wallKd - 1.0f); printStatus(); }
    else if (c == 'u' || c == 'U') { tunnelWallDist = constrain(tunnelWallDist + 1, 5, 80); printStatus(); }
    else if (c == 'j' || c == 'J') { tunnelWallDist = constrain(tunnelWallDist - 1, 5, 80); printStatus(); }
    else if (c == 'd' || c == 'D') { doorDistanceCm = constrain(doorDistanceCm + 1, 3, 40); printStatus(); }
    else if (c == 'e' || c == 'E') { doorDistanceCm = constrain(doorDistanceCm - 1, 3, 40); printStatus(); }
    else if (c == 'o' || c == 'O') { doorOpenCm = constrain(doorOpenCm + 1, 5, 80); printStatus(); }
    else if (c == 'i' || c == 'I') { doorOpenCm = constrain(doorOpenCm - 1, 5, 80); printStatus(); }
    else if (c == 'c' || c == 'C') { printStatus(); }
    else if (c == '?') { printHelp(); }
  }
}

// ── setup / loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(bumper1Pin, INPUT_PULLUP);
  pinMode(bumper2Pin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  updateLEDs();

  Wire.begin();
  Wire1.begin();

  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  stopMotors();

  if (mpu.begin()) {
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[MPU] Ready.");
  } else {
    Serial.println("[MPU] Not found — wall distance only.");
  }

  messenger.onMessage(onCommsMessage);
  messenger.begin(ssid, pass, broker, port, group, board);
  Serial.println("[COMMS] Connecting...");

  printHelp();
}

void loop() {
  messenger.loop();
  heartbeatCheck();
  commsLoop();
  updateLEDs();
  handleSerial();

  if (!enable) { stopMotors(); return; }
  if (!running) { stopMotors(); return; }

  switch (state) {
    case ST_WAIT_REPLY:
      stopMotors();
      if (requestSent && airlockDenied && millis() - airlockWaitMs > AIRLOCK_RETRY_MS) {
        airlockWaitMs = millis();
        Serial.println("[GATE] Retrying...");
        requestAirlock();
      }
      break;

    case ST_WAIT_DOOR_OPEN:
      stopMotors();
      if (doorIsOpenStable()) {
        Serial.println("[DOOR] Ultrasonic clear — moving into ramp.");
        state = ST_RAMP;
        wasOnRamp = false;
        exitingThroughDoor = false;
        wfLastError = 0.0f;
      }
      break;

    case ST_RAMP:
      rampStep();
      if (wasOnRamp && millis() - rampOnSinceMs > 2000) {
        float tilt  = getTilt();
        int distL   = readDistance(TRIG_L, ECHO_L);
        int distR   = readDistance(TRIG_R, ECHO_R);
        bool onRamp = fabsf(tilt) > tiltThreshold || (distL < tunnelWallDist && distR < tunnelWallDist);
        if (!onRamp) {
          stopMotors();
          state   = ST_DONE;
          running = false;
          Serial.println("[DONE] Ramp cleared.");
        }
      }
break;

    case ST_DONE:
      stopMotors();
      break;
  }
}
