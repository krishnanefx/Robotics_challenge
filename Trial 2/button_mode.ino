// Button mode rewrite.
// One competition sketch that wraps the standalone tuning sketches into a
// pin-49 mode cycle. It deliberately avoids QTRSensors and uses raw RC IR reads
// everywhere so it can run without the laptop or the QTR library installed.
//
// Mode order on each pin-49 press:
//   1 LINE_TRACKING      hand IR calibration, then raw line follow
//   2 CHAIN              hand IR calibration, gyro bias, airlock route + ramp
//   3 GATE_RFID_RAMP     request door, ultrasonic door check, ramp
//   4 ARENA_RFID_ROUTE   hand IR calibration show, gyro bias, DR route
//   5 DEAD_RECKONING     gyro bias, DR route
//   6 OBSTACLE           DR node turns around obstacle
//   7 REVIVAL            ultrasonic approach, bumper contact, smooth stop
//
// LED convention:
//   Calibration: green while sampling, red when finished.
//   Normal run: green only while bumper 22/33 is pressed, red otherwise.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <Motoron.h>
#include <Wire.h>
#include <math.h>

#define RFID_WIRE Wire2
#define RFID_RESET_PIN -1

MotoronI2C mc1(18);  // ch2/ch3 drive
MotoronI2C mc2(17);  // ch2/ch3 drive
Adafruit_MPU6050 mpu;
MFRC522_I2C* rfid = nullptr;
MiniMessenger messenger;

void onCommsMessage(const MessageMetadata& meta,
                    const uint8_t* payload,
                    size_t length);

const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* pass = "8igMacNet";
const char* broker = "192.168.0.74";
const uint16_t port = 1883;
const char* group = "6";
const char* board = "LEAK";

const uint8_t SensorCount = 9;
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];
uint8_t sensorPins[SensorCount] = {23, 24, 25, 26, 27, 28, 29, 30, 31};
float weights[SensorCount] = {0.15, 0.12, 0.08, 0.04, 0.0, -0.04, -0.08, -0.12, -0.15};

const uint8_t TRIG_F = 37;
const uint8_t ECHO_F = 36;
const uint8_t TRIG_L = 41;
const uint8_t ECHO_L = 40;
const uint8_t TRIG_R = 39;
const uint8_t ECHO_R = 38;
const long ECHO_TIMEOUT = 25000;

const int bumper1Pin = 22;
const int bumper2Pin = 33;
const int modeButtonPin = 49;
const int redPin = 46;
const int greenPin = 47;

const int encoder1PinA = 52;
const int encoder1PinB = 50;
const int encoder2PinA = 53;
const int encoder2PinB = 51;

const String AIRLOCK_A_TAG_ID = "C2834BF4";

enum RobotMode {
  MODE_LINE_TRACKING = 0,
  MODE_CHAIN = 1,
  MODE_GATE_RFID_RAMP = 2,
  MODE_ARENA_RFID_ROUTE = 3,
  MODE_DEAD_RECKONING = 4,
  MODE_OBSTACLE = 5,
  MODE_REVIVAL = 6,
  MODE_COUNT = 7
};

// Gate mode is separate from chain mode because it starts at the door/ramp:
// it requests the airlock immediately instead of finding the RFID while driving.
enum GateState {
  GATE_SEND_REQUEST,
  GATE_WAIT_REPLY,
  GATE_WAIT_DOOR_OPEN,
  GATE_RAMP,
  GATE_DONE
};

// Chain mode starts on the base line, finds the RFID during line following,
// asks the server for entry, then waits for final line loss + accepted reply +
// ultrasonic door clearance before entering the ramp.
enum ChainState {
  CH_APPROACH,
  CH_WAIT_ENTRY,
  CH_TUNNEL,
  CH_ARENA
};

ChainState chainState = CH_APPROACH;
int chainApproachTurnIndex = 0;

// Motor speed limits from minimum_motor_speed.ino. Keep MAX at the safe value
// for the 10.9 V battery; raise minimums only if motors stall from rest.
const int MAX_MOTOR_SPEED = 660;
const int MIN_FORWARD_SPEED = 200;
const int MIN_TURN_SPEED = 400;

// Raw line-sensor calibration timing. The 6 second window is for moving the
// robot by hand over the black line and white floor while the green LED is on.
const uint16_t RAW_SENSOR_TIMEOUT_US = 2500;
const unsigned long IR_CALIBRATION_MS = 6000;
const unsigned long LINE_END_FORWARD_MS = 0;
const unsigned long AFTER_LINE_END_FORWARD_MS = 50;
const int LEFT_TURN_SLOW_SPEED = 500;

// Dead-reckoning tuning copied from dead_reckoning_test.ino. COUNTS_PER_NODE is
// derived from the wheel geometry but can be replaced with the measured value if
// encoder_counts_per_node.ino shows the robot lands short or long.
const float WHEEL_DIAM_CM = 6.5f;
const float TRACK_WIDTH_CM = 17.0f;
const float COUNTS_PER_REV = 144.0f;
const float CM_PER_COUNT = (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;
const int COUNTS_PER_NODE = (int)(25.0f / CM_PER_COUNT);
const int DRIVE_SPEED = 300;
const float DRIVE_KP = 80.0f;
const int TURN_FAST = 660;
const int TURN_SLOW = 566;
const float TURN_SCALE_LEFT = 0.96f;
const float TURN_SCALE_RIGHT = 0.90f;
const float HEADING_ALPHA = 0.98f;

// Shared 6-node assessment path: forward 2 nodes, right, forward 1 node, left,
// forward 2 nodes. Directions use 0=W, 1=N, 2=E, 3=S.
const int NODE_COUNT = 6;
int pathx[NODE_COUNT] = {0, 0, 0, 1, 1, 1};
int pathy[NODE_COUNT] = {0, 1, 2, 2, 3, 4};
char dirNames[4] = {'W', 'N', 'E', 'S'};

const int OBSTACLE_DETECT_DIST = 25;
const unsigned long AVOID_TIMEOUT_MS = 60000UL;

// Raw line-follow tuning from line_tracking_tune.ino. If the robot weaves,
// reduce lineTurnGain; if it does not correct enough, increase it.
int lineBaseSpeed = 300;
float lineTurnGain = 1.0f;
uint16_t rawLineThreshold = 200;
int branchSearchSpeed = 200;
bool flipLineCorrection = false;
bool rawCalibrated = false;

// Ramp/wall-follow tuning from gate_rfid_ramp_test.ino. The Kp/Kd values are
// intentionally softer than the earlier chain values to avoid wall crashes.
int tunnelBaseSpeed = 400;
float wallKpUp = 25.0f;
float wallKpDown = 10.0f;
float wallKd = 3.0f;
int tunnelWallDist = 20;
int doorDistanceCm = 8;
int doorOpenCm = 18;
int downhillReduction = 300;
float tiltThreshold = 5.0f;
float downhillThreshold = -20.0f;
float compAlpha = 0.90f;

bool running = false;
volatile int enable = 1;
RobotMode currentMode = MODE_REVIVAL;
GateState gateState = GATE_SEND_REQUEST;
bool airlockAccepted = false;
bool airlockDenied = false;
bool airlockRequestSent = false;
bool approachReadyForRamp = false;
bool printTelemetry = false;
bool mpuOk = false;
bool wasInTunnel = false;
bool exitingThroughDoor = false;
float pitch = 0.0f;
float wfLastError = 0.0f;
float gyroBiasZ = 0.0f;
float fusedHeading = 0.0f;
float gyroTurnScaleRight = 0.94f;
float gyroTurnScaleLeft = 1.0f;
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
long prevEncL = 0;
long prevEncR = 0;
unsigned long lastTiltUs = 0;
unsigned long lastHeadingUs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastRegisterMs = 0;
unsigned long lastStatusMs = 0;
unsigned long airlockWaitStartMs = 0;
unsigned long gateAirlockWaitMs = 0;
unsigned long avoidStartMs = 0;
int doorOpenHits = 0;
int direction = 1;
int drIndex = 1;
bool drDone = false;
bool modeButtonWasDown = false;
bool pendingModeAdvance = false;
bool calibrationLedOverride = false;

int approachDistanceCm = 40;
int approachSpeed = 300;
int crawlSpeed = 150;
int currentDriveSpeed = 0;
int revivalSpeedStep = 20;
unsigned long revivalRampStepMs = 20;
unsigned long lastRevivalRampMs = 0;
bool revivalDone = false;

const unsigned long HEARTBEAT_TIMEOUT_MS = 3000;
const unsigned long AIRLOCK_RETRY_MS = 3000;

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
  left = constrain(left, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
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

void setCalibrationLed(bool calibrating) {
  calibrationLedOverride = calibrating;
  digitalWrite(greenPin, calibrating ? HIGH : LOW);
  digitalWrite(redPin, calibrating ? LOW : HIGH);
}

bool anyBumperPressed() {
  return digitalRead(bumper1Pin) == LOW || digitalRead(bumper2Pin) == LOW;
}

bool modeButtonPressedEvent() {
  bool down = digitalRead(modeButtonPin) == LOW;
  bool pressed = down && !modeButtonWasDown;
  modeButtonWasDown = down;
  if (pressed) delay(40);
  return pressed;
}

int readDistance(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, ECHO_TIMEOUT);
  return duration == 0 ? 999 : (int)(duration / 58L);
}

void updateLEDs() {
  if (calibrationLedOverride) return;
  bool contact = anyBumperPressed();
  digitalWrite(redPin, contact ? LOW : HIGH);
  digitalWrite(greenPin, contact ? HIGH : LOW);
}

void onCommsMessage(const MessageMetadata& meta,
                    const uint8_t* payload,
                    size_t length) {
  if (length == 6) {
    if (payload[4]) {
      enable = 0;
      running = false;
      stopMotors();
      Serial.println("[COMMS] Emergency flag — stopped");
    }
    return;
  }
  if (length == 21) return;

  char msg[MiniMessenger::kMaxPayloadSize + 1];
  size_t n = min(length, (size_t)MiniMessenger::kMaxPayloadSize);
  memcpy(msg, payload, n);
  msg[n] = '\0';

  Serial.print("[COMMS] from=");
  Serial.print(meta.fromBoardId);
  Serial.print(" | ");
  Serial.println(msg);

  if (strstr(msg, "type=heartbeat")) {
    lastHeartbeatMs = millis();
    if (strstr(msg, "enable=1")) {
      enable = 1;
    } else if (strstr(msg, "enable=0")) {
      enable = 0;
      running = false;
      stopMotors();
    }
    return;
  }

  if (strstr(msg, "type=disable") || strstr(msg, "type=emergency")) {
    enable = 0;
    running = false;
    stopMotors();
    Serial.println("[COMMS] Disable/emergency — stopped");
    return;
  }

  if (strstr(msg, "type=openAirlockReply")) {
    airlockAccepted = strstr(msg, "accepted=true") != nullptr;
    airlockDenied = !airlockAccepted;

    if (airlockAccepted) {
      Serial.println("[DOOR] Airlock accepted");
      bool replyForEntry = strstr(msg, "airlock=A") != nullptr ||
                           strstr(msg, "airlock=") == nullptr;
      if (replyForEntry) {
        if (approachReadyForRamp) {
          Serial.println("[DOOR] Entry granted — waiting for ultrasonic door clearance");
        } else {
          Serial.println("[DOOR] Entry granted — will enter ramp after line tracking ends");
        }
      }
    } else {
      Serial.print("[DOOR] Airlock denied");
      char* reason = strstr(msg, "reason=");
      if (reason) {
        Serial.print(" — ");
        Serial.println(reason + 7);
      } else {
        Serial.println();
      }
    }
  }
}

void commsHeartbeatCheck() {
  if (lastHeartbeatMs == 0) return;
  if (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    if (enable) {
      enable = 0;
      running = false;
      stopMotors();
      Serial.println("[COMMS] Heartbeat timeout — stopped");
    }
  }
}

void sendStatusUpdate() {
  if (!messenger.isConnected()) return;

  unsigned long now = millis();
  if (now - lastRegisterMs > 10000) {
    lastRegisterMs = now;
    messenger.sendToBoard("server", "type=register");
    Serial.println("[COMMS] Registered with server");
  }

  if (now - lastStatusMs > 2000) {
    lastStatusMs = now;
    char status[96];
    snprintf(status, sizeof(status),
             "STATUS:BUTTON_MODE mode=%s enable=%d running=%d",
             modeName(currentMode), enable, running ? 1 : 0);
    messenger.sendToGroup(status);
  }
}

bool requestAirlock(const char* airlockId, const char* tagId) {
  airlockAccepted = false;
  airlockDenied = false;

  char msg[112];
  snprintf(msg, sizeof(msg),
           "type=openAirlock airlock=%s tag_id=%s team_id=%s board_id=%s",
           airlockId, tagId, group, board);

  bool sent = messenger.sendToBoard("server", msg);
  airlockRequestSent = sent;
  if (!sent) airlockDenied = true;

  Serial.print("[DOOR] Requested airlock ");
  Serial.print(airlockId);
  Serial.print(" with tag ");
  Serial.print(tagId);
  Serial.println(sent ? " — sent" : " — SEND FAILED");
  return sent;
}

byte findRfidAddress() {
  byte foundAddress = 0;
  byte foundCount = 0;

  Serial.println("[RFID] Scanning Wire2 I2C bus...");
  for (byte address = 1; address < 127; address++) {
    RFID_WIRE.beginTransmission(address);
    if (RFID_WIRE.endTransmission() == 0) {
      if (foundAddress == 0) foundAddress = address;
      foundCount++;
      Serial.print("[RFID] Found device at 0x");
      if (address < 0x10) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (foundCount == 0) Serial.println("[RFID] No reader found; RFID stop disabled.");
  if (foundCount > 1) Serial.println("[RFID] Multiple devices found; using first.");
  return foundAddress;
}

String getTagId() {
  String id = "";
  for (byte i = 0; i < rfid->uid.size; i++) {
    if (rfid->uid.uidByte[i] < 0x10) id += "0";
    id += String(rfid->uid.uidByte[i], HEX);
  }
  id.toUpperCase();
  return id;
}

bool readRfidTag(String& tagId) {
  if (rfid == nullptr) return false;
  if (!rfid->PICC_IsNewCardPresent()) return false;
  if (!rfid->PICC_ReadCardSerial()) return false;

  tagId = getTagId();
  rfid->PICC_HaltA();
  return true;
}

bool checkAirlockTagDuringApproach() {
  // Only Chain mode is allowed to request the door from RFID detection. Plain
  // line tracking uses the same raw sensors but should not talk to the server.
  if (currentMode != MODE_CHAIN) return false;
  if (chainState != CH_APPROACH) return false;
  if (airlockRequestSent) return false;

  String tagId;
  if (!readRfidTag(tagId)) return false;

  Serial.print("[RFID] Tag=");
  Serial.println(tagId);
  if (tagId != AIRLOCK_A_TAG_ID) return false;

  if (requestAirlock("A", tagId.c_str())) {
    airlockWaitStartMs = millis();
    Serial.println("[CHAIN] Airlock tag matched. Request sent; continuing approach.");
  } else {
    Serial.println("[CHAIN] Airlock request failed. Continuing approach.");
  }
  return false;
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
                            a.acceleration.z * a.acceleration.z)) *
                 180.0f / PI;

  pitch = compAlpha * (pitch + g.gyro.x * dt * 180.0f / PI) +
          (1.0f - compAlpha) * pitchY;
  return pitch;
}

float readGyroZRadPerSec() {
  if (!mpuOk) return 0.0f;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return g.gyro.z;
}

void calibrateGyroBias() {
  if (!mpuOk) return;

  stopMotors();
  Serial.println("[GYRO] Keep robot still. Calibrating yaw bias...");
  delay(100);

  float sum = 0.0f;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    sum += readGyroZRadPerSec();
    delay(3);
  }
  gyroBiasZ = sum / samples;
  Serial.print("[GYRO] Bias Z=");
  Serial.println(gyroBiasZ, 6);
}

const char* stateName(ChainState state) {
  switch (state) {
    case CH_APPROACH: return "APPROACH";
    case CH_WAIT_ENTRY: return "WAIT_ENTRY";
    case CH_TUNNEL: return "TUNNEL";
    case CH_ARENA: return "ARENA";
  }
  return "?";
}

float readLineError() {
  readRawLineSensors();
  float error = 0.0f;
  for (uint8_t i = 0; i < SensorCount; i++) {
    error += normalizedRawValue(i) * weights[i];
  }
  return flipLineCorrection ? -error : error;
}

void readRawLineSensors() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);

  unsigned long start = micros();
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = RAW_SENSOR_TIMEOUT_US;
  }

  bool done = false;
  while (!done && micros() - start < RAW_SENSOR_TIMEOUT_US) {
    unsigned long elapsed = micros() - start;
    done = true;
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == RAW_SENSOR_TIMEOUT_US && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = elapsed;
      }
      if (sensorValues[i] == RAW_SENSOR_TIMEOUT_US) done = false;
    }
  }
}

uint16_t normalizedRawValue(uint8_t i) {
  if (!rawCalibrated) return sensorValues[i];
  if (sensorMax[i] <= sensorMin[i] + 5) return sensorValues[i];

  long value = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
  return constrain(value, 0, 1000);
}

void followLineStep() {
  if (checkAirlockTagDuringApproach()) return;
  float error = readLineError();
  int correction = (int)(error * lineTurnGain);
  int left = lineBaseSpeed - correction;
  int right = lineBaseSpeed + correction;
  setDrive(left, right);

  if (printTelemetry && millis() - lastPrintMs > 250) {
    lastPrintMs = millis();
    Serial.print("[LINE] state=");
    Serial.print(stateName(chainState));
    Serial.print(" err=");
    Serial.print(error, 1);
    Serial.print(" corr=");
    Serial.print(correction);
    Serial.print(" L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.println(right);
  }
}

bool serialStopRequested() {
  if (modeButtonPressedEvent()) {
    pendingModeAdvance = true;
    running = false;
    stopMotors();
    Serial.println("[MODE] Button pressed — switching mode.");
    return true;
  }

  if (!Serial.available()) return false;
  char c = Serial.read();
  if (c != 's' && c != 'S') return false;

  running = false;
  stopMotors();
  Serial.println("[STOP] Motors stopped.");
  return true;
}

bool onLine() {
  readRawLineSensors();
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (rawCalibrated) {
      if (normalizedRawValue(i) > 100) return true;
    } else if (sensorValues[i] > rawLineThreshold) {
      return true;
    }
  }
  return false;
}

bool rightBranchDetected() {
  readRawLineSensors();
  uint8_t active = 0;
  for (uint8_t i = 6; i <= 8; i++) {
    uint16_t value = rawCalibrated ? normalizedRawValue(i) : sensorValues[i];
    if (value > (rawCalibrated ? 100 : rawLineThreshold)) active++;
  }
  return active >= 2;
}

bool confirmRightBranch() {
  // Thick tape can momentarily look like a branch. Stop and require five
  // consecutive right-side readings before committing to the branch turn.
  stopMotors();
  delay(80);
  for (uint8_t i = 0; i < 5; i++) {
    updateLEDs();
    if (serialStopRequested()) return false;
    if (!rightBranchDetected()) {
      Serial.println("[CHAIN] Branch candidate rejected.");
      return false;
    }
    delay(20);
  }
  return true;
}

void spinLeft(int speed) {
  setDrive(-speed, speed);
}

void spinRight(int speed) {
  setDrive(speed, -speed);
}

void driveForwardForMs(unsigned long durationMs, int speed) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    setDrive(speed, speed);
  }
  stopMotors();
}

void trackLineUntilEnd() {
  Serial.println("[CHAIN] Line tracking until next line end.");
  while (onLine()) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    followLineStep();
  }
  stopMotors();
  Serial.println("[CHAIN] Next line end detected.");
}

void trackLineUntilRightBranch() {
  Serial.println("[CHAIN] Line tracking until right branch.");
  while (true) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    if (rightBranchDetected()) {
      Serial.println("[CHAIN] Branch candidate — stopping to confirm.");
      if (confirmRightBranch()) break;
    }
    float error = readLineError();
    int correction = (int)(error * lineTurnGain);
    int left = branchSearchSpeed - correction;
    int right = branchSearchSpeed + correction;
    setDrive(left, right);
  }
  stopMotors();
  Serial.println("[CHAIN] Right branch detected.");
}

void resetRawCalibration() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = RAW_SENSOR_TIMEOUT_US;
    sensorMax[i] = 0;
  }
  rawCalibrated = false;
}

void sampleRawCalibration() {
  readRawLineSensors();
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < sensorMin[i]) sensorMin[i] = sensorValues[i];
    if (sensorValues[i] > sensorMax[i]) sensorMax[i] = sensorValues[i];
  }
}

void autoCalibrateRawLine() {
  Serial.println("[RAW] Hand calibration starting. Move robot left/right over the line.");
  resetRawCalibration();
  stopMotors();
  setCalibrationLed(true);

  // Sample min/max raw timing values while the robot is moved by hand. This
  // replaces QTR calibration and gives normalizedRawValue() a 0..1000 range.
  unsigned long start = millis();
  while (millis() - start < IR_CALIBRATION_MS) {
    if (modeButtonPressedEvent()) {
      pendingModeAdvance = true;
      break;
    }
    sampleRawCalibration();
    delay(20);
  }

  stopMotors();
  rawCalibrated = true;
  setCalibrationLed(false);
  Serial.println("[RAW] Hand calibration done.");
  delay(500);
}

void turnLeftRaw() {
  Serial.println("[TURN] Left: lose line, then find next line.");
  while (onLine()) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    spinLeft(500);
  }
  while (!onLine()) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    spinLeft(500);
  }
  stopMotors();
  Serial.println("[TURN] Left done.");
}

void turnRightRaw() {
  Serial.println("[TURN] Right: lose line, then find next line.");
  while (onLine()) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    spinRight(500);
  }
  while (!onLine()) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    spinRight(500);
  }
  stopMotors();
  Serial.println("[TURN] Right done.");
}

void gyroTurnInPlace(float angleRad) {
  if (!mpuOk) {
    Serial.println("[GYRO] MPU missing; falling back to raw line turn.");
    if (angleRad < 0.0f) turnRightRaw();
    else turnLeftRaw();
    return;
  }

  int sign = (angleRad > 0.0f) ? 1 : -1;
  float scale = (sign > 0) ? gyroTurnScaleLeft : gyroTurnScaleRight;
  float target = fabsf(angleRad) * scale;
  float accumulated = 0.0f;
  unsigned long prevUs = micros();

  Serial.print("[GYRO] Turning ");
  Serial.println(sign < 0 ? "right 90" : "left 90");

  while (accumulated < target) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;

    unsigned long now = micros();
    float dt = (now - prevUs) / 1000000.0f;
    prevUs = now;
    if (dt <= 0.0f || dt > 0.05f) continue;

    accumulated += fabsf((readGyroZRadPerSec() - gyroBiasZ) * dt);

    float remaining = target - accumulated;
    int slowSpeed = (sign > 0) ? LEFT_TURN_SLOW_SPEED : 400;
    int speed = remaining > target * 0.35f ? 660 : slowSpeed;
    mc1.setSpeed(2, sign * speed);
    mc1.setSpeed(3, -sign * speed);
    mc2.setSpeed(2, sign * speed);
    mc2.setSpeed(3, -sign * speed);
  }

  stopMotors();
  delay(150);
  calibrateGyroBias();
  Serial.println("[GYRO] Turn done.");
}

void gyroTurnRight90() {
  gyroTurnInPlace(-PI / 2.0f);
}

void gyroTurnLeft90() {
  gyroTurnInPlace(PI / 2.0f);
}

void runApproachPathStep() {
  // Chain approach sequence:
  //   follow line to end, right turn, line to end, left, line to end, left,
  //   search slowly for a right branch, right, then follow until final line loss.
  if (checkAirlockTagDuringApproach()) return;

  if (onLine()) {
    followLineStep();
    return;
  }

  if (chainApproachTurnIndex > 0) {
    stopMotors();
    Serial.println("[CHAIN] Right-forward-left maneuver complete; line lost before airlock RFID.");
    return;
  }

  stopMotors();
  Serial.println("[CHAIN] Pausing before first right turn.");
  delay(500);

  Serial.println("[CHAIN] Gyro right, two line-end left turns, then right branch turn.");
  driveForwardForMs(30, lineBaseSpeed);  // turn 1: right — 100 ms
  if (chainState != CH_APPROACH) return;
  gyroTurnRight90();
  if (chainState != CH_APPROACH) return;
  stopMotors();
  delay(200);

  trackLineUntilEnd();
  if (chainState != CH_APPROACH) return;
  driveForwardForMs(50, lineBaseSpeed);  // turn 2: first left — 100 ms
  if (chainState != CH_APPROACH) return;
  delay(200);

  gyroTurnLeft90();
  if (chainState != CH_APPROACH) return;
  stopMotors();
  delay(200);

  trackLineUntilEnd();
  if (chainState != CH_APPROACH) return;
  driveForwardForMs(150, lineBaseSpeed);
  if (chainState != CH_APPROACH) return;
  delay(200);

  gyroTurnLeft90();
  if (chainState != CH_APPROACH) return;
  stopMotors();
  delay(200);

  trackLineUntilRightBranch();
  if (chainState != CH_APPROACH) return;
  driveForwardForMs(150, lineBaseSpeed);
  if (chainState != CH_APPROACH) return;
  delay(200);
  
  gyroTurnRight90();
  if (chainState != CH_APPROACH) return;
  stopMotors();
  delay(200);

  Serial.println("[CHAIN] Following line after last turn; will enter ramp on line loss.");
  while (onLine()) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    followLineStep();
  }

  approachReadyForRamp = true;
  chainApproachTurnIndex++;
  wasInTunnel = false;
  exitingThroughDoor = false;
  running = true;

  if (airlockAccepted && doorIsOpenStable()) {
    chainState = CH_TUNNEL;
    doorOpenHits = 0;
    Serial.println("[CHAIN] Line lost and door clear — entering ramp mode.");
  } else {
    chainState = CH_WAIT_ENTRY;
    stopMotors();
    Serial.println("[CHAIN] Line lost — waiting for server accept and ultrasonic door clearance.");
  }
}

bool waitForDoorIfClosed() {
  int front = readDistance(TRIG_F, ECHO_F);
  if (front >= doorDistanceCm) return false;

  stopMotors();
  Serial.print("[TUNNEL] Door/obstacle at ");
  Serial.print(front);
  Serial.println(" cm. Waiting. Send s to stop.");

  while (readDistance(TRIG_F, ECHO_F) < doorDistanceCm) {
    updateLEDs();
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        running = false;
        stopMotors();
        Serial.println("[STOP] Stopped while waiting for door.");
        return false;
      }
    }
    delay(100);
  }

  Serial.println("[TUNNEL] Door open.");
  return true;
}

bool doorIsOpenStable() {
  // Require three clear front readings so one noisy ultrasonic sample does not
  // make the robot drive into a still-closed door.
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

bool navigateTunnelStep() {
  // Ramp mode starts by driving forward until tilt or close side walls prove the
  // robot is inside the tunnel, then switches to ultrasonic wall following.
  float tilt = getTilt();
  int distL = readDistance(TRIG_L, ECHO_L);
  int distR = readDistance(TRIG_R, ECHO_R);
  bool tilted = fabsf(tilt) > tiltThreshold;
  bool wallsClose = distL < tunnelWallDist && distR < tunnelWallDist;

  if (!tilted && !wallsClose) {
    if (printTelemetry && millis() - lastPrintMs > 250) {
      lastPrintMs = millis();
      Serial.print("[TUNNEL] Not in tunnel. tilt=");
      Serial.print(tilt, 1);
      Serial.print(" L=");
      Serial.print(distL);
      Serial.print(" R=");
      Serial.println(distR);
    }
    return false;
  }

  bool goingDown = tilt < downhillThreshold;
  bool doorOpened = waitForDoorIfClosed();
  if (goingDown && doorOpened) {
    exitingThroughDoor = true;
    Serial.println("[TUNNEL] Downhill door opened; using exit speed.");
  }

  int base = tunnelBaseSpeed;
  if (goingDown && !exitingThroughDoor) {
    base = max(MIN_FORWARD_SPEED, tunnelBaseSpeed - downhillReduction);
  }

  int error = distL - distR;
  int derivative = error - (int)wfLastError;
  wfLastError = error;

  float kp = (goingDown && !exitingThroughDoor) ? wallKpDown : wallKpUp;
  int correction = constrain((int)(kp * error + wallKd * derivative), -220, 220);
  int left = constrain(base + correction, 0, MAX_MOTOR_SPEED);
  int right = constrain(base - correction, 0, MAX_MOTOR_SPEED);
  setDrive(left, right);

  if (printTelemetry && millis() - lastPrintMs > 250) {
    lastPrintMs = millis();
    Serial.print("[TUNNEL] tilt=");
    Serial.print(tilt, 1);
    Serial.print(" Lcm=");
    Serial.print(distL);
    Serial.print(" Rcm=");
    Serial.print(distR);
    Serial.print(" err=");
    Serial.print(error);
    Serial.print(" corr=");
    Serial.print(correction);
    Serial.print(" base=");
    Serial.print(base);
    Serial.print(" motorL=");
    Serial.print(left);
    Serial.print(" motorR=");
    Serial.println(right);
  }

  return true;
}

void resetChain() {
  stopMotors();
  chainState = CH_APPROACH;
  chainApproachTurnIndex = 0;
  running = false;
  wasInTunnel = false;
  exitingThroughDoor = false;
  airlockAccepted = false;
  airlockDenied = false;
  airlockRequestSent = false;
  approachReadyForRamp = false;
  doorOpenHits = 0;
  wfLastError = 0.0f;
  pitch = 0.0f;
  lastTiltUs = 0;
  Serial.println("[RESET] CH_APPROACH, stopped.");
}

void runChainStep() {
  switch (chainState) {
    case CH_APPROACH: {
      if (checkAirlockTagDuringApproach()) return;
      runApproachPathStep();
      return;
    }

    case CH_WAIT_ENTRY:
      stopMotors();
      if (airlockAccepted && approachReadyForRamp && doorIsOpenStable()) {
        chainState = CH_TUNNEL;
        running = true;
        wasInTunnel = false;
        exitingThroughDoor = false;
        wfLastError = 0.0f;
        doorOpenHits = 0;
        Serial.println("[DOOR] Accepted and ultrasonic clear — entering ramp mode.");
        return;
      }
      if (airlockDenied && millis() - airlockWaitStartMs > AIRLOCK_RETRY_MS) {
        airlockWaitStartMs = millis();
        Serial.println("[CHAIN] Retrying airlock request");
        requestAirlock("A", AIRLOCK_A_TAG_ID.c_str());
      }
      return;

    case CH_TUNNEL:
      if (navigateTunnelStep()) {
        wasInTunnel = true;
        return;
      }
      if (wasInTunnel) {
        stopMotors();
        wasInTunnel = false;
        exitingThroughDoor = false;
        chainState = CH_ARENA;
        Serial.println("[CHAIN] Tunnel cleared. Entering arena line follow.");
        return;
      }
      setDrive(tunnelBaseSpeed, tunnelBaseSpeed);
      return;

    case CH_ARENA:
      followLineStep();
      return;
  }
}

void countEncoder1() {
  if (digitalRead(encoder1PinB) == HIGH) encoderCount1++;
  else encoderCount1--;
}

void countEncoder2() {
  if (digitalRead(encoder2PinB) == LOW) encoderCount2++;
  else encoderCount2--;
}

void zeroDriveState() {
  noInterrupts();
  encoderCount1 = 0;
  encoderCount2 = 0;
  interrupts();
  fusedHeading = 0.0f;
  prevEncL = 0;
  prevEncR = 0;
  lastHeadingUs = micros();
}

void updateHeading() {
  if (!mpuOk) return;
  unsigned long now = micros();
  float dt = (now - lastHeadingUs) / 1000000.0f;
  lastHeadingUs = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  float gyroAngle = (readGyroZRadPerSec() - gyroBiasZ) * dt;
  long curL = encoderCount1;
  long curR = encoderCount2;
  float distL = (float)(curL - prevEncL) * CM_PER_COUNT;
  float distR = (float)(curR - prevEncR) * CM_PER_COUNT;
  prevEncL = curL;
  prevEncR = curR;
  float encAngle = (distR - distL) / TRACK_WIDTH_CM;
  fusedHeading += HEADING_ALPHA * gyroAngle + (1.0f - HEADING_ALPHA) * encAngle;
}

void drTurnInPlace(float angleRad) {
  if (!mpuOk) {
    if (angleRad < 0.0f) gyroTurnRight90();
    else gyroTurnLeft90();
    return;
  }

  int sign = (angleRad > 0.0f) ? 1 : -1;
  float scale = (sign > 0) ? TURN_SCALE_LEFT : TURN_SCALE_RIGHT;
  float target = fabsf(angleRad) * scale;
  float accumulated = 0.0f;
  unsigned long prevUs = micros();

  while (accumulated < target) {
    if (serialStopRequested()) return;

    unsigned long now = micros();
    float dt = (now - prevUs) / 1000000.0f;
    prevUs = now;
    if (dt <= 0.0f || dt > 0.05f) continue;

    accumulated += fabsf((readGyroZRadPerSec() - gyroBiasZ) * dt);
    float remaining = target - accumulated;
    int speed;
    if (remaining > target * 0.333f) {
      speed = TURN_FAST;
    } else {
      float frac = remaining / (target * 0.333f);
      speed = (int)(TURN_SLOW + frac * (float)(TURN_FAST - TURN_SLOW));
    }
    speed = constrain(speed, max(MIN_TURN_SPEED, TURN_SLOW), TURN_FAST);

    mc1.setSpeed(2, sign * speed);
    mc1.setSpeed(3, -sign * speed);
    mc2.setSpeed(2, sign * speed);
    mc2.setSpeed(3, -sign * speed);
  }

  stopMotors();
  direction = (sign > 0) ? (direction - 1 + 4) % 4 : (direction + 1) % 4;
  Serial.print("[DR] Facing ");
  Serial.println(dirNames[direction]);
  calibrateGyroBias();
}

void drTurnLeft() {
  drTurnInPlace(PI / 2.0f);
}

void drTurnRight() {
  drTurnInPlace(-PI / 2.0f);
}

void drFaceDir(int targetDir) {
  int diff = (targetDir - direction + 4) % 4;
  if (diff == 1) drTurnRight();
  else if (diff == 3) drTurnLeft();
  else if (diff == 2) {
    drTurnRight();
    drTurnRight();
  }
}

bool driveToNode() {
  zeroDriveState();
  while (true) {
    updateLEDs();
    if (serialStopRequested()) {
      stopMotors();
      return false;
    }
    updateHeading();

    noInterrupts();
    long leftCount = encoderCount1;
    long rightCount = encoderCount2;
    interrupts();

    long avg = (abs(leftCount) + abs(rightCount)) / 2;
    if (avg >= COUNTS_PER_NODE) break;

    String tagId;
    if (readRfidTag(tagId)) {
      Serial.print("[DR] RFID early node stop tag=");
      Serial.println(tagId);
      break;
    }

    int correction = constrain((int)(DRIVE_KP * fusedHeading), -189, 189);
    int leftSpeed = constrain(DRIVE_SPEED + correction, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
    int rightSpeed = constrain(DRIVE_SPEED - correction, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
    setDrive(leftSpeed, rightSpeed);
  }

  stopMotors();
  calibrateGyroBias();
  return true;
}

void resetDeadReckoningPath() {
  drIndex = 1;
  drDone = false;
  direction = 1;
  stopMotors();
}

void runDeadReckoningPath() {
  // Runs the same path as dead_reckoning_test.ino. If a button press interrupts
  // a blocking turn/drive, pendingModeAdvance makes the next loop start the
  // next mode instead of continuing this route.
  if (drDone) return;

  for (; drIndex < NODE_COUNT && running; drIndex++) {
    int dx = pathx[drIndex] - pathx[drIndex - 1];
    int dy = pathy[drIndex] - pathy[drIndex - 1];

    int targetDir;
    if (dy > 0) targetDir = 1;
    else if (dy < 0) targetDir = 3;
    else if (dx > 0) targetDir = 2;
    else targetDir = 0;

    drFaceDir(targetDir);
    if (!running || pendingModeAdvance) return;
    if (!driveToNode()) return;
  }

  stopMotors();
  drDone = true;
  running = false;
  Serial.println("[DR] Path complete.");
}

bool requestGateAirlock() {
  airlockRequestSent = requestAirlock("A", AIRLOCK_A_TAG_ID.c_str());
  gateAirlockWaitMs = millis();
  return airlockRequestSent;
}

void resetGateMode() {
  gateState = GATE_SEND_REQUEST;
  airlockAccepted = false;
  airlockDenied = false;
  airlockRequestSent = false;
  doorOpenHits = 0;
  wasInTunnel = false;
  exitingThroughDoor = false;
  wfLastError = 0.0f;
  pitch = 0.0f;
  lastTiltUs = 0;
}

void runGateRfidRampStep() {
  // Gate-only test mode: ask for the airlock immediately, wait for the server,
  // wait for ultrasonic clearance, then reuse the ramp/tunnel controller.
  switch (gateState) {
    case GATE_SEND_REQUEST:
      stopMotors();
      requestGateAirlock();
      gateState = GATE_WAIT_REPLY;
      Serial.println("[GATE] Waiting for server accept.");
      return;

    case GATE_WAIT_REPLY:
      stopMotors();
      if (airlockAccepted) {
        doorOpenHits = 0;
        gateState = GATE_WAIT_DOOR_OPEN;
        Serial.println("[GATE] Accepted. Waiting for ultrasonic door clearance.");
      } else if (airlockDenied && millis() - gateAirlockWaitMs > AIRLOCK_RETRY_MS) {
        Serial.println("[GATE] Retrying airlock request.");
        requestGateAirlock();
      }
      return;

    case GATE_WAIT_DOOR_OPEN:
      stopMotors();
      if (doorIsOpenStable()) {
        gateState = GATE_RAMP;
        wasInTunnel = false;
        exitingThroughDoor = false;
        wfLastError = 0.0f;
        Serial.println("[GATE] Door clear. Entering ramp.");
      }
      return;

    case GATE_RAMP:
      if (navigateTunnelStep()) {
        wasInTunnel = true;
        return;
      }
      if (wasInTunnel) {
        stopMotors();
        gateState = GATE_DONE;
        running = false;
        Serial.println("[GATE] Ramp complete.");
        return;
      }
      setDrive(tunnelBaseSpeed, tunnelBaseSpeed);
      return;

    case GATE_DONE:
      stopMotors();
      running = false;
      return;
  }
}

bool obsDriveNodes(uint8_t nodeCount) {
  for (uint8_t i = 0; i < nodeCount; i++) {
    if (!driveToNode()) return false;
    if (millis() - avoidStartMs > AVOID_TIMEOUT_MS) {
      stopMotors();
      Serial.println("[OBS] Timeout.");
      return false;
    }
  }
  return true;
}

bool avoidObstacle() {
  // Node-based box swerve from obstacle_avoidance.ino. It uses the same gyro
  // turns and driveToNode() as dead reckoning instead of timing-only turns.
  avoidStartMs = millis();
  stopMotors();
  calibrateGyroBias();

  int leftDist = readDistance(TRIG_L, ECHO_L);
  delay(15);
  int rightDist = readDistance(TRIG_R, ECHO_R);
  bool goLeft = leftDist >= rightDist;
  Serial.println(goLeft ? "[OBS] Swerve left." : "[OBS] Swerve right.");

  if (goLeft) drTurnLeft(); else drTurnRight();
  if (!obsDriveNodes(1)) return false;
  if (goLeft) drTurnRight(); else drTurnLeft();
  if (!obsDriveNodes(2)) return false;
  if (goLeft) drTurnRight(); else drTurnLeft();
  if (!obsDriveNodes(1)) return false;
  if (goLeft) drTurnLeft(); else drTurnRight();
  if (!obsDriveNodes(1)) return false;

  stopMotors();
  Serial.println("[OBS] Swerve complete.");
  return true;
}

void runObstacleStep() {
  int front = readDistance(TRIG_F, ECHO_F);
  if (front <= OBSTACLE_DETECT_DIST) {
    avoidObstacle();
    return;
  }
  setDrive(DRIVE_SPEED, DRIVE_SPEED);
}

void setDriveSingle(int speed) {
  setDrive(speed, speed);
}

void rampRevivalDriveToward(int targetSpeed) {
  unsigned long now = millis();
  if (now - lastRevivalRampMs < revivalRampStepMs) return;
  lastRevivalRampMs = now;

  if (currentDriveSpeed < targetSpeed) {
    currentDriveSpeed = min(currentDriveSpeed + revivalSpeedStep, targetSpeed);
  } else if (currentDriveSpeed > targetSpeed) {
    currentDriveSpeed = max(currentDriveSpeed - revivalSpeedStep, targetSpeed);
  }
  setDriveSingle(currentDriveSpeed);
}

void smoothRevivalStop() {
  while (currentDriveSpeed != 0) {
    updateLEDs();
    if (modeButtonPressedEvent()) {
      pendingModeAdvance = true;
      break;
    }
    rampRevivalDriveToward(0);
    delay(1);
  }
  stopMotors();
  currentDriveSpeed = 0;
}

void resetRevivalMode() {
  revivalDone = false;
  currentDriveSpeed = 0;
  lastRevivalRampMs = 0;
}

void runRevivalStep() {
  // Revival is intentionally simple: fast approach until the ultrasonic sensor
  // sees the target, crawl into bumper contact, then ramp smoothly to zero.
  if (revivalDone) {
    stopMotors();
    return;
  }

  int front = readDistance(TRIG_F, ECHO_F);
  if (anyBumperPressed()) {
    smoothRevivalStop();
    revivalDone = true;
    running = false;
    Serial.println("[REVIVAL] Bumper contact. Motors stopped.");
    return;
  }

  if (front > approachDistanceCm) {
    rampRevivalDriveToward(approachSpeed);
  } else {
    rampRevivalDriveToward(crawlSpeed);
  }
}

const char* modeName(RobotMode mode) {
  switch (mode) {
    case MODE_LINE_TRACKING: return "LINE_TRACKING";
    case MODE_CHAIN: return "CHAIN";
    case MODE_GATE_RFID_RAMP: return "GATE_RFID_RAMP";
    case MODE_ARENA_RFID_ROUTE: return "ARENA_RFID_ROUTE";
    case MODE_DEAD_RECKONING: return "DEAD_RECKONING";
    case MODE_OBSTACLE: return "OBSTACLE";
    case MODE_REVIVAL: return "REVIVAL";
    default: return "?";
  }
}

void startMode(RobotMode mode) {
  // Every mode begins from stopped motors. Modes that need line sensors perform
  // hand calibration before running, so the robot can start without Serial input.
  stopMotors();
  running = false;
  pendingModeAdvance = false;
  calibrationLedOverride = false;
  updateLEDs();

  currentMode = mode;
  Serial.print("[MODE] Starting ");
  Serial.println(modeName(currentMode));

  if (currentMode == MODE_LINE_TRACKING) {
    autoCalibrateRawLine();
    running = !pendingModeAdvance;
  } else if (currentMode == MODE_CHAIN) {
    resetChain();
    autoCalibrateRawLine();
    calibrateGyroBias();
    running = !pendingModeAdvance;
  } else if (currentMode == MODE_GATE_RFID_RAMP) {
    resetGateMode();
    running = true;
  } else if (currentMode == MODE_ARENA_RFID_ROUTE) {
    autoCalibrateRawLine();
    delay(2000);
    calibrateGyroBias();
    resetDeadReckoningPath();
    running = !pendingModeAdvance;
  } else if (currentMode == MODE_DEAD_RECKONING) {
    calibrateGyroBias();
    resetDeadReckoningPath();
    running = true;
  } else if (currentMode == MODE_OBSTACLE) {
    calibrateGyroBias();
    direction = 1;
    running = true;
  } else if (currentMode == MODE_REVIVAL) {
    resetRevivalMode();
    running = true;
  }
}

void advanceMode() {
  RobotMode next = (RobotMode)(((int)currentMode + 1) % MODE_COUNT);
  startMode(next);
}

void printStatus() {
  Serial.println();
  Serial.print("state=");
  Serial.print(stateName(chainState));
  Serial.print(" approachTurn=");
  Serial.print(chainApproachTurnIndex);
  Serial.print("/4");
  Serial.print(" running=");
  Serial.print(running ? "true" : "false");
  Serial.print(" lineBase=");
  Serial.print(lineBaseSpeed);
  Serial.print(" lineGain=");
  Serial.print(lineTurnGain, 2);
  Serial.print(" branchSpeed=");
  Serial.print(branchSearchSpeed);
  Serial.print(" rawThreshold=");
  Serial.print(rawLineThreshold);
  Serial.print(" calibrated=");
  Serial.print(rawCalibrated ? "true" : "false");
  Serial.print(" gyroRightScale=");
  Serial.print(gyroTurnScaleRight, 2);
  Serial.print(" flipLine=");
  Serial.println(flipLineCorrection ? "true" : "false");

  Serial.print("tunnelBase=");
  Serial.print(tunnelBaseSpeed);
  Serial.print(" KpUp=");
  Serial.print(wallKpUp, 1);
  Serial.print(" KpDown=");
  Serial.print(wallKpDown, 1);
  Serial.print(" Kd=");
  Serial.print(wallKd, 1);
  Serial.print(" wallDist=");
  Serial.print(tunnelWallDist);
  Serial.print(" doorCm=");
  Serial.print(doorDistanceCm);
  Serial.print(" doorOpenCm=");
  Serial.print(doorOpenCm);
  Serial.print(" downhillReduction=");
  Serial.println(downhillReduction);

  Serial.print("front=");
  Serial.print(readDistance(TRIG_F, ECHO_F));
  Serial.print(" left=");
  Serial.print(readDistance(TRIG_L, ECHO_L));
  Serial.print(" right=");
  Serial.print(readDistance(TRIG_R, ECHO_R));
  Serial.print(" tilt=");
  Serial.println(getTilt(), 1);
}

void printHelp() {
  Serial.println();
  Serial.println("Button mode rewrite");
  Serial.println("Commands:");
  Serial.println("  pin49  advance/start next mode");
  Serial.println("  g      run current mode");
  Serial.println("  s      stop motors");
  Serial.println("  x      reset to APPROACH");
  Serial.println("  a      manual override: airlock accepted -> TUNNEL");
  Serial.println("  1      force APPROACH");
  Serial.println("  2      force WAIT_ENTRY");
  Serial.println("  3      force TUNNEL");
  Serial.println("  4      force ARENA");
  Serial.println("  + / -  tunnel base speed up/down");
  Serial.println("  > / <  line base speed up/down");
  Serial.println("  ] / [  wall Kp up/down");
  Serial.println("  } / {  wall Kd up/down");
  Serial.println("  u / j  tunnel wall distance up/down");
  Serial.println("  d / e  door distance up/down");
  Serial.println("  f      flip line correction direction");
  Serial.println("  p      toggle telemetry print");
  Serial.println("  k      auto-calibrate raw line sensors by sweeping left/right");
  Serial.println("  b      re-zero gyro yaw bias");
  Serial.println("  m / n  gyro right turn scale up/down");
  Serial.println("  y / h  raw line threshold up/down, used before calibration");
  Serial.println("  c      print status/distances/tilt");
  Serial.println("  ?      print help");
  Serial.println("RFID C2834BF4 sends type=openAirlock to server automatically.");
  printStatus();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'g' || c == 'G') {
      running = true;
      Serial.print("[RUN] ");
      Serial.println(stateName(chainState));
    } else if (c == 's' || c == 'S') {
      running = false;
      stopMotors();
      Serial.println("[STOP] Motors stopped.");
    } else if (c == 'x' || c == 'X') {
      resetChain();
    } else if (c == 'a' || c == 'A') {
      chainState = CH_TUNNEL;
      running = true;
  wasInTunnel = false;
  exitingThroughDoor = false;
  airlockAccepted = false;
  airlockDenied = false;
  airlockRequestSent = false;
  approachReadyForRamp = false;
  doorOpenHits = 0;
  wfLastError = 0.0f;
      Serial.println("[CHAIN] Manual airlock accept -> TUNNEL.");
    } else if (c == '1') {
      chainState = CH_APPROACH;
      chainApproachTurnIndex = 0;
      Serial.println("[STATE] APPROACH");
    } else if (c == '2') {
      chainState = CH_WAIT_ENTRY;
      stopMotors();
      Serial.println("[STATE] WAIT_ENTRY");
    } else if (c == '3') {
      chainState = CH_TUNNEL;
      Serial.println("[STATE] TUNNEL");
    } else if (c == '4') {
      chainState = CH_ARENA;
      Serial.println("[STATE] ARENA");
    } else if (c == '+') {
      tunnelBaseSpeed = constrain(tunnelBaseSpeed + 25, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
      printStatus();
    } else if (c == '-') {
      tunnelBaseSpeed = constrain(tunnelBaseSpeed - 25, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
      printStatus();
    } else if (c == '>') {
      lineBaseSpeed = constrain(lineBaseSpeed + 25, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
      printStatus();
    } else if (c == '<') {
      lineBaseSpeed = constrain(lineBaseSpeed - 25, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
      printStatus();
    } else if (c == ']') {
      wallKpUp += 5.0f;
      wallKpDown += 2.0f;
      printStatus();
    } else if (c == '[') {
      wallKpUp = max(5.0f, wallKpUp - 5.0f);
      wallKpDown = max(2.0f, wallKpDown - 2.0f);
      printStatus();
    } else if (c == '}') {
      wallKd += 1.0f;
      printStatus();
    } else if (c == '{') {
      wallKd = max(0.0f, wallKd - 1.0f);
      printStatus();
    } else if (c == 'u' || c == 'U') {
      tunnelWallDist = constrain(tunnelWallDist + 1, 5, 80);
      printStatus();
    } else if (c == 'j' || c == 'J') {
      tunnelWallDist = constrain(tunnelWallDist - 1, 5, 80);
      printStatus();
    } else if (c == 'd' || c == 'D') {
      doorDistanceCm = constrain(doorDistanceCm + 1, 3, 40);
      printStatus();
    } else if (c == 'e' || c == 'E') {
      doorDistanceCm = constrain(doorDistanceCm - 1, 3, 40);
      printStatus();
    } else if (c == 'f' || c == 'F') {
      flipLineCorrection = !flipLineCorrection;
      printStatus();
    } else if (c == 'p' || c == 'P') {
      printTelemetry = !printTelemetry;
      Serial.print("printTelemetry=");
      Serial.println(printTelemetry ? "true" : "false");
    } else if (c == 'k' || c == 'K') {
      running = false;
      stopMotors();
      autoCalibrateRawLine();
      calibrateGyroBias();
    } else if (c == 'b' || c == 'B') {
      running = false;
      calibrateGyroBias();
    } else if (c == 'm' || c == 'M') {
      gyroTurnScaleRight = constrain(gyroTurnScaleRight + 0.03f, 0.75f, 1.25f);
      printStatus();
    } else if (c == 'n' || c == 'N') {
      gyroTurnScaleRight = constrain(gyroTurnScaleRight - 0.03f, 0.75f, 1.25f);
      printStatus();
    } else if (c == 'y' || c == 'Y') {
      rawLineThreshold = constrain(rawLineThreshold + 50, 50, RAW_SENSOR_TIMEOUT_US);
      printStatus();
    } else if (c == 'h' || c == 'H') {
      rawLineThreshold = constrain(rawLineThreshold - 50, 50, RAW_SENSOR_TIMEOUT_US);
      printStatus();
    } else if (c == 'c' || c == 'C') {
      printStatus();
    } else if (c == '?') {
      printHelp();
    }
  }
}

void setup() {
  Serial.begin(115200);

  messenger.onMessage(onCommsMessage);
  messenger.begin(ssid, pass, broker, port, group, board);
  Serial.println("[COMMS] MiniMessenger starting");

  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(bumper1Pin, INPUT_PULLUP);
  pinMode(bumper2Pin, INPUT_PULLUP);
  pinMode(modeButtonPin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  updateLEDs();

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  Wire.begin();
  Wire1.begin();
  RFID_WIRE.begin();

  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  stopMotors();

  byte rfidAddress = findRfidAddress();
  if (rfidAddress != 0) {
    rfid = new MFRC522_I2C(rfidAddress, RFID_RESET_PIN, &RFID_WIRE);
    rfid->PCD_Init();
    Serial.print("[RFID] Reader initialized at 0x");
    if (rfidAddress < 0x10) Serial.print("0");
    Serial.println(rfidAddress, HEX);
  }

  if (mpu.begin()) {
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[MPU] MPU6050 ready.");
  } else {
    Serial.println("[MPU] MPU6050 not found; tunnel can still use wall distances.");
  }

  resetRawCalibration();
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), countEncoder2, RISING);

  stopMotors();
  Serial.println("[MODE] Ready. Press pin 49 button to start LINE_TRACKING.");
}

void loop() {
  // Pin 49 is the only mode-cycle control. It is checked before the active mode
  // so a new press can break out of long-running mode logic.
  messenger.loop();
  commsHeartbeatCheck();
  sendStatusUpdate();

  updateLEDs();

  if (pendingModeAdvance || modeButtonPressedEvent()) {
    advanceMode();
    return;
  }

  handleSerial();

  if (!enable) {
    stopMotors();
    return;
  }

  if (!running) {
    stopMotors();
    return;
  }

  switch (currentMode) {
    case MODE_LINE_TRACKING:
      followLineStep();
      break;

    case MODE_CHAIN:
      runChainStep();
      break;

    case MODE_GATE_RFID_RAMP:
      runGateRfidRampStep();
      break;

    case MODE_ARENA_RFID_ROUTE:
    case MODE_DEAD_RECKONING:
      runDeadReckoningPath();
      break;

    case MODE_OBSTACLE:
      runObstacleStep();
      break;

    case MODE_REVIVAL:
      runRevivalStep();
      break;

    default:
      stopMotors();
      running = false;
      break;
  }
}
