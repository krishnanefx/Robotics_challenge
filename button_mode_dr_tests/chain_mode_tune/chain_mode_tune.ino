// Chain mode full-path tuner.
// Runs the base-side line approach with raw IR sensors, reads the airlock RFID
// during that approach, requests the door through MiniMessenger, then enters the
// ramp only after final line loss, server acceptance, and ultrasonic clearance.
// No QTR library is used. Tune rawLineThreshold/lineBaseSpeed/lineTurnGain for
// the approach and tunnelBaseSpeed/wallKp*/wallKd for ramp wall-following.

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
const int redPin = 46;
const int greenPin = 47;

const String AIRLOCK_A_TAG_ID = "C2834BF4";

enum ChainState {
  CH_APPROACH,
  CH_WAIT_ENTRY,
  CH_TUNNEL,
  CH_ARENA
};

ChainState chainState = CH_APPROACH;
int chainApproachTurnIndex = 0;

const int MAX_MOTOR_SPEED = 660;
const int MIN_FORWARD_SPEED = 200;
const uint16_t RAW_SENSOR_TIMEOUT_US = 2500;
const unsigned long LINE_END_FORWARD_MS = 0;
const unsigned long AFTER_LINE_END_FORWARD_MS = 50;
const int LEFT_TURN_SLOW_SPEED = 500;

int lineBaseSpeed = 300;
float lineTurnGain = 1.0f;
uint16_t rawLineThreshold = 200;
int branchSearchSpeed = 200;
bool flipLineCorrection = false;
bool rawCalibrated = false;

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
float gyroTurnScaleRight = 0.94f;
float gyroTurnScaleLeft = 1.0f;
unsigned long lastTiltUs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastRegisterMs = 0;
unsigned long lastStatusMs = 0;
unsigned long airlockWaitStartMs = 0;
int doorOpenHits = 0;

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
  bool contact = digitalRead(bumper1Pin) == LOW || digitalRead(bumper2Pin) == LOW;
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
             "STATUS:CHAIN_TUNE state=%s enable=%d running=%d",
             stateName(chainState), enable, running ? 1 : 0);
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
  Serial.println("[RAW] Auto calibration starting.");
  Serial.println("[RAW] Put the sensor bar over the line area. Robot will sweep left/right.");
  resetRawCalibration();
  delay(200);

  const int calSpeed = 500;
  const int sampleDelayMs = 20;

  for (uint16_t i = 0; i < 10; i++) {
    spinRight(calSpeed);
    sampleRawCalibration();
    delay(sampleDelayMs);
  }
  for (uint16_t i = 0; i < 20; i++) {
    spinLeft(calSpeed);
    sampleRawCalibration();
    delay(sampleDelayMs);
  }
  for (uint16_t i = 0; i < 20; i++) {
    spinRight(calSpeed);
    sampleRawCalibration();
    delay(sampleDelayMs);
  }
  for (uint16_t i = 0; i < 10; i++) {
    spinLeft(calSpeed);
    sampleRawCalibration();
    delay(sampleDelayMs);
  }

  stopMotors();
  rawCalibrated = true;
  Serial.println("[RAW] Auto calibration done.");
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
  Serial.println("Chain mode standalone tune");
  Serial.println("Commands:");
  Serial.println("  g      run current chain state");
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
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  updateLEDs();

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
  autoCalibrateRawLine();
  calibrateGyroBias();

  running = true;
  Serial.println("[CHAIN] Auto-run enabled.");
}

void loop() {
  messenger.loop();
  commsHeartbeatCheck();
  sendStatusUpdate();

  updateLEDs();
  handleSerial();

  if (!enable) {
    stopMotors();
    return;
  }

  if (running) {
    runChainStep();
  } else {
    stopMotors();
  }
}
