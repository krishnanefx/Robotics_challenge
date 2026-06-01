// Game mode hard.
// Standalone competition sketch: chain/ramp entry, arena RFID fertility checks,
// seed planting, emergency return, and revival support. It deliberately avoids
// QTRSensors and uses raw RC IR reads so it can run without the laptop.
//
// Safety priority:
//   1. Pin 49 kill switch: immediate motor stop, no automatic resume.
//   2. Server emergency: abandon current work and route to top tunnel node.
//   3. Server disable/heartbeat timeout: stop and hold.
//   4. Distress/revival request.
//   5. Arena-only obstacle avoidance.
//   6. Normal planting game.

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
  MODE_GAME = 0,
  MODE_COUNT = 1
};

enum GameState {
  GAME_CHAIN_ENTRY,
  GAME_INITIAL_PATTERN,
  GAME_SERPENTINE,
  GAME_EMERGENCY_EXIT,
  GAME_TOP_TUNNEL_STOP,
  GAME_DONE
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
const bool HARD_MODE_OBSTACLES = true;
const int ARENA_SIZE = 9;
const int MAX_SEEDS = 5;
const int MAX_TAG_CACHE = 40;
const int SEED_MOTOR_CHANNEL = 1;
const int SEED_SPEED = 150;
const unsigned long SEED_DURATION_MS = 550;
const unsigned long FERTILITY_REPLY_TIMEOUT_MS = 900;
const uint8_t FERTILITY_RETRIES = 2;

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
RobotMode currentMode = MODE_GAME;
GameState gameState = GAME_CHAIN_ENTRY;
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
bool killLatched = false;
bool emergencyExitRequested = false;
bool gameStarted = false;
bool initialPatternDone = false;
uint8_t initialPatternStep = 0;
int gameX = 0;
int gameY = 0;
int verticalDir = 1;
int seedsPlanted = 0;
String seenTags[MAX_TAG_CACHE];
String plantedTags[MAX_TAG_CACHE];
uint8_t seenTagCount = 0;
uint8_t plantedTagCount = 0;
bool fertilityReplyPending = false;
bool fertilityReplyReceived = false;
bool fertilityReplyFertile = false;
bool fertilityReplyPlanted = false;
String fertilityReplyTag = "";
String requestedFertilityTag = "";
bool revivalTargetAvailable = false;
char revivalTargetTeam[8] = "";
char revivalTargetBoard[16] = "";

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

bool killSwitchActive() {
  if (digitalRead(modeButtonPin) == LOW) {
    killLatched = true;
    running = false;
    stopMotors();
    return true;
  }
  return killLatched;
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

void clearRevivalTarget() {
  revivalTargetAvailable = false;
  revivalTargetTeam[0] = '\0';
  revivalTargetBoard[0] = '\0';
}

void copyTokenRange(char* dest, size_t destSize, const char* start, const char* end) {
  if (destSize == 0) return;
  size_t len = (size_t)(end - start);
  if (len >= destSize) len = destSize - 1;
  memcpy(dest, start, len);
  dest[len] = '\0';
}

void parseDistressTarget(const char* msg) {
  if (strstr(msg, "count=0")) {
    clearRevivalTarget();
    return;
  }

  const char* robot = strstr(msg, "robot0=");
  if (!robot) return;
  robot += 7;
  const char* dot = strchr(robot, '.');
  const char* comma = strchr(robot, ',');
  if (!dot || !comma || dot >= comma || dot == robot) return;

  copyTokenRange(revivalTargetTeam, sizeof(revivalTargetTeam), robot, dot);
  copyTokenRange(revivalTargetBoard, sizeof(revivalTargetBoard), dot + 1, comma);
  revivalTargetAvailable = true;
  revivalDone = false;
  Serial.print("[REVIVE] Target team=");
  Serial.print(revivalTargetTeam);
  Serial.print(" board=");
  Serial.println(revivalTargetBoard);
}

void parseFertilityReply(const char* msg) {
  fertilityReplyReceived = true;
  fertilityReplyFertile = strstr(msg, "fertile=true") != nullptr;
  fertilityReplyPlanted = strstr(msg, "planted=true") != nullptr;
  fertilityReplyTag = requestedFertilityTag;
  Serial.print("[FERTILITY] Reply fertile=");
  Serial.print(fertilityReplyFertile ? "true" : "false");
  Serial.print(" planted=");
  Serial.println(fertilityReplyPlanted ? "true" : "false");
}

void requestEmergencyExit() {
  if (killSwitchActive()) return;
  emergencyExitRequested = true;
  gameState = GAME_EMERGENCY_EXIT;
  running = true;
  stopMotors();
  Serial.println("[EMERGENCY] Routing to top tunnel node.");
}

void onCommsMessage(const MessageMetadata& meta,
                    const uint8_t* payload,
                    size_t length) {
  if (length == 6) {
    if (payload[4]) {
      Serial.println("[COMMS] Emergency flag");
      requestEmergencyExit();
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

  if (strstr(msg, "type=distress")) {
    parseDistressTarget(msg);
    return;
  }

  if (strstr(msg, "type=isFertileReply")) {
    parseFertilityReply(msg);
    return;
  }

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

  if (strstr(msg, "type=emergency")) {
    requestEmergencyExit();
    return;
  }

  if (strstr(msg, "type=disable")) {
    enable = 0;
    running = false;
    stopMotors();
    Serial.println("[COMMS] Disable — stopped");
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
             "STATUS:GAME_MODE state=%s enable=%d running=%d",
             gameStateName(gameState), enable, running ? 1 : 0);
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
  if (gameState != GAME_CHAIN_ENTRY) return false;
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
  if (killSwitchActive()) {
    running = false;
    stopMotors();
    Serial.println("[KILL] Pin 49 active.");
    return true;
  }

  if (emergencyExitRequested && gameState != GAME_EMERGENCY_EXIT) {
    stopMotors();
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
    if (killSwitchActive()) {
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
    messenger.loop();
    if (serialStopRequested()) return false;
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
        gameState = GAME_INITIAL_PATTERN;
        gameX = 0;
        gameY = 0;
        direction = 1;
        initialPatternStep = 0;
        initialPatternDone = false;
        Serial.println("[CHAIN] Tunnel cleared. Entering arena game path.");
        return;
      }
      setDrive(tunnelBaseSpeed, tunnelBaseSpeed);
      return;

    case CH_ARENA:
      gameState = GAME_INITIAL_PATTERN;
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

bool driveToNodeCountsOnly() {
  zeroDriveState();
  while (true) {
    updateLEDs();
    messenger.loop();
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

    int correction = constrain((int)(DRIVE_KP * fusedHeading), -189, 189);
    int leftSpeed = constrain(DRIVE_SPEED + correction, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
    int rightSpeed = constrain(DRIVE_SPEED - correction, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
    setDrive(leftSpeed, rightSpeed);
  }

  stopMotors();
  calibrateGyroBias();
  return true;
}

bool stringInCache(String cache[], uint8_t count, const String& value) {
  for (uint8_t i = 0; i < count; i++) {
    if (cache[i] == value) return true;
  }
  return false;
}

void addToCache(String cache[], uint8_t& count, const String& value) {
  if (stringInCache(cache, count, value)) return;
  if (count >= MAX_TAG_CACHE) return;
  cache[count++] = value;
}

void dropSeed() {
  Serial.println("[SEED] Dropping one seed.");
  mc2.setSpeed(SEED_MOTOR_CHANNEL, SEED_SPEED);
  delay(SEED_DURATION_MS);
  mc2.setSpeed(SEED_MOTOR_CHANNEL, 0);
  Serial.println("[SEED] Done.");
}

void sendSeedPlanted(const String& tagId) {
  char msg[96];
  snprintf(msg, sizeof(msg),
           "type=seedPlanted tag_id=%s board_id=%s",
           tagId.c_str(), board);
  messenger.sendToBoard("server", msg);
  Serial.print("[SEED] seedPlanted sent for ");
  Serial.println(tagId);
}

void sendReviveRequest(const char* targetTeam, const char* targetBoard) {
  char msg[80];
  snprintf(msg, sizeof(msg),
           "type=reviveRequest target_team=%s target_board=%s",
           targetTeam, targetBoard);
  messenger.sendToBoard("server", msg);
  Serial.print("[REVIVE] Request sent target=");
  Serial.print(targetTeam);
  Serial.print(".");
  Serial.println(targetBoard);
}

bool queryFertility(const String& tagId) {
  for (uint8_t attempt = 0; attempt < FERTILITY_RETRIES; attempt++) {
    requestedFertilityTag = tagId;
    fertilityReplyReceived = false;
    fertilityReplyPending = true;

    char msg[96];
    snprintf(msg, sizeof(msg),
             "type=isFertile tag_id=%s board_id=%s",
             tagId.c_str(), board);
    messenger.sendToBoard("server", msg);
    Serial.print("[FERTILITY] Request ");
    Serial.print(attempt + 1);
    Serial.print(" tag=");
    Serial.println(tagId);

    unsigned long start = millis();
    while (millis() - start < FERTILITY_REPLY_TIMEOUT_MS) {
      messenger.loop();
      updateLEDs();
      if (serialStopRequested()) {
        fertilityReplyPending = false;
        return false;
      }
      if (fertilityReplyReceived) {
        fertilityReplyPending = false;
        return fertilityReplyFertile && !fertilityReplyPlanted;
      }
      delay(10);
    }
  }

  fertilityReplyPending = false;
  Serial.println("[FERTILITY] No reply; skipping node.");
  return false;
}

void checkCurrentNodeForSeed() {
  if (seedsPlanted >= MAX_SEEDS) return;

  String tagId;
  if (!readRfidTag(tagId)) return;

  Serial.print("[RFID] Arena node tag=");
  Serial.println(tagId);
  if (stringInCache(plantedTags, plantedTagCount, tagId)) return;
  if (stringInCache(seenTags, seenTagCount, tagId)) return;

  addToCache(seenTags, seenTagCount, tagId);
  if (!queryFertility(tagId)) return;

  dropSeed();
  sendSeedPlanted(tagId);
  addToCache(plantedTags, plantedTagCount, tagId);
  seedsPlanted++;
  Serial.print("[GAME] Seeds planted=");
  Serial.println(seedsPlanted);
}

bool moveGameNode(int targetDir, bool allowObstacle) {
  if (allowObstacle && HARD_MODE_OBSTACLES && readDistance(TRIG_F, ECHO_F) <= OBSTACLE_DETECT_DIST) {
    if (!avoidObstacle()) return false;
  }

  drFaceDir(targetDir);
  if (serialStopRequested()) return false;
  if (!driveToNodeCountsOnly()) return false;

  if (targetDir == 0) gameX--;
  else if (targetDir == 1) gameY++;
  else if (targetDir == 2) gameX++;
  else if (targetDir == 3) gameY--;
  gameX = constrain(gameX, 0, ARENA_SIZE - 1);
  gameY = constrain(gameY, 0, ARENA_SIZE - 1);

  Serial.print("[GAME] Node x=");
  Serial.print(gameX);
  Serial.print(" y=");
  Serial.println(gameY);
  checkCurrentNodeForSeed();
  return true;
}

bool runInitialPatternStep() {
  switch (initialPatternStep) {
    case 0:
      if (!moveGameNode(1, true)) return false;
      break;
    case 1:
      if (!moveGameNode(2, true)) return false;
      break;
    case 2:
      if (!moveGameNode(1, true)) return false;
      break;
    case 3:
      if (!moveGameNode(1, true)) return false;
      break;
    case 4:
      drTurnLeft();
      verticalDir = 1;
      initialPatternDone = true;
      gameState = GAME_SERPENTINE;
      Serial.println("[GAME] Initial pattern complete.");
      return true;
    default:
      initialPatternDone = true;
      gameState = GAME_SERPENTINE;
      return true;
  }
  initialPatternStep++;
  return true;
}

bool runSerpentineStep() {
  if (seedsPlanted >= MAX_SEEDS) {
    gameState = GAME_EMERGENCY_EXIT;
    return true;
  }

  if (verticalDir > 0 && gameY < ARENA_SIZE - 1) return moveGameNode(1, true);
  if (verticalDir < 0 && gameY > 0) return moveGameNode(3, true);

  if (gameX < ARENA_SIZE - 1) {
    if (!moveGameNode(2, true)) return false;
    verticalDir = -verticalDir;
    return true;
  }

  Serial.println("[GAME] Sweep complete before 5 seeds; going to top tunnel.");
  gameState = GAME_EMERGENCY_EXIT;
  return true;
}

bool routeToTopTunnelNode() {
  while (gameY < ARENA_SIZE - 1) {
    if (!moveGameNode(1, false)) return false;
  }
  while (gameX > 0) {
    if (!moveGameNode(0, false)) return false;
  }
  drFaceDir(0);
  stopMotors();
  return true;
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
    if (killSwitchActive()) {
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
    sendReviveRequest(revivalTargetTeam, revivalTargetBoard);
    clearRevivalTarget();
    revivalDone = true;
    running = true;
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
  return "GAME";
}

void startMode(RobotMode mode) {
  stopMotors();
  currentMode = MODE_GAME;
  gameState = GAME_CHAIN_ENTRY;
  emergencyExitRequested = false;
  pendingModeAdvance = false;
  calibrationLedOverride = false;
  updateLEDs();
  resetChain();
  autoCalibrateRawLine();
  calibrateGyroBias();
  running = !killSwitchActive();
  gameStarted = running;
  Serial.println("[GAME] Started.");
}

void advanceMode() {
  startMode(MODE_GAME);
}

const char* gameStateName(GameState state) {
  switch (state) {
    case GAME_CHAIN_ENTRY: return "CHAIN_ENTRY";
    case GAME_INITIAL_PATTERN: return "INITIAL_PATTERN";
    case GAME_SERPENTINE: return "SERPENTINE";
    case GAME_EMERGENCY_EXIT: return "EMERGENCY_EXIT";
    case GAME_TOP_TUNNEL_STOP: return "TOP_TUNNEL_STOP";
    case GAME_DONE: return "DONE";
  }
  return "?";
}

void printStatus() {
  Serial.println();
  Serial.print("gameState=");
  Serial.print(gameStateName(gameState));
  Serial.print(" chainState=");
  Serial.print(stateName(chainState));
  Serial.print(" running=");
  Serial.print(running ? "true" : "false");
  Serial.print(" killLatched=");
  Serial.print(killLatched ? "true" : "false");
  Serial.print(" emergencyExit=");
  Serial.print(emergencyExitRequested ? "true" : "false");
  Serial.print(" seeds=");
  Serial.print(seedsPlanted);
  Serial.print("/");
  Serial.print(MAX_SEEDS);
  Serial.print(" node=(");
  Serial.print(gameX);
  Serial.print(",");
  Serial.print(gameY);
  Serial.print(") facing=");
  Serial.println(dirNames[direction]);

  Serial.print("rawLine calibrated=");
  Serial.print(rawCalibrated ? "true" : "false");
  Serial.print(" threshold=");
  Serial.print(rawLineThreshold);
  Serial.print(" lineBase=");
  Serial.print(lineBaseSpeed);
  Serial.print(" lineGain=");
  Serial.println(lineTurnGain, 2);

  Serial.print("ramp base=");
  Serial.print(tunnelBaseSpeed);
  Serial.print(" KpUp=");
  Serial.print(wallKpUp, 1);
  Serial.print(" KpDown=");
  Serial.print(wallKpDown, 1);
  Serial.print(" Kd=");
  Serial.print(wallKd, 1);
  Serial.print(" doorOpenCm=");
  Serial.println(doorOpenCm);

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
  Serial.println(HARD_MODE_OBSTACLES ? "Game mode hard" : "Game mode");
  Serial.println("Commands:");
  Serial.println("  g      start/resume game if not killed");
  Serial.println("  s      stop motors and pause game");
  Serial.println("  e      simulate server emergency route");
  Serial.println("  k      rerun raw IR + gyro calibration while stopped");
  Serial.println("  p      toggle telemetry print");
  Serial.println("  c      print status/distances/tilt");
  Serial.println("  ?      print help");
  Serial.println("Pin 49 is the hardware kill switch and cannot be cleared by Serial.");
  printStatus();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'g' || c == 'G') {
      if (!killLatched) {
        running = true;
        Serial.println("[RUN] Game running.");
      } else {
        Serial.println("[RUN] Ignored: pin 49 kill switch is latched.");
      }
    } else if (c == 's' || c == 'S') {
      running = false;
      stopMotors();
      Serial.println("[STOP] Motors stopped.");
    } else if (c == 'e' || c == 'E') {
      requestEmergencyExit();
    } else if (c == 'k' || c == 'K') {
      running = false;
      stopMotors();
      autoCalibrateRawLine();
      calibrateGyroBias();
    } else if (c == 'p' || c == 'P') {
      printTelemetry = !printTelemetry;
      Serial.print("printTelemetry=");
      Serial.println(printTelemetry ? "true" : "false");
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
  Serial.println("[GAME] Auto-starting. Pin 49 is kill switch.");
  startMode(MODE_GAME);
}

void loop() {
  messenger.loop();
  commsHeartbeatCheck();
  sendStatusUpdate();

  updateLEDs();

  if (killSwitchActive()) {
    stopMotors();
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

  if (revivalTargetAvailable && !emergencyExitRequested && gameState != GAME_CHAIN_ENTRY) {
    runRevivalStep();
    return;
  }

  switch (gameState) {
    case GAME_CHAIN_ENTRY:
      runChainStep();
      break;

    case GAME_INITIAL_PATTERN:
      runInitialPatternStep();
      break;

    case GAME_SERPENTINE:
      runSerpentineStep();
      break;

    case GAME_EMERGENCY_EXIT:
      if (routeToTopTunnelNode()) {
        gameState = GAME_TOP_TUNNEL_STOP;
      }
      break;

    case GAME_TOP_TUNNEL_STOP:
    case GAME_DONE:
      stopMotors();
      running = false;
      Serial.println("[GAME] Stopped at top tunnel node.");
      break;

    default:
      stopMotors();
      running = false;
      break;
  }
}
