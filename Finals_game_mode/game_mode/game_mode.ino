// Game mode.
// Standalone competition sketch: chain/ramp entry, arena RFID fertility checks,
// seed planting, emergency return, and revival support. It deliberately avoids
// QTRSensors and uses raw RC IR reads so it can run without the laptop.
//
// Safety priority:
//   1. Pin 49 or WiFi disable kill: immediate motor stop, red LED blink, no automatic resume.
//   2. Server emergency: abandon current work and route to top tunnel node.
//   3. Heartbeat timeout: stop and hold.
//   4. Distress/revival request.
//   5. Normal planting game.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <Motoron.h>
#include <Wire.h>
#include <math.h>

#define RFID_WIRE Wire2
#define RFID_RESET_PIN -1

MotoronI2C mc1(18);  // ch1 seed drop, ch2/ch3 drive
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
const String AIRLOCK_B_TAG_ID = "1B0AAB41";

enum RobotMode {
  MODE_GAME = 0,
  MODE_COUNT = 1
};

enum GameState {
  GAME_WAIT_READY,
  GAME_CHAIN_ENTRY,
  GAME_MAIN_COURSE,
  GAME_REQUEST_EXIT_B,
  GAME_WAIT_EXIT_B,
  GAME_EXIT_RAMP,
  GAME_EXIT_LINE,
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
const int MAX_MOTOR_SPEED = 760;
const int MIN_FORWARD_SPEED = 200;
const int MIN_TURN_SPEED = 400;

// Raw line-sensor calibration timing. The 2 second window is for moving the
// robot by hand over the black line and white floor while the green LED is on.
const uint16_t RAW_SENSOR_TIMEOUT_US = 2500;
const unsigned long IR_CALIBRATION_MS = 2000;
const unsigned long GYRO_PLACEMENT_PAUSE_MS = 3000;
const unsigned long LINE_END_FORWARD_MS = 0;
const unsigned long AFTER_LINE_END_FORWARD_MS = 50;
const unsigned long RIGHT_BRANCH_LOCKOUT_AFTER_SECOND_LEFT_MS = 2000;
const unsigned long TURN_REACQUIRE_TIMEOUT_MS = 2500;
const unsigned long CHAIN_LINE_END_TIMEOUT_MS = 10000;
const unsigned long CHAIN_BRANCH_TIMEOUT_MS = 12000;
const int LEFT_TURN_SLOW_SPEED = 600;

// Dead-reckoning tuning copied from dead_reckoning_test.ino. COUNTS_PER_NODE is
// derived from the wheel geometry but can be replaced with the measured value if
// encoder_counts_per_node.ino shows the robot lands short or long.
const float WHEEL_DIAM_CM = 6.5f;
const float TRACK_WIDTH_CM = 17.0f;
const float COUNTS_PER_REV = 144.0f;
const float CM_PER_COUNT = (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;
const int COUNTS_PER_NODE = (int)(25.0f / CM_PER_COUNT);
const int DRIVE_SPEED = 440;
const float DRIVE_KP = 80.0f;
// Positive trim counters a right drift by driving the left side slightly faster
// and the right side slightly slower during dead-reckoning node moves.
int driveTrim = 18;
const int TURN_FAST = 760;
const int TURN_SLOW = 620;
const float TURN_SCALE_LEFT = 0.96f;
const float TURN_SCALE_RIGHT = 0.90f;
const float HEADING_ALPHA = 0.98f;

// Main course after the entry ramp:
// 9 nodes, left, 2 nodes up the incline, then 4 more nodes and stop.
// Directions use 0=W, 1=N, 2=E, 3=S.
const int8_t COURSE_TURN_NONE = 0;
const int8_t COURSE_TURN_LEFT = 1;
const int8_t COURSE_TURN_RIGHT = -1;
const uint8_t COURSE_SEGMENT_COUNT = 3;
const uint8_t courseSegmentNodes[COURSE_SEGMENT_COUNT] = {9, 2, 4};
const int8_t courseTurnAfterSegment[COURSE_SEGMENT_COUNT] = {
  COURSE_TURN_LEFT,
  COURSE_TURN_NONE,
  COURSE_TURN_NONE
};
const uint8_t INCLINE_SEGMENT_INDEX = 1;
const uint8_t LINE_TRACK_NODE_COUNT = 4;
const int FIRST_NODE_SPEED = 330;
const int INCLINE_COURSE_SPEED = 520;
const int NODE_COUNT = 6;
int pathx[NODE_COUNT] = {0, 0, 0, 1, 1, 1};
int pathy[NODE_COUNT] = {0, 1, 2, 2, 3, 4};
char dirNames[4] = {'W', 'N', 'E', 'S'};

const int OBSTACLE_DETECT_DIST = 25;
const unsigned long AVOID_TIMEOUT_MS = 60000UL;
const bool HARD_MODE_OBSTACLES = false;
const int ARENA_SIZE = 9;
const int MAX_SEEDS = 5;
const uint8_t MAX_SEEDS_PER_NODE = 1;
const int MAX_TAG_CACHE = 40;
const int SEED_MOTOR_CHANNEL = 1;
const int SEED_SPEED = 150;
const unsigned long SEED_DURATION_MS = 650;
const unsigned long FERTILITY_REPLY_TIMEOUT_MS = 900;
const uint8_t FERTILITY_RETRIES = 2;
const unsigned long NODE_RFID_WAIT_MS = 1500;
const unsigned long NODE_RFID_LOCKOUT_MS = 700;
const unsigned long NODE_LINE_TIMEOUT_MS = 12000;
const unsigned long ARENA_PRE_TURN_FORWARD_MS = 300;
const unsigned long ARENA_LINE_SEARCH_TIMEOUT_MS = 2500;
const unsigned long ARENA_LINE_SEARCH_SEGMENT_MS = 250;
const int ARENA_LINE_SEARCH_SPEED = 320;

// Raw line-follow tuning from line_tracking_tune.ino. If the robot weaves,
// reduce lineTurnGain; if it does not correct enough, increase it.
int lineBaseSpeed = 440;
float lineTurnGain = 1.6f;
float lineKd = 0.7f;
int lineCorrectionCap = 220;
uint16_t rawLineThreshold = 200;
int branchSearchSpeed = 200;
bool flipLineCorrection = false;
bool rawCalibrated = false;
const uint8_t MIN_LINE_ACTIVE_SENSORS = 2;
const uint8_t LINE_LOSS_CONFIRM_READS = 5;
const uint8_t RIGHT_BRANCH_ACTIVE_SENSORS = 3;
const uint8_t RIGHT_BRANCH_CONFIRM_READS = 8;
const int LINE_CENTER_BASE_SPEED = 440;
const int LINE_MODERATE_BASE_SPEED = 350;
const int LINE_LARGE_BASE_SPEED = 260;
const int LINE_MIN_FORWARD_SPEED = 160;
const int LINE_HARD_TURN_MIN_SPEED = 80;
const float LINE_MODERATE_ERROR = 45.0f;
const float LINE_LARGE_ERROR = 90.0f;
const unsigned long LINE_RECOVERY_MS = 350;

// Ramp/wall-follow tuning from the latest gate/ramp tests. Before tunnel
// detection, the robot drives straight/line-tracks into the ramp; wall PID
// starts only after tilt or close side walls prove it is inside the tunnel.
int tunnelBaseSpeed = 400;
int rampEntryBoostSpeed = 600;
float wallKpUp = -80.0f;
float wallKpDown = 10.0f;
float wallKd = 10.0f;
int tunnelWallDist = 20;
int wallCorrectionCap = 220;
int doorDistanceCm = 8;
int doorOpenCm = 7;
unsigned long minRampRunMs = 5000;
int rampExitClearHitsRequired = 6;
unsigned long rampExitForwardMs = 500;
int downhillReduction = 300;
int downhillRampSpeed = 180;
int rampDoorEntrySpeed = 220;
unsigned long rampDoorStraightMs = 700;
float tiltThreshold = 5.0f;
float downhillThreshold = -20.0f;
float compAlpha = 0.90f;

bool running = false;
volatile int enable = 1;
RobotMode currentMode = MODE_GAME;
GameState gameState = GAME_WAIT_READY;
GateState gateState = GATE_SEND_REQUEST;
bool airlockAccepted = false;
bool airlockDenied = false;
bool airlockRequestSent = false;
bool approachReadyForRamp = false;
bool printTelemetry = false;
bool mpuOk = false;
bool wasInTunnel = false;
int rampExitClearHits = 0;
unsigned long rampStartMs = 0;
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
unsigned long lastLineSeenMs = 0;
unsigned long lastLineUpdateMs = 0;
int doorOpenHits = 0;
float lastLineError = 0.0f;
int8_t lastLineSeenSide = 0;
int direction = 1;
int activeDriveSpeed = DRIVE_SPEED;
int drIndex = 1;
bool drDone = false;
bool modeButtonWasDown = false;
bool pendingModeAdvance = false;
bool calibrationLedOverride = false;
bool commsStarted = false;
bool serverReadyIdle = false;
bool runKillArmed = false;
bool killLatched = false;
bool wifiKillLatched = false;
bool resumeAfterKill = false;
bool emergencyExitRequested = false;
bool gamePrepared = false;
uint8_t courseSegmentIndex = 0;
uint8_t courseNodesCompleted = 0;
bool exitBRequestSent = false;
String finalNodeTag = "";
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
bool obstacleWaitingForRfid = false;
String lastAcceptedRfidTag = "";

int approachDistanceCm = 40;
int approachSpeed = 300;
int crawlSpeed = 150;
const unsigned long REVIVAL_BUMP_MS = 1500;
const unsigned long REVIVAL_REVERSE_MS = 500;
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
  digitalWrite(redPin, calibrating ? HIGH : LOW);
  digitalWrite(greenPin, calibrating ? HIGH : LOW);
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

bool killSwitchLatched() {
  return killLatched || wifiKillLatched;
}

void clearRobotKill(const char* source) {
  bool wasDisabled = killLatched || wifiKillLatched || !enable;
  if (!wasDisabled) return;

  killLatched = false;
  wifiKillLatched = false;
  enable = 1;
  running = resumeAfterKill;
  resumeAfterKill = false;
  runKillArmed = false;
  stopMotors();
  Serial.print("[ENABLE] Robot enabled from ");
  Serial.println(source);
  Serial.println(running ? "[ENABLE] Resuming previous run state." : "[ENABLE] Ready/idle.");
}

// Pin 49 starts the game while ready/idle. After the run starts and the button
// has been released once, the next press latches an emergency stop. If either
// WiFi or pin 49 has latched a stop, a fresh pin 49 press clears both latches.
bool pollRunKillSwitch() {
  if (wifiKillLatched || killLatched) {
    if (modeButtonPressedEvent()) {
      clearRobotKill("pin 49");
      return false;
    }
    return true;
  }
  if (!running) return false;

  if (!runKillArmed) {
    if (digitalRead(modeButtonPin) == HIGH) runKillArmed = true;
    return false;
  }

  if (digitalRead(modeButtonPin) == LOW) {
    killLatched = true;
    resumeAfterKill = running;
    modeButtonWasDown = true;
    running = false;
    stopMotors();
    Serial.println("[KILL] Pin 49 kill latched.");
    return true;
  }
  return false;
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
  if (killSwitchLatched()) {
    bool blinkOn = (millis() / 250) % 2 == 0;
    digitalWrite(redPin, blinkOn ? HIGH : LOW);
    digitalWrite(greenPin, LOW);
    return;
  }
  if (calibrationLedOverride) return;
  if (anyBumperPressed()) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    return;
  }
  if (running) {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    return;
  }
  if (serverReadyIdle && gamePrepared && messenger.isConnected() && enable) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    return;
  }
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
}

// MiniMessenger official disable / heartbeat enable=0 is treated as a WiFi kill switch.
void latchWifiKill(const char* source) {
  wifiKillLatched = true;
  resumeAfterKill = running;
  enable = 0;
  running = false;
  stopMotors();
  Serial.print("[KILL] WiFi kill latched from ");
  Serial.println(source);
}

void clearWifiKill(const char* source) {
  clearRobotKill(source);
}

void enableAndRequestStart(const char* source) {
  clearRobotKill(source);
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
  if (killSwitchLatched()) return;
  emergencyExitRequested = true;
  stopMotors();
  Serial.println("[EMERGENCY] Server emergency received; motors stopped.");
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
      clearWifiKill("heartbeat enable=1");
    } else if (strstr(msg, "enable=0")) {
      latchWifiKill("heartbeat enable=0");
    }
    return;
  }

  if (strstr(msg, "type=kill") ||
      strstr(msg, "type=killswitch") ||
      strstr(msg, "kill=true") ||
      strstr(msg, "kill=1")) {
    latchWifiKill("server command");
    return;
  }

  if (strstr(msg, "type=emergency")) {
    requestEmergencyExit();
    return;
  }

  if (strstr(msg, "type=disable")) {
    latchWifiKill("type=disable");
    return;
  }

  if (strstr(msg, "type=enable") ||
      strstr(msg, "type=resume") ||
      strstr(msg, "enable=true") ||
      strstr(msg, "enable=1")) {
    enableAndRequestStart("server command");
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
      resumeAfterKill = running;
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
             "STATUS:GAME_MODE mode=%s enable=%d running=%d",
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
  calibrationLedOverride = true;
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  Serial.println("[GYRO] Keep robot still. Calibrating yaw bias...");
  delay(100);

  float sum = 0.0f;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    sum += readGyroZRadPerSec();
    delay(3);
  }
  gyroBiasZ = sum / samples;
  calibrationLedOverride = false;
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

struct LineSnapshot {
  float error;
  uint8_t activeCount;
  bool leftEdge;
  bool rightEdge;
  bool onLine;
};

LineSnapshot readLineSnapshot();

float readLineError() {
  LineSnapshot line = readLineSnapshot();
  return line.error;
}

void resetLineController() {
  lastLineError = 0.0f;
  lastLineUpdateMs = 0;
  lastLineSeenMs = 0;
  lastLineSeenSide = 0;
}

LineSnapshot readLineSnapshot() {
  LineSnapshot line;
  line.error = 0.0f;
  line.activeCount = 0;
  line.leftEdge = false;
  line.rightEdge = false;
  line.onLine = false;

  readRawLineSensors();
  float error = 0.0f;
  float activeWeightedError = 0.0f;
  float activeStrength = 0.0f;

  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t value = rawCalibrated ? normalizedRawValue(i) : sensorValues[i];
    bool active = value > (rawCalibrated ? 100 : rawLineThreshold);
    error += value * weights[i];
    if (active) {
      line.activeCount++;
      activeWeightedError += value * weights[i];
      activeStrength += value;
      if (i <= 1) line.leftEdge = true;
      if (i >= 7) line.rightEdge = true;
    }
  }

  if (activeStrength > 0.0f) {
    error = activeWeightedError / activeStrength * 1000.0f;
  }

  if (line.leftEdge && !line.rightEdge) error = max(error, LINE_LARGE_ERROR);
  if (line.rightEdge && !line.leftEdge) error = min(error, -LINE_LARGE_ERROR);

  if (flipLineCorrection) error = -error;
  line.error = error;
  line.onLine = line.activeCount >= MIN_LINE_ACTIVE_SENSORS;
  return line;
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
  LineSnapshot line = readLineSnapshot();
  unsigned long now = millis();
  float error = line.error;

  if (line.onLine) {
    lastLineSeenMs = now;
    if (error > 3.0f) lastLineSeenSide = 1;
    else if (error < -3.0f) lastLineSeenSide = -1;
  } else if (lastLineSeenSide != 0 && now - lastLineSeenMs <= LINE_RECOVERY_MS) {
    error = lastLineSeenSide > 0 ? LINE_LARGE_ERROR : -LINE_LARGE_ERROR;
  } else {
    stopMotors();
    return;
  }

  float derivative = 0.0f;
  if (lastLineUpdateMs != 0) derivative = error - lastLineError;
  lastLineUpdateMs = now;
  lastLineError = error;

  float absError = fabsf(error);
  int base = LINE_CENTER_BASE_SPEED;
  if (absError >= LINE_LARGE_ERROR) base = LINE_LARGE_BASE_SPEED;
  else if (absError >= LINE_MODERATE_ERROR) base = LINE_MODERATE_BASE_SPEED;
  base = min(base, lineBaseSpeed);

  int correction = constrain((int)(lineTurnGain * error + lineKd * derivative),
                             -lineCorrectionCap, lineCorrectionCap);
  int minSpeed = absError >= LINE_LARGE_ERROR ? LINE_HARD_TURN_MIN_SPEED : LINE_MIN_FORWARD_SPEED;
  int left = constrain(base - correction, minSpeed, MAX_MOTOR_SPEED);
  int right = constrain(base + correction, minSpeed, MAX_MOTOR_SPEED);
  setDrive(left, right);

  if (printTelemetry && millis() - lastPrintMs > 250) {
    lastPrintMs = millis();
    Serial.print("[LINE] state=");
    Serial.print(stateName(chainState));
    Serial.print(" err=");
    Serial.print(error, 1);
    Serial.print(" active=");
    Serial.print(line.activeCount);
    Serial.print(" d=");
    Serial.print(derivative, 1);
    Serial.print(" base=");
    Serial.print(base);
    Serial.print(" corr=");
    Serial.print(correction);
    Serial.print(" L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.println(right);
  }
}

void followArenaLineStep() {
  LineSnapshot line = readLineSnapshot();
  unsigned long now = millis();
  float error = line.error;

  if (line.onLine) {
    lastLineSeenMs = now;
    if (error > 3.0f) lastLineSeenSide = 1;
    else if (error < -3.0f) lastLineSeenSide = -1;
  } else if (lastLineSeenSide != 0 && now - lastLineSeenMs <= LINE_RECOVERY_MS) {
    error = lastLineSeenSide > 0 ? LINE_LARGE_ERROR : -LINE_LARGE_ERROR;
  } else {
    stopMotors();
    return;
  }

  float derivative = 0.0f;
  if (lastLineUpdateMs != 0) derivative = error - lastLineError;
  lastLineUpdateMs = now;
  lastLineError = error;

  float absError = fabsf(error);
  int base = LINE_CENTER_BASE_SPEED;
  if (absError >= LINE_LARGE_ERROR) base = LINE_LARGE_BASE_SPEED;
  else if (absError >= LINE_MODERATE_ERROR) base = LINE_MODERATE_BASE_SPEED;
  base = min(base, lineBaseSpeed);

  int correction = constrain((int)(lineTurnGain * error + lineKd * derivative),
                             -lineCorrectionCap, lineCorrectionCap);
  int minSpeed = absError >= LINE_LARGE_ERROR ? LINE_HARD_TURN_MIN_SPEED : LINE_MIN_FORWARD_SPEED;
  int left = constrain(base - correction, minSpeed, MAX_MOTOR_SPEED);
  int right = constrain(base + correction, minSpeed, MAX_MOTOR_SPEED);
  setDrive(left, right);

  if (printTelemetry && millis() - lastPrintMs > 250) {
    lastPrintMs = millis();
    Serial.print("[ARENA_LINE] err=");
    Serial.print(error, 1);
    Serial.print(" active=");
    Serial.print(line.activeCount);
    Serial.print(" d=");
    Serial.print(derivative, 1);
    Serial.print(" base=");
    Serial.print(base);
    Serial.print(" corr=");
    Serial.print(correction);
    Serial.print(" L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.println(right);
  }
}

bool searchForArenaLine() {
  Serial.println("[HYBRID] Searching for lost line.");
  unsigned long start = millis();
  unsigned long segmentStart = millis();
  int initialSide = lastLineSeenSide;
  int searchDir = initialSide == 0 ? 1 : initialSide;

  while (millis() - start < ARENA_LINE_SEARCH_TIMEOUT_MS) {
    updateLEDs();
    messenger.loop();
    if (serialStopRequested()) {
      stopMotors();
      return false;
    }

    LineSnapshot line = readLineSnapshot();
    if (line.onLine) {
      stopMotors();
      resetLineController();
      lastLineSeenMs = millis();
      lastLineSeenSide = 0;
      Serial.println("[HYBRID] Line found.");
      return true;
    }

    if (millis() - segmentStart >= ARENA_LINE_SEARCH_SEGMENT_MS) {
      segmentStart = millis();
      searchDir = -searchDir;
    }

    if (searchDir > 0) spinLeft(ARENA_LINE_SEARCH_SPEED);
    else spinRight(ARENA_LINE_SEARCH_SPEED);
  }

  stopMotors();
  resetLineController();
  Serial.println("[HYBRID] Line search timeout.");
  return false;
}

void followRampEntryLineStep() {
  LineSnapshot line = readLineSnapshot();
  unsigned long now = millis();
  float error = line.error;

  if (line.onLine) {
    lastLineSeenMs = now;
    if (error > 3.0f) lastLineSeenSide = 1;
    else if (error < -3.0f) lastLineSeenSide = -1;
  } else if (lastLineSeenSide != 0 && now - lastLineSeenMs <= LINE_RECOVERY_MS) {
    error = lastLineSeenSide > 0 ? LINE_LARGE_ERROR : -LINE_LARGE_ERROR;
  }

  float derivative = 0.0f;
  if (lastLineUpdateMs != 0) derivative = error - lastLineError;
  lastLineUpdateMs = now;
  lastLineError = error;

  int correction = constrain((int)(lineTurnGain * error + lineKd * derivative),
                             -lineCorrectionCap, lineCorrectionCap);
  int left = constrain(rampEntryBoostSpeed - correction, LINE_MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
  int right = constrain(rampEntryBoostSpeed + correction, LINE_MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
  setDrive(left, right);
}

bool serialStopRequested() {
  if (pollRunKillSwitch()) {
    running = false;
    stopMotors();
    return true;
  }

  if (emergencyExitRequested) {
    running = false;
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
  LineSnapshot line = readLineSnapshot();
  unsigned long now = millis();
  if (line.onLine) {
    lastLineSeenMs = now;
    if (line.error > 3.0f) lastLineSeenSide = 1;
    else if (line.error < -3.0f) lastLineSeenSide = -1;
    return true;
  }

  return lastLineSeenSide != 0 && now - lastLineSeenMs <= LINE_RECOVERY_MS;
}

bool physicalLinePresent() {
  LineSnapshot line = readLineSnapshot();
  return line.onLine;
}

bool rightBranchDetected() {
  readRawLineSensors();
  uint8_t active = 0;
  for (uint8_t i = 6; i <= 8; i++) {
    uint16_t value = rawCalibrated ? normalizedRawValue(i) : sensorValues[i];
    if (value > (rawCalibrated ? 100 : rawLineThreshold)) active++;
  }
  return active >= RIGHT_BRANCH_ACTIVE_SENSORS;
}

bool confirmRightBranch() {
  // Thick tape can momentarily look like a branch. Stop and require repeated
  // consecutive right-side readings before committing to the branch turn.
  stopMotors();
  delay(80);
  for (uint8_t i = 0; i < RIGHT_BRANCH_CONFIRM_READS; i++) {
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

bool lineLostConfirmed() {
  for (uint8_t i = 0; i < LINE_LOSS_CONFIRM_READS; i++) {
    updateLEDs();
    if (serialStopRequested()) return false;
    if (checkAirlockTagDuringApproach()) return false;
    if (physicalLinePresent()) return false;
    delay(15);
  }
  resetLineController();
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

void driveArenaForwardForMs(unsigned long durationMs, int speed) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    updateLEDs();
    messenger.loop();
    if (serialStopRequested()) return;
    setDrive(speed, speed);
  }
  stopMotors();
}

void trackLineUntilEnd() {
  Serial.println("[CHAIN] Line tracking until next line end.");
  unsigned long start = millis();
  while (true) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    if (millis() - start > CHAIN_LINE_END_TIMEOUT_MS) {
      stopMotors();
      resetLineController();
      Serial.println("[CHAIN] Line end timeout.");
      return;
    }
    if (!physicalLinePresent() && lineLostConfirmed()) break;
    followLineStep();
  }
  stopMotors();
  resetLineController();
  Serial.println("[CHAIN] Next line end detected.");
}

void trackLineUntilRightBranch() {
  Serial.println("[CHAIN] Line tracking until right branch.");
  unsigned long start = millis();
  while (true) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    if (millis() - start > CHAIN_BRANCH_TIMEOUT_MS) {
      stopMotors();
      resetLineController();
      Serial.println("[CHAIN] Right branch timeout.");
      return;
    }
    if (rightBranchDetected()) {
      Serial.println("[CHAIN] Branch candidate — stopping to confirm.");
      if (confirmRightBranch()) break;
    }
    followLineStep();
  }
  stopMotors();
  resetLineController();
  Serial.println("[CHAIN] Right branch detected.");
}

void followLineIgnoringRightBranchForMs(unsigned long durationMs) {
  Serial.println("[CHAIN] Branch detection lockout after second left turn.");
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    updateLEDs();
    messenger.loop();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    followLineStep();
  }
  stopMotors();
  resetLineController();
  Serial.println("[CHAIN] Branch detection lockout complete.");
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
    messenger.loop();
    if (killSwitchLatched()) {
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

void waitForGyroPlacement() {
  stopMotors();
  calibrationLedOverride = true;
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  Serial.println("[GYRO] LEDs off. Put robot down for gyro calibration.");

  unsigned long start = millis();
  while (millis() - start < GYRO_PLACEMENT_PAUSE_MS) {
    messenger.loop();
    if (killSwitchLatched()) break;
    delay(10);
  }

  calibrationLedOverride = false;
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
  resetLineController();
  Serial.println("[GYRO] Turn done.");
}

void gyroTurnRight90() {
  gyroTurnInPlace(-PI / 2.0f);
}

void gyroTurnLeft90() {
  gyroTurnInPlace(PI / 2.0f);
}

bool reacquireLineAfterTurn(bool rightTurn) {
  if (physicalLinePresent()) {
    resetLineController();
    return true;
  }

  Serial.println("[CHAIN] Reacquiring line after turn.");
  unsigned long start = millis();
  while (millis() - start < TURN_REACQUIRE_TIMEOUT_MS) {
    updateLEDs();
    if (serialStopRequested()) {
      stopMotors();
      return false;
    }
    if (checkAirlockTagDuringApproach()) {
      stopMotors();
      return false;
    }
    if (physicalLinePresent()) {
      stopMotors();
      resetLineController();
      Serial.println("[CHAIN] Line reacquired.");
      return true;
    }
    if (rightTurn) spinRight(300);
    else spinLeft(300);
  }

  stopMotors();
  resetLineController();
  Serial.println("[CHAIN] Line reacquire timeout.");
  return false;
}

void runApproachPathStep() {
  // Chain approach sequence:
  //   follow line to end, right turn, line to end, left, line to end, left,
  //   ignore branch candidates for 2 s, scan for the right branch, turn right,
  //   and follow to the ramp/door stop.
  if (checkAirlockTagDuringApproach()) return;

  if (physicalLinePresent() || !lineLostConfirmed()) {
    followLineStep();
    return;
  }

  if (chainApproachTurnIndex > 0) {
    stopMotors();
    Serial.println("[CHAIN] Approach already completed; waiting at ramp/door.");
    return;
  }

  stopMotors();
  Serial.println("[CHAIN] Pausing before first right turn.");
  delay(500);

  Serial.println("[CHAIN] Right, line end, left, line end, left, right branch, right, ramp line.");
  driveForwardForMs(30, lineBaseSpeed);
  if (chainState != CH_APPROACH) return;
  gyroTurnRight90();
  if (chainState != CH_APPROACH) return;
  if (!reacquireLineAfterTurn(true)) return;
  stopMotors();
  delay(200);

  trackLineUntilEnd();
  if (chainState != CH_APPROACH) return;
  driveForwardForMs(50, lineBaseSpeed);
  if (chainState != CH_APPROACH) return;
  delay(200);

  gyroTurnLeft90();
  if (chainState != CH_APPROACH) return;
  if (!reacquireLineAfterTurn(false)) return;
  stopMotors();
  delay(200);

  trackLineUntilEnd();
  if (chainState != CH_APPROACH) return;
  driveForwardForMs(150, lineBaseSpeed);
  if (chainState != CH_APPROACH) return;
  delay(200);

  gyroTurnLeft90();
  if (chainState != CH_APPROACH) return;
  if (!reacquireLineAfterTurn(false)) return;
  stopMotors();
  delay(200);

  followLineIgnoringRightBranchForMs(RIGHT_BRANCH_LOCKOUT_AFTER_SECOND_LEFT_MS);
  if (chainState != CH_APPROACH) return;

  trackLineUntilRightBranch();
  if (chainState != CH_APPROACH) return;
  driveForwardForMs(150, lineBaseSpeed);
  if (chainState != CH_APPROACH) return;
  delay(200);
  
  gyroTurnRight90();
  if (chainState != CH_APPROACH) return;
  if (!reacquireLineAfterTurn(true)) return;
  stopMotors();
  delay(200);

  Serial.println("[CHAIN] Following ramp line; will stop/wait on final line loss.");
  while (true) {
    updateLEDs();
    if (serialStopRequested()) return;
    if (checkAirlockTagDuringApproach()) return;
    if (!physicalLinePresent() && lineLostConfirmed()) break;
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
    rampExitClearHits = 0;
    rampStartMs = millis();
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

bool rampTunnelDetected(float tilt, int distL, int distR) {
  bool tilted = fabsf(tilt) > tiltThreshold;
  bool wallsClose = distL < tunnelWallDist && distR < tunnelWallDist;
  return tilted || wallsClose;
}

bool validTunnelSideReadings(int distL, int distR) {
  return distL >= 3 && distL <= 80 && distR >= 3 && distR <= 80;
}

bool navigateTunnelStep() {
  // Ramp mode starts by driving forward until tilt or close side walls prove the
  // robot is inside the tunnel, then switches to ultrasonic wall following.
  float tilt = getTilt();
  int distL = readDistance(TRIG_L, ECHO_L);
  int distR = readDistance(TRIG_R, ECHO_R);

  if (!rampTunnelDetected(tilt, distL, distR)) {
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
  if (doorOpened) {
    wfLastError = 0.0f;
    unsigned long start = millis();
    while (millis() - start < rampDoorStraightMs) {
      messenger.loop();
      updateLEDs();
      if (serialStopRequested()) break;
      setDrive(rampDoorEntrySpeed, rampDoorEntrySpeed);
      delay(5);
    }
  }

  if (goingDown && doorOpened) {
    exitingThroughDoor = true;
    Serial.println("[TUNNEL] Downhill door opened; using exit speed.");
  }

  int base = tunnelBaseSpeed;
  if (goingDown && !exitingThroughDoor) {
    base = downhillRampSpeed;
  }

  distL = readDistance(TRIG_L, ECHO_L);
  distR = readDistance(TRIG_R, ECHO_R);
  if (!validTunnelSideReadings(distL, distR)) {
    wfLastError = 0.0f;
    setDrive(base, base);
    if (printTelemetry && millis() - lastPrintMs > 250) {
      lastPrintMs = millis();
      Serial.print("[TUNNEL] Bad side reading; straight. L=");
      Serial.print(distL);
      Serial.print(" R=");
      Serial.println(distR);
    }
    return true;
  }

  int error = distL - distR;
  int derivative = error - (int)wfLastError;
  wfLastError = error;

  float kp = (goingDown && !exitingThroughDoor) ? wallKpDown : wallKpUp;
  int correction = constrain((int)(kp * error + wallKd * derivative),
                             -wallCorrectionCap, wallCorrectionCap);
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

bool tunnelTraversalComplete() {
  if (!wasInTunnel) return false;
  int clearSpeed = downhillRampSpeed;
  unsigned long elapsed = millis() - rampStartMs;
  if (elapsed < minRampRunMs) {
    setDrive(clearSpeed, clearSpeed);
    return false;
  }

  rampExitClearHits++;
  if (rampExitClearHits < rampExitClearHitsRequired) {
    setDrive(clearSpeed, clearSpeed);
    return false;
  }

  setDrive(clearSpeed, clearSpeed);
  unsigned long start = millis();
  while (millis() - start < rampExitForwardMs) {
    messenger.loop();
    updateLEDs();
    if (serialStopRequested()) break;
    setDrive(clearSpeed, clearSpeed);
  }
  stopMotors();
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
  rampExitClearHits = 0;
  rampStartMs = 0;
  wfLastError = 0.0f;
  pitch = 0.0f;
  lastTiltUs = 0;
  resetLineController();
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
        rampExitClearHits = 0;
        rampStartMs = millis();
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
        rampExitClearHits = 0;
        return;
      }
      if (tunnelTraversalComplete()) {
        stopMotors();
        wasInTunnel = false;
        exitingThroughDoor = false;
        chainState = CH_ARENA;
        gameState = GAME_MAIN_COURSE;
        gameX = 0;
        gameY = 0;
        direction = 1;
        courseSegmentIndex = 0;
        courseNodesCompleted = 0;
        Serial.println("[CHAIN] Tunnel cleared. Entering arena game path.");
        return;
      }
      followRampEntryLineStep();
      return;

    case CH_ARENA:
      gameState = GAME_MAIN_COURSE;
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

void setDeadReckoningDrive(int correction) {
  int leftSpeed = constrain(activeDriveSpeed + correction + driveTrim, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
  int rightSpeed = constrain(activeDriveSpeed - correction - driveTrim, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
  setDrive(leftSpeed, rightSpeed);
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
    setDeadReckoningDrive(correction);
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
    setDeadReckoningDrive(correction);
  }

  stopMotors();
  calibrateGyroBias();
  return true;
}

bool waitForRfidTag(String& tagId, unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    messenger.loop();
    updateLEDs();
    if (serialStopRequested()) return false;
    if (readRfidTag(tagId)) return true;
    delay(10);
  }
  return false;
}

bool driveToNodeAndReadTag(String& tagId, bool needTag) {
  tagId = "";
  zeroDriveState();
  const long minRfidCounts = COUNTS_PER_NODE / 3;

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

    if (needTag && avg >= minRfidCounts && readRfidTag(tagId)) {
      Serial.print("[DR] RFID node stop tag=");
      Serial.println(tagId);
      break;
    }

    int correction = constrain((int)(DRIVE_KP * fusedHeading), -189, 189);
    setDeadReckoningDrive(correction);
  }

  stopMotors();
  calibrateGyroBias();

  if (needTag && tagId.length() == 0) {
    Serial.println("[RFID] Waiting at node for tag.");
    if (!waitForRfidTag(tagId, NODE_RFID_WAIT_MS)) {
      Serial.println("[RFID] No node tag found after stop.");
      return true;
    }
    Serial.print("[RFID] Node tag=");
    Serial.println(tagId);
  }
  return true;
}

bool lineTrackToNodeAndReadTag(String& tagId) {
  tagId = "";
  resetLineController();
  unsigned long start = millis();

  if (!physicalLinePresent() && lineLostConfirmed()) {
    Serial.println("[HYBRID] No line at node start; using gyro/RFID node drive.");
    return driveToNodeAndReadTag(tagId, true);
  }

  while (true) {
    updateLEDs();
    messenger.loop();
    if (serialStopRequested()) {
      stopMotors();
      return false;
    }

    unsigned long elapsed = millis() - start;
    if (elapsed >= NODE_RFID_LOCKOUT_MS && readRfidTag(tagId)) {
      stopMotors();
      resetLineController();
      calibrateGyroBias();
      Serial.print("[HYBRID] RFID node tag=");
      Serial.println(tagId);
      return true;
    }

    if (elapsed > NODE_LINE_TIMEOUT_MS) {
      stopMotors();
      resetLineController();
      Serial.println("[HYBRID] Timed out before finding next RFID node.");
      return false;
    }

    if (!physicalLinePresent() && lineLostConfirmed()) {
      if (!searchForArenaLine()) {
        Serial.println("[HYBRID] Could not reacquire line; falling back to gyro/RFID node drive.");
        return driveToNodeAndReadTag(tagId, true);
      }
    }

    followArenaLineStep();
  }
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
  mc1.setSpeed(SEED_MOTOR_CHANNEL, SEED_SPEED);
  unsigned long start = millis();
  while (millis() - start < SEED_DURATION_MS) {
    messenger.loop();
    updateLEDs();
    if (serialStopRequested()) break;
    delay(5);
  }
  mc1.setSpeed(SEED_MOTOR_CHANNEL, 0);
  Serial.println("[SEED] Done.");
}

bool sendSeedPlanted(const String& tagId) {
  char msg[96];
  snprintf(msg, sizeof(msg),
           "type=seedPlanted tag_id=%s board_id=%s",
           tagId.c_str(), board);
  bool sent = false;
  for (uint8_t attempt = 0; attempt < 3 && !sent; attempt++) {
    messenger.loop();
    sent = messenger.sendToBoard("server", msg);
    if (!sent) delay(50);
  }
  Serial.print("[SEED] seedPlanted sent for ");
  Serial.print(tagId);
  Serial.println(sent ? " — sent" : " — SEND FAILED");
  return sent;
}

void plantSeedsAtNode(const String& tagId) {
  uint8_t drops = 0;
  while (drops < MAX_SEEDS_PER_NODE) {
    dropSeed();
    sendSeedPlanted(tagId);
    seedsPlanted++;
    drops++;
    Serial.print("[GAME] Seeds planted=");
    Serial.println(seedsPlanted);
  }

  addToCache(plantedTags, plantedTagCount, tagId);
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
  String tagId;
  if (!readRfidTag(tagId)) return;

  Serial.print("[RFID] Arena node tag=");
  Serial.println(tagId);
  if (!queryFertility(tagId)) return;
  plantSeedsAtNode(tagId);
}

void checkNodeTagForSeed(const String& tagId) {
  if (tagId.length() == 0) return;

  Serial.print("[RFID] Course node tag=");
  Serial.println(tagId);
  if (!queryFertility(tagId)) return;
  plantSeedsAtNode(tagId);
}

uint8_t courseNodeIndex() {
  uint8_t index = courseNodesCompleted;
  for (uint8_t i = 0; i < courseSegmentIndex; i++) {
    index += courseSegmentNodes[i];
  }
  return index;
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

bool moveCourseNode(int targetDir) {
  drFaceDir(targetDir);
  if (serialStopRequested()) return false;

  bool finalCourseNode = courseSegmentIndex == COURSE_SEGMENT_COUNT - 1 &&
                         courseNodesCompleted + 1 == courseSegmentNodes[courseSegmentIndex];
  bool inclineSegment = courseSegmentIndex == INCLINE_SEGMENT_INDEX;
  uint8_t nodeIndex = courseNodeIndex();
  bool useLineTracking = nodeIndex < LINE_TRACK_NODE_COUNT;
  int courseSpeed = DRIVE_SPEED;
  if (nodeIndex == 0) courseSpeed = FIRST_NODE_SPEED;
  else if (inclineSegment) courseSpeed = INCLINE_COURSE_SPEED;

  int savedLineBaseSpeed = lineBaseSpeed;
  int savedActiveDriveSpeed = activeDriveSpeed;
  lineBaseSpeed = courseSpeed;
  activeDriveSpeed = courseSpeed;

  if (nodeIndex == 0) {
    Serial.println("[GAME] First node slow speed active.");
  } else if (inclineSegment) {
    Serial.println("[GAME] Incline segment speed boost active.");
  }
  Serial.println(useLineTracking ? "[GAME] Node move using line tracking." :
                                   "[GAME] Node move using gyro/RFID only.");

  String tagId;
  bool moved = useLineTracking ? lineTrackToNodeAndReadTag(tagId)
                               : driveToNodeAndReadTag(tagId, true);
  lineBaseSpeed = savedLineBaseSpeed;
  activeDriveSpeed = savedActiveDriveSpeed;
  if (!moved) {
    return false;
  }

  if (targetDir == 0) gameX--;
  else if (targetDir == 1) gameY++;
  else if (targetDir == 2) gameX++;
  else if (targetDir == 3) gameY--;

  Serial.print("[GAME] Course node x=");
  Serial.print(gameX);
  Serial.print(" y=");
  Serial.print(gameY);
  Serial.print(" tag=");
  Serial.println(tagId.length() ? tagId : "NONE");

  if (finalCourseNode) {
    Serial.println("[GAME] Final course node reached; robot will stop after this leg.");
  }

  checkNodeTagForSeed(tagId);

  return true;
}

bool runMainCourseStep() {
  if (courseSegmentIndex >= COURSE_SEGMENT_COUNT) {
    stopMotors();
    running = false;
    gameState = GAME_DONE;
    Serial.println("[GAME] Course complete. Motors stopped.");
    return true;
  }

  if (courseNodesCompleted < courseSegmentNodes[courseSegmentIndex]) {
    if (!moveCourseNode(direction)) return false;
    if (gameState == GAME_REQUEST_EXIT_B) return true;
    courseNodesCompleted++;
    return true;
  }

  int8_t turn = courseTurnAfterSegment[courseSegmentIndex];
  if (turn != COURSE_TURN_NONE) {
    driveArenaForwardForMs(ARENA_PRE_TURN_FORWARD_MS, lineBaseSpeed);
    if (serialStopRequested()) return false;
    if (turn == COURSE_TURN_LEFT) drTurnLeft();
    else if (turn == COURSE_TURN_RIGHT) drTurnRight();
    if (serialStopRequested()) return false;
  }

  if (courseSegmentIndex < COURSE_SEGMENT_COUNT - 1) {
    courseSegmentIndex++;
    courseNodesCompleted = 0;
    return true;
  }

  stopMotors();
  running = false;
  gameState = GAME_DONE;
  Serial.println("[GAME] Course complete. Motors stopped.");
  return true;
}

bool requestExitBAirlock() {
  if (finalNodeTag.length() == 0) {
    Serial.println("[DOOR] No final RFID tag available for airlock B request.");
    return false;
  }
  exitBRequestSent = requestAirlock("B", finalNodeTag.c_str());
  airlockWaitStartMs = millis();
  return exitBRequestSent;
}

void beginExitRamp() {
  wasInTunnel = false;
  exitingThroughDoor = false;
  rampExitClearHits = 0;
  rampStartMs = millis();
  wfLastError = 0.0f;
  pitch = 0.0f;
  lastTiltUs = 0;
  doorOpenHits = 0;
  gameState = GAME_EXIT_RAMP;
  Serial.println("[DOOR] Airlock B clear. Descending ramp.");
}

bool runExitRampStep() {
  if (navigateTunnelStep()) {
    wasInTunnel = true;
    rampExitClearHits = 0;
    return true;
  }

  if (tunnelTraversalComplete()) {
    wasInTunnel = false;
    exitingThroughDoor = false;
    gameState = GAME_EXIT_LINE;
    Serial.println("[EXIT] Ramp cleared. Line tracking down exit line.");
    return true;
  }

  followRampEntryLineStep();
  return true;
}

bool runExitLineStep() {
  if (physicalLinePresent() || !lineLostConfirmed()) {
    followLineStep();
    return true;
  }

  stopMotors();
  running = false;
  gameState = GAME_DONE;
  Serial.println("[EXIT] Line ended. Motors stopped.");
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
    if (pollRunKillSwitch()) {
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
    const char* targetTeam = revivalTargetAvailable ? revivalTargetTeam : "unknown";
    const char* targetBoard = revivalTargetAvailable ? revivalTargetBoard : "unknown";
    sendReviveRequest(targetTeam, targetBoard);
    clearRevivalTarget();
    revivalDone = true;
    running = true;
    Serial.println("[REVIVAL] Bumper contact. Revive request sent; motors stopped.");
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
  gameState = GAME_MAIN_COURSE;
  emergencyExitRequested = false;
  pendingModeAdvance = false;
  calibrationLedOverride = false;
  serverReadyIdle = false;
  runKillArmed = false;
  chainState = CH_ARENA;
  courseSegmentIndex = 0;
  courseNodesCompleted = 0;
  activeDriveSpeed = DRIVE_SPEED;
  exitBRequestSent = false;
  finalNodeTag = "";
  seedsPlanted = 0;
  seenTagCount = 0;
  plantedTagCount = 0;
  gameX = 0;
  gameY = 0;
  direction = 1;
  wasInTunnel = false;
  exitingThroughDoor = false;
  airlockAccepted = false;
  airlockDenied = false;
  airlockRequestSent = false;
  approachReadyForRamp = false;
  doorOpenHits = 0;
  rampExitClearHits = 0;
  rampStartMs = 0;
  wfLastError = 0.0f;
  resetLineController();
  updateLEDs();
  calibrateGyroBias();
  running = !killSwitchLatched();
  Serial.println("[GAME] Started arena 9-left-2-incline-4-stop path. Chain/ramp entry skipped.");
}

void advanceMode() {
  startMode(MODE_GAME);
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
  Serial.print(" driveTrim=");
  Serial.print(driveTrim);
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
  Serial.print(downhillReduction);
  Serial.print(" downhillRampSpeed=");
  Serial.print(downhillRampSpeed);
  Serial.print(" rampDoorEntrySpeed=");
  Serial.print(rampDoorEntrySpeed);
  Serial.print(" rampDoorStraightMs=");
  Serial.println(rampDoorStraightMs);

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
  Serial.println("  v / z  drive trim up/down (positive counters right drift)");
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
      if (gamePrepared && messenger.isConnected() && enable && !killSwitchLatched()) {
        startMode(MODE_GAME);
      } else {
        Serial.println("[RUN] Ignored until prepared, connected, and enabled.");
      }
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
    } else if (c == 'v' || c == 'V') {
      driveTrim = constrain(driveTrim + 3, -80, 80);
      printStatus();
    } else if (c == 'z' || c == 'Z') {
      driveTrim = constrain(driveTrim - 3, -80, 80);
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
      waitForGyroPlacement();
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
  commsStarted = true;
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
  autoCalibrateRawLine();
  waitForGyroPlacement();
  calibrateGyroBias();
  gamePrepared = true;
  serverReadyIdle = true;
  gameState = GAME_WAIT_READY;
  modeButtonWasDown = digitalRead(modeButtonPin) == LOW;
  Serial.println("[GAME] Prepared. Waiting for server connection and pin 49 start.");
}

void loop() {
  messenger.loop();
  commsHeartbeatCheck();
  sendStatusUpdate();

  updateLEDs();

  if (pollRunKillSwitch()) {
    stopMotors();
    return;
  }

  handleSerial();

  if (!enable) {
    stopMotors();
    return;
  }

  if (!running) {
    if (gamePrepared && messenger.isConnected() && enable && modeButtonPressedEvent()) {
      startMode(MODE_GAME);
      return;
    }
    stopMotors();
    return;
  }

  if (!revivalTargetAvailable && !anyBumperPressed()) {
    revivalDone = false;
  }

  if ((revivalTargetAvailable || anyBumperPressed()) &&
      !emergencyExitRequested && gameState != GAME_CHAIN_ENTRY) {
    runRevivalStep();
    return;
  }

  switch (gameState) {
    case GAME_WAIT_READY:
      stopMotors();
      running = false;
      break;

    case GAME_CHAIN_ENTRY:
      runChainStep();
      break;

    case GAME_MAIN_COURSE:
      runMainCourseStep();
      break;

    case GAME_REQUEST_EXIT_B:
      stopMotors();
      if (requestExitBAirlock()) {
        gameState = GAME_WAIT_EXIT_B;
        Serial.println("[DOOR] Waiting for airlock B reply and ultrasonic clearance.");
      } else {
        running = false;
        gameState = GAME_DONE;
      }
      break;

    case GAME_WAIT_EXIT_B:
      stopMotors();
      if (airlockAccepted && doorIsOpenStable()) {
        beginExitRamp();
      } else if (airlockDenied && millis() - airlockWaitStartMs > AIRLOCK_RETRY_MS) {
        Serial.println("[DOOR] Retrying airlock B request.");
        requestExitBAirlock();
      }
      break;

    case GAME_EXIT_RAMP:
      runExitRampStep();
      break;

    case GAME_EXIT_LINE:
      runExitLineStep();
      break;

    case GAME_DONE:
      stopMotors();
      running = false;
      break;

    default:
      stopMotors();
      running = false;
      break;
  }
}
