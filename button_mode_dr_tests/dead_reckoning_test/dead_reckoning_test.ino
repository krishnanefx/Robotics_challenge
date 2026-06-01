// Dead-reckoning standalone tuner.
// Drives the 25 cm node path using encoders for distance and gyro yaw for
// 90-degree turns. RFID can stop a node early when a tag is seen. This is the
// source of the tuned dead-reckoning values used by the final button-mode sketch.
// Main knobs: COUNTS_PER_NODE, DRIVE_SPEED, DRIVE_KP, TURN_SCALE_LEFT/RIGHT.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MFRC522_I2C.h>
#include <Motoron.h>
#include <Wire.h>
#include <math.h>

#define RFID_WIRE Wire2
#define RFID_RESET_PIN -1

MotoronI2C mc1(18);  // ch2 = right drive, ch3 = left drive
MotoronI2C mc2(17);  // ch2 = right drive, ch3 = left drive
Adafruit_MPU6050 mpu;
MFRC522_I2C* rfid = nullptr;

const int encoder1PinA = 52;
const int encoder1PinB = 50;
const int encoder2PinA = 53;
const int encoder2PinB = 51;

const float WHEEL_DIAM_CM = 6.5f;
const float TRACK_WIDTH_CM = 17.0f;
const float COUNTS_PER_REV = 144.0f;
const float CM_PER_COUNT = (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;
const int COUNTS_PER_NODE = (int)(25.0f / CM_PER_COUNT);

const int MAX_MOTOR_SPEED = 660;
const int DRIVE_SPEED = 300;
const float DRIVE_KP = 80.0f;
const int TURN_FAST = 660;
const int TURN_SLOW = 566;
const float TURN_SCALE_LEFT = 0.96f;
const float TURN_SCALE_RIGHT = 0.90f;

const int MIN_FORWARD_SPEED = 200;
const int MIN_TURN_SPEED = 400;

const float ALPHA = 0.98f;

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

float gyroBiasZ = 0.0f;
float fusedHeading = 0.0f;
unsigned long lastHeadingUs = 0;
long prevEncL = 0;
long prevEncR = 0;

// direction index: 0=W  1=N  2=E  3=S
int direction = 1;
char dirNames[4] = {'W', 'N', 'E', 'S'};

const int NODE_COUNT = 6;
int pathx[NODE_COUNT] = {0, 0, 0, 1, 1, 1};
int pathy[NODE_COUNT] = {0, 1, 2, 2, 3, 4};

int drIndex = 1;
bool drDone = false;
bool runningPath = false;
bool useRfidEarlyStop = true;

void countEncoder1() {
  if (digitalRead(encoder1PinB) == HIGH) encoderCount1++;
  else encoderCount1--;
}

void countEncoder2() {
  if (digitalRead(encoder2PinB) == LOW) encoderCount2++;
  else encoderCount2--;
}

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  for (int ch = 1; ch <= 3; ch++) {
    mc.setMaxAcceleration(ch, 200);
    mc.setMaxDeceleration(ch, 300);
  }
}

void stopMotors() {
  for (int ch = 1; ch <= 3; ch++) {
    mc1.setSpeed(ch, 0);
    mc2.setSpeed(ch, 0);
  }
}

byte findRfidI2CAddress() {
  byte foundAddress = 0;
  byte foundCount = 0;

  Serial.println("[RFID] Scanning Wire2 I2C bus...");
  for (byte address = 1; address < 127; address++) {
    RFID_WIRE.beginTransmission(address);
    if (RFID_WIRE.endTransmission() == 0) {
      if (foundAddress == 0) foundAddress = address;
      foundCount++;
      Serial.print("[RFID] Found I2C device at 0x");
      if (address < 0x10) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (foundCount == 0) Serial.println("[RFID] No device found; odometry-only mode");
  if (foundCount > 1) Serial.println("[RFID] Multiple devices found; using first");
  return foundAddress;
}

void setupRfid() {
  RFID_WIRE.begin();
  delay(100);

  byte address = findRfidI2CAddress();
  if (address == 0) return;

  rfid = new MFRC522_I2C(address, RFID_RESET_PIN, &RFID_WIRE);
  rfid->PCD_Init();
  Serial.print("[RFID] Reader initialized at 0x");
  if (address < 0x10) Serial.print("0");
  Serial.println(address, HEX);
}

bool readRfidTag(char* tagId, size_t tagIdLen) {
  if (rfid == nullptr) return false;
  if (!rfid->PICC_IsNewCardPresent()) return false;
  if (!rfid->PICC_ReadCardSerial()) return false;

  snprintf(tagId, tagIdLen, "%02X%02X%02X%02X",
           rfid->uid.uidByte[0], rfid->uid.uidByte[1],
           rfid->uid.uidByte[2], rfid->uid.uidByte[3]);
  rfid->PICC_HaltA();
  return true;
}

bool serialStopRequested() {
  if (!Serial.available()) return false;
  char c = Serial.read();
  if (c == 's' || c == 'S') {
    stopMotors();
    runningPath = false;
    Serial.println("[DEAD_RECKONING] Stopped");
    return true;
  }
  return false;
}

float readGyroZ_radps() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return g.gyro.z;
}

void rezeroGyroBias() {
  stopMotors();
  delay(200);

  const int N = 500;
  float sum = 0.0f;
  float sumSq = 0.0f;
  for (int i = 0; i < N; i++) {
    float z = readGyroZ_radps();
    sum += z;
    sumSq += z * z;
    delay(2);
  }

  float mean = sum / N;
  float variance = (sumSq / N) - (mean * mean);
  if (variance > (0.05f * 0.05f)) {
    Serial.print("[GYRO] Re-zero skipped, variance=");
    Serial.println(variance, 6);
    return;
  }

  gyroBiasZ = mean;
  Serial.print("[GYRO] Bias Z=");
  Serial.println(gyroBiasZ, 6);
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
  unsigned long now = micros();
  float dt = (now - lastHeadingUs) / 1000000.0f;
  lastHeadingUs = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  float gyroAngle = (readGyroZ_radps() - gyroBiasZ) * dt;

  long curL = encoderCount1;
  long curR = encoderCount2;
  float distL = (float)(curL - prevEncL) * CM_PER_COUNT;
  float distR = (float)(curR - prevEncR) * CM_PER_COUNT;
  prevEncL = curL;
  prevEncR = curR;

  float encAngle = (distR - distL) / TRACK_WIDTH_CM;
  fusedHeading += ALPHA * gyroAngle + (1.0f - ALPHA) * encAngle;
}

void printCounts() {
  noInterrupts();
  long left = encoderCount1;
  long right = encoderCount2;
  interrupts();

  long avg = (abs(left) + abs(right)) / 2;
  Serial.print("L=");
  Serial.print(left);
  Serial.print(" R=");
  Serial.print(right);
  Serial.print(" avg=");
  Serial.print(avg);
  Serial.print(" cm=");
  Serial.print(avg * CM_PER_COUNT, 2);
  Serial.print(" headingDeg=");
  Serial.print(fusedHeading * 180.0f / PI, 2);
  Serial.print(" facing=");
  Serial.println(dirNames[direction]);
}

void turnInPlace(float angleRad) {
  int sign = (angleRad > 0.0f) ? 1 : -1;
  float turnScale = (sign > 0) ? TURN_SCALE_LEFT : TURN_SCALE_RIGHT;
  float target = fabsf(angleRad) * turnScale;
  float accumulated = 0.0f;
  unsigned long prevUs = micros();

  while (accumulated < target) {
    if (serialStopRequested()) return;

    unsigned long now = micros();
    float dt = (now - prevUs) / 1000000.0f;
    prevUs = now;
    if (dt <= 0.0f || dt > 0.05f) continue;

    accumulated += fabsf((readGyroZ_radps() - gyroBiasZ) * dt);

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
  Serial.print("[TURN] Facing ");
  Serial.println(dirNames[direction]);
  rezeroGyroBias();
}

void turnLeft() {
  turnInPlace(PI / 2.0f);
}

void turnRight() {
  turnInPlace(-PI / 2.0f);
}

void faceDir(int targetDir) {
  int diff = (targetDir - direction + 4) % 4;
  if (diff == 1) turnRight();
  else if (diff == 3) turnLeft();
  else if (diff == 2) {
    turnRight();
    turnRight();
  }
}

void driveOneNode() {
  zeroDriveState();
  Serial.print("[DEAD_RECKONING] Driving one node, target counts=");
  Serial.println(COUNTS_PER_NODE);

  while (true) {
    if (serialStopRequested()) return;
    updateHeading();

    noInterrupts();
    long left = encoderCount1;
    long right = encoderCount2;
    interrupts();

    long avg = (abs(left) + abs(right)) / 2;
    if (avg >= COUNTS_PER_NODE) break;

    char tagId[12];
    if (useRfidEarlyStop && readRfidTag(tagId, sizeof(tagId))) {
      Serial.print("[RFID] Early node stop tag=");
      Serial.print(tagId);
      Serial.print(" avgCounts=");
      Serial.println(avg);
      break;
    }

    int correction = constrain((int)(DRIVE_KP * fusedHeading), -189, 189);
    int leftSpeed = constrain(DRIVE_SPEED + correction, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);
    int rightSpeed = constrain(DRIVE_SPEED - correction, MIN_FORWARD_SPEED, MAX_MOTOR_SPEED);

    mc1.setSpeed(2, rightSpeed);
    mc1.setSpeed(3, leftSpeed);
    mc2.setSpeed(2, rightSpeed);
    mc2.setSpeed(3, leftSpeed);
  }

  stopMotors();
  printCounts();
  rezeroGyroBias();
}

void resetPath() {
  drIndex = 1;
  drDone = false;
  runningPath = false;
  direction = 1;
  stopMotors();
  Serial.println("[DEAD_RECKONING] Path reset. Facing N, next index=1.");
}

void runPath() {
  if (drDone) {
    Serial.println("[DEAD_RECKONING] Path already complete. Send x to reset.");
    return;
  }

  runningPath = true;
  for (; drIndex < NODE_COUNT && runningPath; drIndex++) {
    int dx = pathx[drIndex] - pathx[drIndex - 1];
    int dy = pathy[drIndex] - pathy[drIndex - 1];

    int targetDir;
    if (dy > 0) targetDir = 1;
    else if (dy < 0) targetDir = 3;
    else if (dx > 0) targetDir = 2;
    else targetDir = 0;

    Serial.print("[DEAD_RECKONING] Node index ");
    Serial.print(drIndex);
    Serial.print(" targetDir=");
    Serial.println(dirNames[targetDir]);

    faceDir(targetDir);
    if (!runningPath) return;
    driveOneNode();
  }

  stopMotors();
  drDone = true;
  runningPath = false;
  Serial.println("[DEAD_RECKONING] Path complete");
}

void printHelp() {
  Serial.println();
  Serial.println("Dead reckoning standalone test");
  Serial.println("Commands:");
  Serial.println("  f      drive one 25 cm node");
  Serial.println("  l      gyro turn left 90 deg");
  Serial.println("  r      gyro turn right 90 deg");
  Serial.println("  p      run full path");
  Serial.println("  x      reset path state");
  Serial.println("  z      re-zero gyro bias");
  Serial.println("  c      print encoder counts/status");
  Serial.println("  i      toggle RFID early stop");
  Serial.println("  s      stop immediately");
  Serial.println("  ?      print help");
  Serial.println();
  Serial.print("COUNTS_PER_NODE=");
  Serial.println(COUNTS_PER_NODE);
  Serial.print("DRIVE_SPEED=");
  Serial.print(DRIVE_SPEED);
  Serial.print(" MIN_FORWARD_SPEED=");
  Serial.print(MIN_FORWARD_SPEED);
  Serial.print(" MIN_TURN_SPEED=");
  Serial.println(MIN_TURN_SPEED);
  Serial.print("TURN_SCALE_LEFT=");
  Serial.print(TURN_SCALE_LEFT, 2);
  Serial.print(" TURN_SCALE_RIGHT=");
  Serial.println(TURN_SCALE_RIGHT, 2);
  Serial.print("RFID early stop=");
  Serial.print(useRfidEarlyStop ? "ON" : "OFF");
  Serial.print(" reader=");
  Serial.println(rfid == nullptr ? "not found" : "ready");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  Wire.begin();
  Wire1.begin();
  RFID_WIRE.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  stopMotors();

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), countEncoder2, RISING);

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found.");
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  setupRfid();

  Serial.println("Keep robot still. Calibrating gyro...");
  rezeroGyroBias();
  resetPath();
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == 'f' || c == 'F') {
    driveOneNode();
  } else if (c == 'l' || c == 'L') {
    turnLeft();
  } else if (c == 'r' || c == 'R') {
    turnRight();
  } else if (c == 'p' || c == 'P') {
    runPath();
  } else if (c == 'x' || c == 'X') {
    resetPath();
  } else if (c == 'z' || c == 'Z') {
    rezeroGyroBias();
  } else if (c == 'c' || c == 'C') {
    printCounts();
  } else if (c == 'i' || c == 'I') {
    useRfidEarlyStop = !useRfidEarlyStop;
    Serial.print("[DEAD_RECKONING] RFID early stop=");
    Serial.println(useRfidEarlyStop ? "ON" : "OFF");
  } else if (c == 's' || c == 'S') {
    stopMotors();
    runningPath = false;
    Serial.println("[DEAD_RECKONING] Stopped");
  } else if (c == '?') {
    printHelp();
  }
}
