// Raw IR line-tracking tuner.
// Uses the 9 RC-style IR sensors directly, with no QTRSensors library. Tune
// baseSpeed for straight speed, turnGain for correction strength, and
// rawLineThreshold only when calibration is not available or looks wrong.

#include <Motoron.h>
#include <Wire.h>

MotoronI2C mc1(18);  // ch2 = right drive, ch3 = left drive
MotoronI2C mc2(17);  // ch2 = right drive, ch3 = left drive

const uint8_t SensorCount = 9;
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];
uint8_t sensorPins[SensorCount] = {23, 24, 25, 26, 27, 28, 29, 30, 31};

// Positive weights steer one way, negative weights steer the other.
// If correction is backwards, send serial command x to flip direction.
float weights[SensorCount] = {0.15, 0.12, 0.08, 0.04, 0.0, -0.04, -0.08, -0.12, -0.15};

const uint16_t RAW_SENSOR_TIMEOUT_US = 2500;

int baseSpeed = 245;
float turnGain = 1.30f;
int minSpeed = -250;
int maxSpeed = 700;
uint16_t rawLineThreshold = 200;
bool rawCalibrated = false;
bool running = false;
bool flipCorrection = false;
bool printSensors = false;

unsigned long lastPrintMs = 0;

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
  left = constrain(left, minSpeed, maxSpeed);
  right = constrain(right, minSpeed, maxSpeed);

  mc1.setSpeed(2, right);
  mc1.setSpeed(3, left);
  mc2.setSpeed(2, right);
  mc2.setSpeed(3, left);
}

void stopDrive() {
  setDrive(0, 0);
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

void spinLeft(int speed) {
  setDrive(-speed, speed);
}

void spinRight(int speed) {
  setDrive(speed, -speed);
}

void autoCalibrateRawLine() {
  Serial.println("Auto calibration starting. Robot will sweep left/right.");
  resetRawCalibration();
  delay(200);

  const int calSpeed = 500;
  const int sampleDelayMs = 20;

  for (uint16_t i = 0; i < 10; i++) { spinRight(calSpeed); sampleRawCalibration(); delay(sampleDelayMs); }
  for (uint16_t i = 0; i < 20; i++) { spinLeft(calSpeed);  sampleRawCalibration(); delay(sampleDelayMs); }
  for (uint16_t i = 0; i < 20; i++) { spinRight(calSpeed); sampleRawCalibration(); delay(sampleDelayMs); }
  for (uint16_t i = 0; i < 10; i++) { spinLeft(calSpeed);  sampleRawCalibration(); delay(sampleDelayMs); }

  stopDrive();
  rawCalibrated = true;
  Serial.println("Auto calibration done.");
}

float readWeightedError() {
  readRawLineSensors();
  float error = 0.0f;
  for (uint8_t i = 0; i < SensorCount; i++) {
    error += normalizedRawValue(i) * weights[i];
  }
  return flipCorrection ? -error : error;
}

void followLineStep() {
  float error = readWeightedError();
  int correction = (int)(error * turnGain);

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  setDrive(leftSpeed, rightSpeed);

  if (printSensors && millis() - lastPrintMs > 200) {
    lastPrintMs = millis();
    Serial.print("err=");
    Serial.print(error, 1);
    Serial.print(" corr=");
    Serial.print(correction);
    Serial.print(" L=");
    Serial.print(leftSpeed);
    Serial.print(" R=");
    Serial.print(rightSpeed);
    Serial.print(" sensors=");
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(normalizedRawValue(i));
      if (i < SensorCount - 1) Serial.print(",");
    }
    Serial.println();
  }
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

void turnLeftUntilLine() {
  Serial.println("Turn left: losing line, then finding next line");
  while (onLine()) spinLeft(600);
  while (!onLine()) spinLeft(600);
  stopDrive();
  Serial.println("Left turn done");
}

void turnRightUntilLine() {
  Serial.println("Turn right: losing line, then finding next line");
  while (onLine()) spinRight(600);
  while (!onLine()) spinRight(600);
  stopDrive();
  Serial.println("Right turn done");
}

void printStatus() {
  Serial.print("baseSpeed=");
  Serial.print(baseSpeed);
  Serial.print(" turnGain=");
  Serial.print(turnGain, 2);
  Serial.print(" minSpeed=");
  Serial.print(minSpeed);
  Serial.print(" maxSpeed=");
  Serial.print(maxSpeed);
  Serial.print(" rawThreshold=");
  Serial.print(rawLineThreshold);
  Serial.print(" calibrated=");
  Serial.print(rawCalibrated ? "true" : "false");
  Serial.print(" flipCorrection=");
  Serial.print(flipCorrection ? "true" : "false");
  Serial.print(" running=");
  Serial.println(running ? "true" : "false");
}

void printHelp() {
  Serial.println();
  Serial.println("Line tracking tune");
  Serial.println("Commands:");
  Serial.println("  g      go / start line following");
  Serial.println("  s      stop");
  Serial.println("  + / -  base speed up/down");
  Serial.println("  ] / [  turn gain up/down");
  Serial.println("  p      toggle sensor/motor printing");
  Serial.println("  x      flip correction direction if steering is backwards");
  Serial.println("  k      auto-calibrate by sweeping left/right");
  Serial.println("  y / h  raw threshold up/down (used before calibration)");
  Serial.println("  l      test left intersection turn");
  Serial.println("  r      test right intersection turn");
  Serial.println("  c      print status");
  Serial.println("  ?      print this help");
  Serial.println();
  Serial.println("If it does not turn enough: increase turnGain with ].");
  Serial.println("If it turns too slowly: increase baseSpeed with + and/or turnGain with ].");
  Serial.println("If it drives away from the line: press x.");
  Serial.println();
  printStatus();
}

void setup() {
  Serial.begin(115200);

  Wire1.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  stopDrive();

  resetRawCalibration();
  autoCalibrateRawLine();

  printHelp();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      running = true;
      Serial.println("Running");
    } else if (c == 's' || c == 'S') {
      running = false;
      stopDrive();
      Serial.println("Stopped");
    } else if (c == '+') {
      baseSpeed = constrain(baseSpeed + 25, 0, 800);
      printStatus();
    } else if (c == '-') {
      baseSpeed = constrain(baseSpeed - 25, 0, 800);
      printStatus();
    } else if (c == ']') {
      turnGain += 0.10f;
      printStatus();
    } else if (c == '[') {
      turnGain = max(0.10f, turnGain - 0.10f);
      printStatus();
    } else if (c == 'p' || c == 'P') {
      printSensors = !printSensors;
      Serial.print("printSensors=");
      Serial.println(printSensors ? "true" : "false");
    } else if (c == 'x' || c == 'X') {
      flipCorrection = !flipCorrection;
      printStatus();
    } else if (c == 'k' || c == 'K') {
      running = false;
      stopDrive();
      autoCalibrateRawLine();
    } else if (c == 'y' || c == 'Y') {
      rawLineThreshold = constrain(rawLineThreshold + 50, 50, RAW_SENSOR_TIMEOUT_US);
      printStatus();
    } else if (c == 'h' || c == 'H') {
      rawLineThreshold = constrain(rawLineThreshold - 50, 50, RAW_SENSOR_TIMEOUT_US);
      printStatus();
    } else if (c == 'l' || c == 'L') {
      running = false;
      turnLeftUntilLine();
    } else if (c == 'r' || c == 'R') {
      running = false;
      turnRightUntilLine();
    } else if (c == 'c' || c == 'C') {
      printStatus();
    } else if (c == '?') {
      printHelp();
    }
  }

  if (running) {
    followLineStep();
  }
}
