// Gyro turn tuner.
// Tests left/right 90-degree pivots using the MPU6050 Z gyro. If the robot
// overshoots, lower the matching turnScale value; if it undershoots, raise it.
// Keep the robot still during bias calibration before each test.

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Motoron.h>
#include <Wire.h>
#include <math.h>

MotoronI2C mc1(18);  // ch2 = right drive, ch3 = left drive
MotoronI2C mc2(17);  // ch2 = right drive, ch3 = left drive
Adafruit_MPU6050 mpu;

float gyroBiasZ = 0.0f;
float turnScaleLeft = 0.96f;
float turnScaleRight = 0.90f;
int turnFast = 700;
int turnSlow = 600;

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  for (int ch = 1; ch <= 3; ch++) {
    mc.setMaxAcceleration(ch, 200);
    mc.setMaxDeceleration(ch, 300);
  }
}

void setTurnSpeed(int sign, int speed) {
  mc1.setSpeed(2, sign * speed);
  mc1.setSpeed(3, -sign * speed);
  mc2.setSpeed(2, sign * speed);
  mc2.setSpeed(3, -sign * speed);
}

void stopDrive() {
  mc1.setSpeed(2, 0);
  mc1.setSpeed(3, 0);
  mc2.setSpeed(2, 0);
  mc2.setSpeed(3, 0);
}

float readGyroZ_radps() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return g.gyro.z;
}

void rezeroGyroBias() {
  stopDrive();
  delay(200);
  const int sampleCount = 500;
  float sum = 0.0f;
  float sumSq = 0.0f;

  for (int i = 0; i < sampleCount; i++) {
    float z = readGyroZ_radps();
    sum += z;
    sumSq += z * z;
    delay(2);
  }

  float mean = sum / sampleCount;
  float variance = (sumSq / sampleCount) - (mean * mean);
  gyroBiasZ = mean;

  Serial.print("gyroBiasZ=");
  Serial.print(gyroBiasZ, 6);
  Serial.print(" variance=");
  Serial.println(variance, 6);
}

void turnInPlace(float angleRad) {
  int sign = (angleRad > 0.0f) ? 1 : -1;
  float fullTarget = fabsf(angleRad);
  float turnScale = (sign > 0) ? turnScaleLeft : turnScaleRight;
  float target = fullTarget * turnScale;
  float accumulated = 0.0f;
  unsigned long prevUs = micros();

  while (accumulated < target) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stopDrive();
        Serial.println("Manual stop");
        return;
      }
    }

    unsigned long now = micros();
    float dt = (now - prevUs) / 1000000.0f;
    prevUs = now;
    if (dt <= 0.0f || dt > 0.05f) continue;

    accumulated += fabsf((readGyroZ_radps() - gyroBiasZ) * dt);
    float remaining = target - accumulated;

    int speed;
    if (remaining > target * 0.333f) {
      speed = turnFast;
    } else {
      float frac = remaining / (target * 0.333f);
      speed = (int)(turnSlow + frac * (float)(turnFast - turnSlow));
    }
    speed = constrain(speed, turnSlow, turnFast);
    setTurnSpeed(sign, speed);
  }

  stopDrive();
  Serial.print("Stopped at integrated angle=");
  Serial.print(accumulated * 180.0f / PI, 1);
  Serial.print(" deg, target brake angle=");
  Serial.print(target * 180.0f / PI, 1);
  Serial.print(" deg, turnScale=");
  Serial.println(turnScale, 2);
}

void printTurnScales() {
  Serial.print("turnScaleLeft=");
  Serial.print(turnScaleLeft, 2);
  Serial.print(" turnScaleRight=");
  Serial.println(turnScaleRight, 2);
}

void printHelp() {
  Serial.println();
  Serial.println("Gyro turn test");
  Serial.println("Commands:");
  Serial.println("  z      re-zero gyro bias; keep robot still");
  Serial.println("  l      left 90 deg");
  Serial.println("  r      right 90 deg");
  Serial.println("  u      180 deg");
  Serial.println("  + / -  left turnScale up/down by 0.03");
  Serial.println("  > / <  right turnScale up/down by 0.03");
  Serial.println("  s      stop during a turn");
  Serial.println("Tune turnScale: overshoot -> lower it, undershoot -> raise it.");
  printTurnScales();
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  Wire.begin();
  Wire1.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  stopDrive();

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found.");
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  printHelp();
  rezeroGyroBias();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == 'z' || c == 'Z') {
    rezeroGyroBias();
  } else if (c == 'l' || c == 'L') {
    turnInPlace(PI / 2.0f);
  } else if (c == 'r' || c == 'R') {
    turnInPlace(-PI / 2.0f);
  } else if (c == 'u' || c == 'U') {
    turnInPlace(PI);
  } else if (c == '+') {
    turnScaleLeft = constrain(turnScaleLeft + 0.03f, 0.60f, 1.20f);
    printTurnScales();
  } else if (c == '-') {
    turnScaleLeft = constrain(turnScaleLeft - 0.03f, 0.60f, 1.20f);
    printTurnScales();
  } else if (c == '>') {
    turnScaleRight = constrain(turnScaleRight + 0.03f, 0.60f, 1.20f);
    printTurnScales();
  } else if (c == '<') {
    turnScaleRight = constrain(turnScaleRight - 0.03f, 0.60f, 1.20f);
    printTurnScales();
  } else if (c == 's' || c == 'S') {
    stopDrive();
    Serial.println("Stopped");
  } else if (c == '?') {
    printHelp();
  }
}
