// Encoder counts per node tuner.
// Drives the robot while printing left/right encoder counts so the 25 cm node
// distance can be verified. Use this when wheel diameter, battery, or surface
// changes make dead reckoning overshoot or undershoot.

#include <Motoron.h>
#include <Wire.h>
#include <math.h>

MotoronI2C mc1(18);  // ch2 = right drive, ch3 = left drive
MotoronI2C mc2(17);  // ch2 = right drive, ch3 = left drive

const int encoder1PinA = 52;
const int encoder1PinB = 50;
const int encoder2PinA = 53;
const int encoder2PinB = 51;

const float WHEEL_DIAM_CM = 6.5f;
const float COUNTS_PER_REV = 144.0f;
const float NODE_CM = 25.0f;
const float CM_PER_COUNT = (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;
const long COUNTS_PER_NODE = (long)(NODE_CM / CM_PER_COUNT + 0.5f);

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
int driveSpeed = 500;

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

void setDrive(int left, int right) {
  mc1.setSpeed(2, right);
  mc1.setSpeed(3, left);
  mc2.setSpeed(2, right);
  mc2.setSpeed(3, left);
}

void stopDrive() {
  setDrive(0, 0);
}

void zeroEncoders() {
  noInterrupts();
  encoderCount1 = 0;
  encoderCount2 = 0;
  interrupts();
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
  Serial.println(avg * CM_PER_COUNT, 2);
}

void driveOneNode() {
  zeroEncoders();
  Serial.print("Driving to target counts: ");
  Serial.println(COUNTS_PER_NODE);

  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stopDrive();
        Serial.println("Manual stop");
        printCounts();
        return;
      }
    }

    long avg = (abs(encoderCount1) + abs(encoderCount2)) / 2;
    if (avg >= COUNTS_PER_NODE) break;

    setDrive(driveSpeed, driveSpeed);
  }

  stopDrive();
  printCounts();
}

void printHelp() {
  Serial.println();
  Serial.println("Encoder counts per 25 cm node test");
  Serial.println("Commands:");
  Serial.println("  f      drive one 25 cm node by encoder count");
  Serial.println("  p      print counts");
  Serial.println("  z      zero counts");
  Serial.println("  + / -  speed up/down");
  Serial.println("  s      stop during a run");
  Serial.print("Computed COUNTS_PER_NODE=");
  Serial.println(COUNTS_PER_NODE);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  Wire1.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  stopDrive();

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), countEncoder2, RISING);

  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == 'f' || c == 'F') {
    driveOneNode();
  } else if (c == 'p' || c == 'P') {
    printCounts();
  } else if (c == 'z' || c == 'Z') {
    zeroEncoders();
    Serial.println("Counts zeroed");
  } else if (c == '+') {
    driveSpeed = constrain(driveSpeed + 25, 0, 800);
    Serial.print("driveSpeed=");
    Serial.println(driveSpeed);
  } else if (c == '-') {
    driveSpeed = constrain(driveSpeed - 25, 0, 800);
    Serial.print("driveSpeed=");
    Serial.println(driveSpeed);
  } else if (c == 's' || c == 'S') {
    stopDrive();
    Serial.println("Stopped");
  } else if (c == '?') {
    printHelp();
  }
}
