// Minimum usable motor speed tuner.
// Finds the lowest speed that reliably starts each drive direction from rest.
// Current tuned values from testing: forward/backward around 200, pivot turns
// around 400. Use the results as MIN_FORWARD_SPEED and MIN_TURN_SPEED.

#include <Motoron.h>
#include <Wire.h>

MotoronI2C mc1(18);  // ch2 = right drive, ch3 = left drive
MotoronI2C mc2(17);  // ch2 = right drive, ch3 = left drive

int testSpeed = 300;
int stepSize = 25;

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

void printHelp() {
  Serial.println();
  Serial.println("Minimum usable motor speed test");
  Serial.println("Commands:");
  Serial.println("  + / -  speed up/down");
  Serial.println("  f      drive both sides forward");
  Serial.println("  b      drive both sides backward");
  Serial.println("  l      left side only");
  Serial.println("  r      right side only");
  Serial.println("  s      stop");
  Serial.println("Tune MIN_SPEED to the lowest speed that starts reliably every time.");
  Serial.println();
}

void printStatus() {
  Serial.print("testSpeed=");
  Serial.print(testSpeed);
  Serial.print(" step=");
  Serial.println(stepSize);
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

  printHelp();
  printStatus();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == '+') {
    testSpeed = constrain(testSpeed + stepSize, 0, 800);
    printStatus();
  } else if (c == '-') {
    testSpeed = constrain(testSpeed - stepSize, 0, 800);
    printStatus();
  } else if (c == 'f' || c == 'F') {
    setDrive(testSpeed, testSpeed);
    Serial.println("Forward");
  } else if (c == 'b' || c == 'B') {
    setDrive(-testSpeed, -testSpeed);
    Serial.println("Backward");
  } else if (c == 'l' || c == 'L') {
    setDrive(testSpeed, 0);
    Serial.println("Left side only");
  } else if (c == 'r' || c == 'R') {
    setDrive(0, testSpeed);
    Serial.println("Right side only");
  } else if (c == 's' || c == 'S') {
    stopDrive();
    Serial.println("Stopped");
  } else if (c == '?') {
    printHelp();
  }
}
