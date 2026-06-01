// Revival standalone tuner.
// Drives toward a stranded robot, slows inside approachDistanceCm, crawls until
// bumper contact on pin 22 or 33, then smoothly stops all motors. The LED rule
// is the competition rule: green while bumper contact is active, red otherwise.

#include <Motoron.h>
#include <Wire.h>

MotoronI2C mc1(18);  // ch2 = right drive, ch3 = left drive
MotoronI2C mc2(17);  // ch2 = right drive, ch3 = left drive

const int bumper1Pin = 22;
const int bumper2Pin = 33;
const int redPin = 46;
const int greenPin = 47;

const uint8_t TRIG_F = 37;
const uint8_t ECHO_F = 36;
const long ECHO_TIMEOUT = 25000;

int approachDistanceCm = 40;
int approachSpeed = 300;
int crawlSpeed = 150;
int currentDriveSpeed = 0;
int speedStep = 20;
unsigned long rampStepMs = 20;
unsigned long lastRampMs = 0;

bool running = false;
bool revived = false;
unsigned long lastPrintMs = 0;

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  for (int ch = 1; ch <= 3; ch++) {
    mc.setMaxAcceleration(ch, 200);
    mc.setMaxDeceleration(ch, 300);
  }
}

void setDrive(int speed) {
  mc1.setSpeed(2, speed);
  mc1.setSpeed(3, speed);
  mc2.setSpeed(2, speed);
  mc2.setSpeed(3, speed);
}

void stopMotors() {
  for (int ch = 1; ch <= 3; ch++) {
    mc1.setSpeed(ch, 0);
    mc2.setSpeed(ch, 0);
  }
  currentDriveSpeed = 0;
}

void rampDriveToward(int targetSpeed) {
  unsigned long now = millis();
  if (now - lastRampMs < rampStepMs) return;
  lastRampMs = now;

  if (currentDriveSpeed < targetSpeed) {
    currentDriveSpeed = min(currentDriveSpeed + speedStep, targetSpeed);
  } else if (currentDriveSpeed > targetSpeed) {
    currentDriveSpeed = max(currentDriveSpeed - speedStep, targetSpeed);
  }

  setDrive(currentDriveSpeed);
}

void smoothStop() {
  while (currentDriveSpeed != 0) {
    updateLEDs();
    rampDriveToward(0);
    delay(1);
  }
  stopMotors();
}

int readFrontDistanceCm() {
  digitalWrite(TRIG_F, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_F, LOW);

  long duration = pulseIn(ECHO_F, HIGH, ECHO_TIMEOUT);
  return (duration == 0) ? 999 : (int)(duration / 58L);
}

bool anyBumperPressed() {
  return digitalRead(bumper1Pin) == LOW || digitalRead(bumper2Pin) == LOW;
}

void updateLEDs() {
  bool contact = anyBumperPressed();
  digitalWrite(redPin, contact ? LOW : HIGH);
  digitalWrite(greenPin, contact ? HIGH : LOW);
}

bool serialStopRequested() {
  if (!Serial.available()) return false;
  char c = Serial.read();
  if (c == 's' || c == 'S') {
    running = false;
    smoothStop();
    Serial.println("[REVIVAL_TEST] Stopped");
    return true;
  }
  return false;
}

void printStatus() {
  Serial.print("approachDistanceCm=");
  Serial.print(approachDistanceCm);
  Serial.print(" approachSpeed=");
  Serial.print(approachSpeed);
  Serial.print(" crawlSpeed=");
  Serial.print(crawlSpeed);
  Serial.print(" rampStep=");
  Serial.print(speedStep);
  Serial.print("/");
  Serial.print(rampStepMs);
  Serial.print("ms");
  Serial.print(" currentDriveSpeed=");
  Serial.print(currentDriveSpeed);
  Serial.print(" front=");
  Serial.print(readFrontDistanceCm());
  Serial.print(" bumper1=");
  Serial.print(digitalRead(bumper1Pin) == LOW ? "PRESSED" : "released");
  Serial.print(" bumper2=");
  Serial.println(digitalRead(bumper2Pin) == LOW ? "PRESSED" : "released");
}

void runRevivalSequence() {
  running = true;
  revived = false;
  Serial.println("[REVIVAL_TEST] Starting approach");
  printStatus();

  while (running) {
    updateLEDs();
    if (serialStopRequested()) return;

    int dist = readFrontDistanceCm();
    if (dist <= approachDistanceCm) break;

    rampDriveToward(approachSpeed);

    if (millis() - lastPrintMs > 250) {
      lastPrintMs = millis();
      Serial.print("[REVIVAL_TEST] Approach front=");
      Serial.println(dist);
    }
  }

  Serial.println("[REVIVAL_TEST] Smooth deceleration to crawl speed");
  while (currentDriveSpeed != crawlSpeed) {
    updateLEDs();
    if (serialStopRequested()) return;
    rampDriveToward(crawlSpeed);
    delay(1);
  }
  Serial.println("[REVIVAL_TEST] Within approach zone, crawling to bumper contact");

  while (running) {
    updateLEDs();
    if (serialStopRequested()) return;

    if (anyBumperPressed()) {
      smoothStop();
      updateLEDs();
      revived = true;
      running = false;
      Serial.println("[REVIVAL_TEST] Bumper contact detected");
      Serial.println("[REVIVAL_TEST] All Motoron channels stopped");
      Serial.println("[REVIVAL_TEST] REVIVED - this is where reviveRequest is sent in main code");
      return;
    }

    rampDriveToward(crawlSpeed);

    if (millis() - lastPrintMs > 250) {
      lastPrintMs = millis();
      Serial.print("[REVIVAL_TEST] Crawl front=");
      Serial.println(readFrontDistanceCm());
    }
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Revival standalone test");
  Serial.println("Commands:");
  Serial.println("  g      run revival approach/crawl sequence");
  Serial.println("  s      stop immediately");
  Serial.println("  p      print distance, bumper, speed status");
  Serial.println("  + / -  approach distance up/down by 5 cm");
  Serial.println("  ] / [  crawl speed up/down by 25");
  Serial.println("  } / {  approach speed up/down by 25");
  Serial.println("  > / <  ramp step up/down by 5 speed units");
  Serial.println("  ?      print help");
  Serial.println();
  Serial.println("Expected behavior:");
  Serial.println("  Red LED on when bumpers are released.");
  Serial.println("  Green LED on while bumper 22 or 33 is pressed.");
  Serial.println("  Robot drives fast until front distance <= threshold, then crawls at 200.");
  Serial.println("  It ramps speed changes smoothly instead of instant jumps.");
  Serial.println("  On bumper contact, it smoothly stops all Motoron channels and stays stopped.");
  Serial.println();
  printStatus();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  Wire1.begin();
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  stopMotors();

  pinMode(bumper1Pin, INPUT_PULLUP);
  pinMode(bumper2Pin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  updateLEDs();
  printHelp();
}

void loop() {
  updateLEDs();
  if (revived) stopMotors();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      runRevivalSequence();
    } else if (c == 's' || c == 'S') {
      running = false;
      smoothStop();
      Serial.println("[REVIVAL_TEST] Stopped");
    } else if (c == 'p' || c == 'P') {
      printStatus();
    } else if (c == '+') {
      approachDistanceCm += 5;
      printStatus();
    } else if (c == '-') {
      approachDistanceCm = max(5, approachDistanceCm - 5);
      printStatus();
    } else if (c == ']') {
      crawlSpeed = constrain(crawlSpeed + 25, 0, 660);
      printStatus();
    } else if (c == '[') {
      crawlSpeed = constrain(crawlSpeed - 25, 0, 660);
      printStatus();
    } else if (c == '}') {
      approachSpeed = constrain(approachSpeed + 25, 0, 660);
      printStatus();
    } else if (c == '{') {
      approachSpeed = constrain(approachSpeed - 25, 0, 660);
      printStatus();
    } else if (c == '>') {
      speedStep = constrain(speedStep + 5, 5, 100);
      printStatus();
    } else if (c == '<') {
      speedStep = constrain(speedStep - 5, 5, 100);
      printStatus();
    } else if (c == '?') {
      printHelp();
    }
  }
}
