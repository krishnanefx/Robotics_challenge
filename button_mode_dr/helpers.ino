// ══════════════════════════════════════════════════════════════════════════════
//  helpers.ino — Hardware utilities: RFID scan, Motoron init,
//                ultrasonic, seed dispenser, motor stop, buttons
// ══════════════════════════════════════════════════════════════════════════════


// ── RFID I2C scanner ──────────────────────────────────────────────────────────

byte findI2CAddress() {
  byte foundAddress = 0, foundCount = 0;
  Serial.println("Scanning Wire2 I2C bus...");
  for (byte address = 1; address < 127; address++) {
    RFID_WIRE.beginTransmission(address);
    if (RFID_WIRE.endTransmission() == 0) {
      if (foundAddress == 0) foundAddress = address;
      foundCount++;
      Serial.print("Found I2C device at 0x");
      if (address < 0x10) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (foundCount == 0) { Serial.println("ERROR: No I2C devices on Wire2."); return 0; }
  if (foundCount > 1)    Serial.println("WARNING: Multiple devices, using first.");
  return foundAddress;
}

// Format the RFID UID currently in rfid->uid as an 8-char hex string.
void readTagUID(char* buf, size_t bufLen) {
  snprintf(buf, bufLen, "%02X%02X%02X%02X",
           rfid->uid.uidByte[0], rfid->uid.uidByte[1],
           rfid->uid.uidByte[2], rfid->uid.uidByte[3]);
}


// ── Motoron initialisation ────────────────────────────────────────────────────

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
}


// ── Ultrasonic distance (cm) ──────────────────────────────────────────────────

int readDistance(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, ECHO_TIMEOUT);
  return (dur == 0) ? 999 : (int)(dur / 58L);
}


// ── Seed dispenser ────────────────────────────────────────────────────────────

void dropSeed() {
  Serial.println("[SEED] Dispensing...");
  mc2.setSpeed(1, 600);
  delay(700);
  mc2.setSpeed(1, 0);
  Serial.println("[SEED] Done");
}


// ── Motor stop ────────────────────────────────────────────────────────────────

void stopMotors() {
  mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
  mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);
}


// ── Buttons + kill switch (non-blocking, debounced) ───────────────────────────
//
//  Kill switch (45) LOW → robotRunning = false  (hardware stop, any mode)
//  Button 1 or 2  LOW → when stopped: robotRunning = true  (revival / start)
//                      → when running (btn1 only): presses++  (mode advance)
//                        (guarded in revival mode so bumper contact doesn't
//                         accidentally skip a mode)

static bool          lb1 = HIGH, lb2 = HIGH, lkill = HIGH;
static unsigned long lb1t = 0,   lb2t = 0,   lkillt = 0;

void checkButton() {
  unsigned long now = millis();
  bool b1   = digitalRead(button1Pin);
  bool b2   = digitalRead(button2Pin);
  bool kill = digitalRead(killSwitchPin);

  // ── Kill switch ──
  if (lkill == HIGH && kill == LOW && now - lkillt > DEBOUNCE_MS) {
    lkillt = now;
    robotRunning = false;
    stopMotors();
    Serial.println("[BTN] Kill switch — stopped");
  }
  lkill = kill;

  // ── Button 1 ──
  if (lb1 == HIGH && b1 == LOW && now - lb1t > DEBOUNCE_MS) {
    lb1t = now;
    if (!robotRunning) {
      robotRunning = true;
      Serial.println("[BTN] Button 1 — robot started");
    } else if (presses != 4) {   // don't advance mode on bumper contact
      presses++;
      Serial.print("[BTN] Mode → "); Serial.println(presses);
    }
  }
  lb1 = b1;

  // ── Button 2 ──
  if (lb2 == HIGH && b2 == LOW && now - lb2t > DEBOUNCE_MS) {
    lb2t = now;
    if (!robotRunning) {
      robotRunning = true;
      Serial.println("[BTN] Button 2 — robot started");
    }
  }
  lb2 = b2;
}

// Returns true if either bumper button is currently pressed.
// Used by revival mode (mode 4) to detect physical contact with target robot.
bool anyBumperPressed() {
  return (digitalRead(button1Pin) == LOW || digitalRead(button2Pin) == LOW);
}
