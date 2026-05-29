// ══════════════════════════════════════════════════════════════════════════════
//  line_following.ino — QTR line following, intersection turns, arena nav
//
//  Public API:
//    bool  onLine()              — true if any sensor reads the line
//    void  turnLeftQTR()         — CCW 90° using QTR line detection
//    void  turnRightQTR()        — CW  90° using QTR line detection
//    void  faceNorth/S/E/W()     — absolute cardinal turn (QTR, arena only)
//    void  followLine()          — proportional follow; on RFID: updates grid
//                                  position, faces next waypoint, skips tag
//    bool  goToPointB()          — pre-programmed turn sequence to base door;
//                                  returns false when front sensor sees door
//
//  Note: faceNorth/S/E/W use QTR detection — call only when on the arena floor.
//        For encoder-based cardinal turns (tunnel approach), call faceDir() in
//        wall_following.ino directly.
// ══════════════════════════════════════════════════════════════════════════════


// ── Are any sensors reading the line? ────────────────────────────────────────

bool onLine() {
  qtr.readCalibrated(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 100) return true;
  }
  return false;
}


// ── QTR-based 90° pivot turns ─────────────────────────────────────────────────
// Phase 1: spin until the line is lost.
// Phase 2: keep spinning until the line is found again.
// Stops on the new line and updates the direction index.
// direction: 0=W  1=N  2=E  3=S

void turnLeftQTR() {
  while (onLine()) {
    mc1.setSpeed(2,  600); mc1.setSpeed(3, -600);
    mc2.setSpeed(2,  600); mc2.setSpeed(3, -600);
  }
  while (!onLine()) {
    mc1.setSpeed(2,  600); mc1.setSpeed(3, -600);
    mc2.setSpeed(2,  600); mc2.setSpeed(3, -600);
  }
  stopMotors();
  direction = (direction - 1 + 4) % 4;  // CCW
}

void turnRightQTR() {
  while (onLine()) {
    mc1.setSpeed(2, -600); mc1.setSpeed(3,  600);
    mc2.setSpeed(2, -600); mc2.setSpeed(3,  600);
  }
  while (!onLine()) {
    mc1.setSpeed(2, -600); mc1.setSpeed(3,  600);
    mc2.setSpeed(2, -600); mc2.setSpeed(3,  600);
  }
  stopMotors();
  direction = (direction + 1) % 4;       // CW
}


// ── Cardinal facing helpers (QTR, arena only) ─────────────────────────────────
// Handles all 4 starting directions, including 180° flip.

static void faceDirQTR(int target) {
  int diff = (target - direction + 4) % 4;
  if      (diff == 1) turnRightQTR();
  else if (diff == 3) turnLeftQTR();
  else if (diff == 2) { turnRightQTR(); turnRightQTR(); }
  // diff == 0: already facing
}

void faceNorth() { faceDirQTR(1); }
void faceSouth() { faceDirQTR(3); }
void faceEast()  { faceDirQTR(2); }
void faceWest()  { faceDirQTR(0); }


// ── Proportional line follow ───────────────────────────────────────────────────
// On RFID detection (inArena=true):
//   1. Stops briefly (3 s).
//   2. Updates grid position (x, y) from current heading.
//   3. Advances step and turns to face next waypoint.
//   4. Drives for 200 iterations to clear the tag before resuming normal follow.

void followLine() {
  if (rfid != nullptr && rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
    stopMotors();
    delay(3000);

    if (inArena) {
      // Update grid position from current heading
      if      (direction == 0) x -= 1;
      else if (direction == 1) y += 1;
      else if (direction == 2) x += 1;
      else if (direction == 3) y -= 1;

      // Advance step and face the next waypoint
      if (step < 4) {
        step++;
        int dx = pathx[step] - x;
        int dy = pathy[step] - y;
        if      (dx ==  1) faceEast();
        else if (dx == -1) faceWest();
        else if (dy ==  1) faceNorth();
        else if (dy == -1) faceSouth();
      }

      // Seed drop at this node if fertile
      if (step > 0 && step <= 5 && fertiles[step - 1]) {
        dropSeed();
        fertiles[step - 1] = false;
      }
    }

    // Drive 200 iterations to physically clear the RFID tag
    for (int n = 0; n < 200; n++) {
      qtr.readCalibrated(sensorValues);
      float err = 0;
      for (uint8_t i = 0; i < SensorCount; i++) err += sensorValues[i] * weights[i];
      int e = (int)err;
      mc1.setSpeed(2, (int)((400 + e) * 0.8f));
      mc1.setSpeed(3, (int)((400 - e) * 0.8f));
      mc2.setSpeed(2, (int)((400 + e) * 0.8f));
      mc2.setSpeed(3, (int)((400 - e) * 0.8f));
    }

    rfid->PICC_HaltA();
    return;
  }

  // Normal proportional follow
  qtr.readCalibrated(sensorValues);
  float err = 0;
  for (uint8_t i = 0; i < SensorCount; i++) err += sensorValues[i] * weights[i];
  int e = (int)err;
  mc1.setSpeed(2, (int)((400 + e) * 0.8f));
  mc1.setSpeed(3, (int)((400 - e) * 0.8f));
  mc2.setSpeed(2, (int)((400 + e) * 0.8f));
  mc2.setSpeed(3, (int)((400 - e) * 0.8f));
}


// ── Navigate to Point B (base exit door) ─────────────────────────────────────
// Executes a pre-programmed turn sequence, line-following between turns.
// Stops and returns false when the front sensor sees the door (< 8 cm).

bool goToPointB() {
  // Brief pause at any RFID tag
  if (rfid != nullptr && rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
    Serial.println("[GoB] RFID — pausing");
    stopMotors();
    rfid->PICC_HaltA();
    delay(5000);
  }

  // Stop at door
  if (readDistance(TRIG_F, ECHO_F) < 8) {
    stopMotors();
    return false;
  }

  // Turn sequence or line follow
  if (onLine()) {
    followLine();
  } else {
    if      (turns == 0) { turnRightQTR(); turns++; }
    else if (turns == 1) { turnLeftQTR();  turns++; }
    else if (turns == 2) { turnLeftQTR();  turns++; }
    else                 { turnRightQTR(); }
  }
  return true;
}
