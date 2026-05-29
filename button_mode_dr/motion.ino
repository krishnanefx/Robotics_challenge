// ══════════════════════════════════════════════════════════════════════════════
//  motion.ino — Dead-reckoning motion stack
//
//  All globals (mc1, mc2, mpu, encoderCount1/2, fusedHeading, etc.) are
//  declared in button_mode_dr.ino and visible here automatically.
//
//  Public API:
//    void countEncoder1/2()      — ISRs, attached in setup()
//    void rezeroGyroBias()       — recalibrate Z bias while stationary (~1.2 s)
//    void turnLeft/Right()       — closed-loop CCW / CW 90° turns
//    bool driveToNode()          — drive 25 cm straight, returns true if RFID stopped it
//    void driveOneNode(int idx)  — drive + RFID read + conditional seed + bias re-zero
//    void runDeadReckoning()     — execute full path; runs once then idles
// ══════════════════════════════════════════════════════════════════════════════


// ── Encoder ISRs ──────────────────────────────────────────────────────────────

void countEncoder1() {
  if (digitalRead(encoder1PinB) == HIGH) encoderCount1++;
  else                                    encoderCount1--;
}

void countEncoder2() {
  if (digitalRead(encoder2PinB) == LOW)  encoderCount2++;
  else                                    encoderCount2--;
}


// ── Internal: raw gyro Z in rad/s ─────────────────────────────────────────────

float readGyroZ_radps() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return g.gyro.z;
}


// ── Complementary-filter heading update ──────────────────────────────────────
// Blends gyro integration (98%) with encoder-differential angle (2%).
// fusedHeading: +ve = CCW/left, -ve = CW/right, in radians.
// Call on every iteration of the drive loop (target ≥ 200 Hz).

void updateHeading() {
  unsigned long now = micros();
  float dt = (now - lastHeadingUs) / 1e6f;
  lastHeadingUs = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  float gyroAngle = (readGyroZ_radps() - gyroBiasZ) * dt;

  long curL = encoderCount1, curR = encoderCount2;
  float distL = (float)(curL - prevEncL) * CM_PER_COUNT;
  float distR = (float)(curR - prevEncR) * CM_PER_COUNT;
  prevEncL = curL;
  prevEncR = curR;
  float encAngle = (distR - distL) / TRACK_WIDTH_CM;

  fusedHeading += ALPHA * gyroAngle + (1.0f - ALPHA) * encAngle;
}


// ── Gyro bias calibration ─────────────────────────────────────────────────────
// Sample 500 readings over ~1 s. Skips update if robot is still vibrating.

void rezeroGyroBias() {
  delay(200);
  const int N = 500;
  float sum = 0.0f, sumSq = 0.0f;
  for (int i = 0; i < N; i++) {
    float z = readGyroZ_radps();
    sum   += z;
    sumSq += z * z;
    delay(2);
  }
  float mean     = sum / N;
  float variance = (sumSq / N) - (mean * mean);
  if (variance > (0.05f * 0.05f)) {
    Serial.print("[GYRO] Re-zero skipped — variance: ");
    Serial.println(variance, 5);
    return;
  }
  gyroBiasZ = mean;
  Serial.print("[GYRO] Bias: "); Serial.println(gyroBiasZ, 5);
}


// ── Closed-loop gyro turn ─────────────────────────────────────────────────────
// angleRad: +PI/2 = CCW (left), -PI/2 = CW (right).
// Integrates the gyro Z-axis; stops motors at TURN_SCALE × target so coasting
// carries the robot the rest of the way to 90°.
//
// Tuning:  overshoot → decrease TURN_SCALE (e.g. 0.87)
//          undershoot → increase TURN_SCALE (e.g. 0.93)

void turnInPlace(float angleRad) {
  int   sign        = (angleRad > 0.0f) ? 1 : -1;
  float target      = fabsf(angleRad) * TURN_SCALE;  // scaled brake point
  float accumulated = 0.0f;
  unsigned long prevUs = micros();

  while (accumulated < target) {
    unsigned long now = micros();
    float dt = (now - prevUs) / 1e6f;
    prevUs = now;
    if (dt <= 0.0f || dt > 0.05f) continue;

    accumulated += fabsf((readGyroZ_radps() - gyroBiasZ) * dt);

    float remaining = target - accumulated;
    int spd;
    if (remaining > target * 0.333f) {
      spd = TURN_FAST;
    } else {
      // Linear taper: TURN_FAST → TURN_SLOW as remaining approaches 0
      float frac = remaining / (target * 0.333f);        // 1.0 → 0.0
      spd = (int)(TURN_SLOW + frac * (float)(TURN_FAST - TURN_SLOW));
    }
    spd = constrain(spd, TURN_SLOW, TURN_FAST);

    mc1.setSpeed(2,  sign * spd);  mc1.setSpeed(3, -sign * spd);
    mc2.setSpeed(2,  sign * spd);  mc2.setSpeed(3, -sign * spd);
  }

  mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
  mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);

  direction = (sign > 0) ? (direction - 1 + 4) % 4 : (direction + 1) % 4;
  Serial.print("[TURN] Facing "); Serial.println(dirNames[direction]);
  rezeroGyroBias();
}

void turnLeft()  { turnInPlace( PI / 2.0f); }
void turnRight() { turnInPlace(-PI / 2.0f); }


// ── Drive straight to the next 25 cm node ─────────────────────────────────────
// Returns true if an RFID tag stopped the run early.

bool driveToNode() {
  encoderCount1 = 0;  encoderCount2 = 0;
  fusedHeading  = 0.0f;
  prevEncL = 0;       prevEncR = 0;
  lastHeadingUs = micros();

  while (true) {
    updateHeading();

    long avg = (abs(encoderCount1) + abs(encoderCount2)) / 2;
    if (avg >= COUNTS_PER_NODE) {
      Serial.print("[DR] Node by odometry (L=");
      Serial.print(encoderCount1); Serial.print(" R=");
      Serial.print(encoderCount2); Serial.println(")");
      break;
    }

    if (rfid != nullptr && rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
      Serial.println("[DR] Node confirmed by RFID");
      rfid->PICC_HaltA();
      mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
      mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);
      return true;
    }

    int correction = constrain((int)(DRIVE_KP * fusedHeading), -200, 200);
    mc1.setSpeed(2, constrain(DRIVE_SPEED - correction, 0, 800));
    mc1.setSpeed(3, constrain(DRIVE_SPEED + correction, 0, 800));
    mc2.setSpeed(2, constrain(DRIVE_SPEED - correction, 0, 800));
    mc2.setSpeed(3, constrain(DRIVE_SPEED + correction, 0, 800));
  }

  mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
  mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);
  return false;
}


// ── Drive to one node, log RFID, seed if fertile, re-zero bias ────────────────

void driveOneNode(int nodeIdx) {
  Serial.print("[DR] >>> Node "); Serial.println(nodeIdx);
  driveToNode();

  if (rfid != nullptr && rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
    char tagId[12];
    snprintf(tagId, sizeof(tagId), "%02X%02X%02X%02X",
             rfid->uid.uidByte[0], rfid->uid.uidByte[1],
             rfid->uid.uidByte[2], rfid->uid.uidByte[3]);
    Serial.print("[RFID] Node "); Serial.print(nodeIdx);
    Serial.print(" → "); Serial.println(tagId);
    rfid->PICC_HaltA();
  }

  if (nodeIdx >= 0 && nodeIdx < 5 && fertiles[nodeIdx]) {
    dropSeed();
  }

  rezeroGyroBias();
}


// ── Navigate the full pathx/pathy array using dead reckoning ──────────────────
// Turns to face each waypoint's required direction, drives one node, seeds if
// fertile. Runs once; subsequent calls while presses==3 are no-ops.

void runDeadReckoning() {
  static bool drDone = false;
  if (drDone) return;

  Serial.println("[DR] Starting dead-reckoning sequence");
  rezeroGyroBias();

  if (fertiles[0]) dropSeed();

  for (int i = 1; i < 5; i++) {
    int dx = pathx[i] - pathx[i - 1];
    int dy = pathy[i] - pathy[i - 1];

    int targetDir;
    if      (dy > 0) targetDir = 1;
    else if (dy < 0) targetDir = 3;
    else if (dx > 0) targetDir = 2;
    else             targetDir = 0;

    // diff==1: CW (right)  diff==3: CCW (left)  diff==2: 180°
    int diff = (targetDir - direction + 4) % 4;
    if      (diff == 1) turnRight();
    else if (diff == 3) turnLeft();
    else if (diff == 2) { turnRight(); turnRight(); }

    driveOneNode(i);
  }

  mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
  mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);
  Serial.println("[DR] Sequence complete");
  drDone = true;
}
