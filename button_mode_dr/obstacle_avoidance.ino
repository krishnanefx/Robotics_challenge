// ══════════════════════════════════════════════════════════════════════════════
//  obstacle_avoidance.ino — 3-point box swerve around a single obstacle
//
//  Public API:
//    void runObstacleMode()   — mode 5 entry point; called from loop()
//
//  Internal:
//    bool avoidObstacle()     — full 8-step swerve; returns false on timeout
//    driveForwardDist()       — front-sensor odometer drive
//    obsTurnInPlace()         — encoder-based pivot (separate from gyro DR turns)
//
//  Uses TRACK_WIDTH_CM (17.0), WHEEL_DIAM_CM (6.5), and COUNTS_PER_REV (144)
//  from button_mode_dr.ino — all turn geometry is consistent with dead reckoning.
// ══════════════════════════════════════════════════════════════════════════════


// ── Obstacle-mode motion constants ────────────────────────────────────────────
// TRACK_WIDTH_CM and COUNTS_PER_REV come from button_mode_dr.ino (17.0 / 144).
// WHEEL_DIAM_CM also comes from button_mode_dr.ino (6.5).
const int16_t OBS_TURN_FAST  = 400;
const int16_t OBS_TURN_SLOW  = 150;
const int16_t OBS_DRIVE_SPEED = 500;

const int      OBSTACLE_DETECT_DIST = 25;    // cm — trigger swerve when front <= this
const uint32_t AVOID_TIMEOUT        = 60000UL; // ms — abort after 60 s

static uint32_t avoidStart = 0;

// Sensor-index aliases (local to this file's logic)
enum { OBS_FRONT = 0, OBS_LEFT = 1, OBS_RIGHT = 2 };
static int obsDist[3];  // populated by checkAllDirections()


// ── CHECK_TIMEOUT macro ───────────────────────────────────────────────────────
// Drops out of avoidObstacle() and stops motors if the 60 s budget is exceeded.
#define CHECK_TIMEOUT() \
  if (millis() - avoidStart > AVOID_TIMEOUT) { \
    stopMotors(); Serial.println("[OBS] !! 60 s timeout"); return false; }


// ── Read all three sensors into obsDist[] ─────────────────────────────────────
static void checkAllDirections() {
  obsDist[OBS_FRONT] = readDistance(TRIG_F, ECHO_F);
  delay(15);
  obsDist[OBS_LEFT]  = readDistance(TRIG_L, ECHO_L);
  delay(15);
  obsDist[OBS_RIGHT] = readDistance(TRIG_R, ECHO_R);

  Serial.println("--- Sensor Readings ---");
  Serial.print("  FRONT: "); Serial.print(obsDist[OBS_FRONT]); Serial.println(" cm");
  Serial.print("  LEFT:  "); Serial.print(obsDist[OBS_LEFT]);  Serial.println(" cm");
  Serial.print("  RIGHT: "); Serial.print(obsDist[OBS_RIGHT]); Serial.println(" cm");
}


// ── Encoder-based pivot (obstacle mode only) ──────────────────────────────────
// Uses OBS_* constants — kept separate from the gyro-based turnInPlace() in
// motion.ino that is used for dead reckoning.
// +PI/2 = CCW (left),  -PI/2 = CW (right)

static void obsTurnInPlace(float angleRad) {
  long target = (long)(fabsf(angleRad) * TRACK_WIDTH_CM / 2.0f
                       / (PI * WHEEL_DIAM_CM) * COUNTS_PER_REV);
  int  sign   = (angleRad > 0.0f) ? 1 : -1;

  long startL = encoderCount1;
  long startR = encoderCount2;

  while (true) {
    long avg = (abs(encoderCount1 - startL) + abs(encoderCount2 - startR)) / 2;
    long err = target - avg;
    if (err <= 0) break;

    int spd = (err > target / 3)
              ? OBS_TURN_FAST
              : (int)map(err, 0L, target / 3, (long)OBS_TURN_SLOW, (long)OBS_TURN_FAST);
    spd = constrain(spd, OBS_TURN_SLOW, OBS_TURN_FAST);

    mc1.setSpeed(2,  sign * spd);  mc1.setSpeed(3, -sign * spd);
    mc2.setSpeed(2,  sign * spd);  mc2.setSpeed(3, -sign * spd);
  }
  stopMotors();
  delay(150);   // settle before next move
}

static void obsLeft90()  { obsTurnInPlace( PI / 2.0f); }
static void obsRight90() { obsTurnInPlace(-PI / 2.0f); }


// ── Drive forward until front sensor shortens by targetCm ────────────────────
// Falls back to a timed drive if the front sensor reads open space (>=990 cm).

static void driveForwardDist(int targetCm, unsigned long safetyMs = 5000) {
  int startDist = readDistance(TRIG_F, ECHO_F);

  if (startDist >= 990) {
    // No wall ahead — drive blind for safetyMs
    unsigned long t0 = millis();
    while (millis() - t0 < safetyMs) {
      mc1.setSpeed(2, OBS_DRIVE_SPEED); mc1.setSpeed(3, OBS_DRIVE_SPEED);
      mc2.setSpeed(2, OBS_DRIVE_SPEED); mc2.setSpeed(3, OBS_DRIVE_SPEED);
      delay(20);
    }
    stopMotors();
    return;
  }

  int targetReading = startDist - targetCm;
  unsigned long t0 = millis();
  while (true) {
    if (readDistance(TRIG_F, ECHO_F) <= targetReading) break;
    if (millis() - t0 > safetyMs)                      break;
    mc1.setSpeed(2, OBS_DRIVE_SPEED); mc1.setSpeed(3, OBS_DRIVE_SPEED);
    mc2.setSpeed(2, OBS_DRIVE_SPEED); mc2.setSpeed(3, OBS_DRIVE_SPEED);
    delay(20);
  }
  stopMotors();
}


// ── 3-Point Box Swerve ────────────────────────────────────────────────────────
//
//  Top-down path (goLeft example):
//
//   start → [obsLeft90] → 25cm → [obsRight90] → 50cm → [obsRight90]
//         → 25cm → [obsLeft90] → 25cm → done
//
//  Net displacement: 3 nodes forward on original heading, shifted 0 laterally.
//
//  Returns false if the 60 s budget is exhausted mid-swerve.

bool avoidObstacle() {
  avoidStart = millis();
  stopMotors();
  delay(150);
  Serial.println("--- 3-Point Swerve START ---");

  checkAllDirections();
  CHECK_TIMEOUT();

  bool goLeft = (obsDist[OBS_LEFT] >= obsDist[OBS_RIGHT]);
  Serial.println(goLeft ? "[OBS] Swerving LEFT" : "[OBS] Swerving RIGHT");

  // Step 1: turn 90° away from obstacle lane
  if (goLeft) obsLeft90();  else obsRight90();
  CHECK_TIMEOUT();

  // Step 2: move 1 node sideways (clear the obstacle lane)
  driveForwardDist(25, 3000);
  CHECK_TIMEOUT();

  // Step 3: turn back to face original forward direction
  if (goLeft) obsRight90(); else obsLeft90();
  CHECK_TIMEOUT();

  // Step 4: drive 2 nodes forward (past the obstacle)
  driveForwardDist(50, 5000);
  CHECK_TIMEOUT();

  // Step 5: turn 90° toward original lane
  if (goLeft) obsRight90(); else obsLeft90();
  CHECK_TIMEOUT();

  // Step 6: move 1 node sideways (back onto original path)
  driveForwardDist(25, 3000);
  CHECK_TIMEOUT();

  // Step 7: turn to face forward again
  if (goLeft) obsLeft90();  else obsRight90();
  CHECK_TIMEOUT();

  // Step 8: advance 1 final node (total = 3 nodes from start)
  driveForwardDist(25, 3000);

  stopMotors();
  Serial.println("--- 3-Point Swerve COMPLETE ---");
  return true;
}


// ── runObstacleMode() — mode 5 entry point ───────────────────────────────────
// Called repeatedly from loop() while presses == 5.
// Drives forward continuously; swerves when front obstacle is within
// OBSTACLE_DETECT_DIST cm.  Also seeds on RFID and resumes.

void runObstacleMode() {
  int frontDist = readDistance(TRIG_F, ECHO_F);
  Serial.print("[OBS] Front: "); Serial.print(frontDist); Serial.println(" cm");

  if (frontDist <= OBSTACLE_DETECT_DIST) {
    Serial.println("[OBS] Obstacle — executing swerve");
    avoidObstacle();
    return;
  }

  // RFID waypoint — seed and continue
  if (rfid != nullptr && rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
    stopMotors();
    dropSeed();
    rfid->PICC_HaltA();
    return;
  }

  // Baseline forward drive
  mc1.setSpeed(2, OBS_DRIVE_SPEED); mc1.setSpeed(3, OBS_DRIVE_SPEED);
  mc2.setSpeed(2, OBS_DRIVE_SPEED); mc2.setSpeed(3, OBS_DRIVE_SPEED);
  delay(30);
}
