// ══════════════════════════════════════════════════════════════════════════════
//  wall_following.ino — Tunnel/ramp traversal + Airlock A state machine
//
//  Public API:
//    void runChainMode()   — mode 1 entry point; handles the full
//                            NAVIGATING → WAITING_AIRLOCK → LINE_FOLLOWING chain
//
//  Internal:
//    float getTilt()       — complementary-filter pitch angle (deg)
//    bool  navigateTunnel()— dual-wall PD + tilt detection; returns true while in tunnel
//    bool  waitForDoor()   — blocks while front wall < 8 cm (door closed)
//    void  faceNorth/South/East/West() — turn to an absolute cardinal heading
// ══════════════════════════════════════════════════════════════════════════════


// ── Wall / tunnel constants ───────────────────────────────────────────────────
const float WK_P              = 80.0f;  // P gain going uphill
const float WK_P_DOWN         = 20.0f;  // P gain downhill — gentler to avoid over-steer
const float WK_D              = 6.0f;
const int   W_BASE            = 700;
const float TILT_THRESHOLD    = 5.0f;   // |pitch| above this → treat as on ramp
const float DOWNHILL_THRESHOLD = -20.0f;// pitch below this → going downhill
const float COMP_ALPHA        = 0.90f;  // complementary filter weight (gyro vs accel)
const int   DOWNHILL_REDUCTION = 650;   // speed reduction while descending
const int   TUNNEL_WALL_DIST  = 20;     // cm — walls this close also counts as tunnel


// ── Airlock A RFID tag UID ────────────────────────────────────────────────────
// Scan the tag once, check Serial for the printed UID, paste it here.
const String AIRLOCK_A_TAG_ID = "XXXXXXXX";   // ← replace with actual UID


// ── Runtime state ─────────────────────────────────────────────────────────────
static int   wfLastError        = 0;
static bool  wasInTunnel        = false;
static bool  exitingThroughDoor = false;
static bool  hadRamp            = false;
static float pitch              = 0.0f;
static unsigned long lastTiltUs = 0;


// ── getTilt ───────────────────────────────────────────────────────────────────
// Complementary filter: blends gyro integration with accelerometer-derived pitch.
// Returns pitch in degrees — positive = nose-up, negative = nose-down.

float getTilt() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (lastTiltUs == 0) ? 0.0f : (now - lastTiltUs) / 1e6f;
  lastTiltUs = now;

  // Accelerometer pitch around each axis (for debugging orientation)
  float pitchX = atan2(a.acceleration.x,
                       sqrt(a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z))
                 * 180.0f / PI;
  float pitchY = atan2(a.acceleration.y,
                       sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.z * a.acceleration.z))
                 * 180.0f / PI;
  float pitchZ = atan2(a.acceleration.z,
                       sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y))
                 * 180.0f / PI;

  Serial.print("[TILT] X="); Serial.print(pitchX);
  Serial.print("  Y=");       Serial.print(pitchY);
  Serial.print("  Z=");       Serial.print(pitchZ);
  Serial.println(" deg");

  // pitchY = forward/backward tilt — use as the accel reference
  pitch = COMP_ALPHA * (pitch + g.gyro.x * dt * 180.0f / PI)
        + (1.0f - COMP_ALPHA) * pitchY;

  return pitch;
}


// ── waitForDoor ───────────────────────────────────────────────────────────────
// Stops and polls until the front obstacle clears 8 cm (door opens).
// Returns true if it actually had to wait (i.e. a door was in the way).

bool waitForDoor() {
  if (readDistance(TRIG_F, ECHO_F) >= 8) return false;
  stopMotors();
  Serial.println("[TUNNEL] Door detected — waiting");
  while (readDistance(TRIG_F, ECHO_F) < 8) {
    messenger.loop();
    Serial.println("[TUNNEL] Door closed...");
    delay(200);
  }
  Serial.println("[TUNNEL] Door open — resuming");
  return true;
}


// ── navigateTunnel ────────────────────────────────────────────────────────────
// Single-iteration PD wall-follower with ramp tilt compensation.
// Returns true while the robot is still in the tunnel; false once clear.

bool navigateTunnel() {
  float tilt = getTilt();

  int distL = readDistance(TRIG_L, ECHO_L);
  int distR = readDistance(TRIG_R, ECHO_R);

  bool tilted     = (fabsf(tilt) > TILT_THRESHOLD);
  bool wallsClose = (distL < TUNNEL_WALL_DIST && distR < TUNNEL_WALL_DIST);

  Serial.print("[TUNNEL] tilt="); Serial.print(tilt);
  Serial.print(" L="); Serial.print(distL);
  Serial.print(" R="); Serial.print(distR);
  Serial.print(tilted ? "  RAMP" : "  flat");
  Serial.println(wallsClose ? "  WALLS_CLOSE" : "");

  if (!tilted && !wallsClose) return false;  // not in tunnel

  bool goingDown = (tilt < DOWNHILL_THRESHOLD);

  // Check for door on the way down; latch exit speed once it opens
  bool doorOpened = waitForDoor();
  if (goingDown && doorOpened) {
    exitingThroughDoor = true;
    Serial.println("[TUNNEL] Door opened on decline — exit speed");
  }

  int baseSpeed = (goingDown && !exitingThroughDoor)
                  ? W_BASE - DOWNHILL_REDUCTION
                  : W_BASE;

  // Positive error = further from right wall = steer right
  int error      = distL - distR;
  int derivative = error - wfLastError;
  wfLastError    = error;

  float kp = (goingDown && !exitingThroughDoor) ? WK_P_DOWN : WK_P;
  int correction = constrain((int)(kp * error + WK_D * derivative), -450, 450);

  int leftSpeed  = constrain(baseSpeed + correction, 0, 800);
  int rightSpeed = constrain(baseSpeed - correction, 0, 800);

  mc1.setSpeed(2, leftSpeed);
  mc1.setSpeed(3, rightSpeed);
  mc2.setSpeed(2, leftSpeed);
  mc2.setSpeed(3, rightSpeed);

  return true;
}


// ── Encoder-based cardinal facing (tunnel / pre-arena phase) ─────────────────
// Uses gyro turnLeft/turnRight from motion.ino.
// faceNorth/S/E/W for the arena are in line_following.ino (QTR-based).
// direction index: 0=W  1=N  2=E  3=S

void faceDir(int targetDir) {
  int diff = (targetDir - direction + 4) % 4;
  if      (diff == 1) turnRight();
  else if (diff == 3) turnLeft();
  else if (diff == 2) { turnRight(); turnRight(); }
  // diff == 0: already facing
}


// ── getTagId ──────────────────────────────────────────────────────────────────
// Returns the UID of the card currently in rfid->uid as an uppercase hex String.

String getTagId() {
  String id = "";
  for (byte i = 0; i < rfid->uid.size; i++) {
    if (rfid->uid.uidByte[i] < 0x10) id += "0";
    id += String(rfid->uid.uidByte[i], HEX);
  }
  id.toUpperCase();
  return id;
}


// ── runChainMode ──────────────────────────────────────────════════════════════
// Mode 1 state machine — call repeatedly from loop().
//
//   NAVIGATING:       wall-follow through tunnel; scan RFID tags for waypoints
//                     and for the Airlock A tag.
//   WAITING_AIRLOCK:  motors stopped; comms callback will set state=LINE_FOLLOWING
//                     once the server accepts the openAirlock request.
//   LINE_FOLLOWING:   QTR proportional line follow in the arena.

void runChainMode() {

  // ── Post-airlock: line follow in arena ─────────────────────────────────────
  if (chainState == LINE_FOLLOWING) {
    followLine();
    return;
  }

  // ── Waiting for server airlock reply ───────────────────────────────────────
  if (chainState == WAITING_AIRLOCK) {
    stopMotors();
    return;
  }

  // ── NAVIGATING: tunnel wall-follow ─────────────────────────────────────────
  if (navigateTunnel()) {
    wasInTunnel = true;
    hadRamp     = true;
    return;
  }

  // Tunnel just exited (navigateTunnel returned false after being true)
  if (wasInTunnel) {
    wasInTunnel        = false;
    exitingThroughDoor = false;
    stopMotors();
    Serial.println("[CHAIN] Exited tunnel");
    if (hadRamp) {
      hadRamp = false;
      // Ramp traversed but no Airlock A tag found — go straight to line follow
      chainState = LINE_FOLLOWING;
      inArena    = true;
      Serial.println("[CHAIN] No airlock tag — entering arena, LINE_FOLLOWING");
      return;
    }
  }

  // ── RFID waypoint / airlock detection ──────────────────────────────────────
  if (rfid != nullptr && rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
    String tagId = getTagId();
    Serial.print("[CHAIN] Tag: "); Serial.println(tagId);

    if (tagId == AIRLOCK_A_TAG_ID) {
      // Convert String to char array for requestAirlock
      char tagIdBuf[12];
      tagId.toCharArray(tagIdBuf, sizeof(tagIdBuf));
      requestAirlock("A", tagIdBuf);
      chainState = WAITING_AIRLOCK;
      stopMotors();
      Serial.println("[CHAIN] Airlock A requested — waiting for server");
      rfid->PICC_HaltA();
      return;
    }

    // Normal waypoint — update position, drop seed, face next node
    rfid->PICC_HaltA();
    if      (direction == 0) x -= 1;
    else if (direction == 1) y += 1;
    else if (direction == 2) x += 1;
    else if (direction == 3) y -= 1;

    if (step < 5 && fertiles[step]) {
      dropSeed();
      fertiles[step] = false;
    }

    step++;
    if (step < 5) {
      int dx = pathx[step] - x;
      int dy = pathy[step] - y;
      if      (dx ==  1) faceDir(2);   // East
      else if (dx == -1) faceDir(0);   // West
      else if (dy ==  1) faceDir(1);   // North
      else if (dy == -1) faceDir(3);   // South
    }
  }
}
