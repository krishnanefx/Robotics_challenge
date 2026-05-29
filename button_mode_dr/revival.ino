// ══════════════════════════════════════════════════════════════════════════════
//  revival.ino — Task 8: Touch-Based Robot Revival (presses == 4)
//
//  Behaviour:
//    1. Drive forward at full speed until ~40 cm from target
//    2. Drop to MIN_SPEED (400) — the lowest speed before motor stall
//    3. Continue crawling until EITHER bumper button (button1 or button2)
//       registers physical contact
//    4. Stop immediately, notify server, wait for reviveReply
//
//  runRevival() runs the sequence once then idles.
//  Caller (loop) must keep calling it while presses == 4.
// ══════════════════════════════════════════════════════════════════════════════

// Distance thresholds (cm)
static const int REVIVAL_APPROACH_DIST = 40;  // switch from fast → crawl


// Drive both motors at the given speed (positive = forward).
static void setDrive(int spd) {
  mc1.setSpeed(2, spd); mc1.setSpeed(3, spd);
  mc2.setSpeed(2, spd); mc2.setSpeed(3, spd);
}


void runRevival() {
  static bool revivalDone = false;
  if (revivalDone) return;

  Serial.println("[REVIVAL] Starting approach");

  // ── Phase 1: full speed until inside approach zone ────────────────────────
  while (true) {
    messenger.loop();
    if (!enable) { stopMotors(); return; }

    int dist = readDistance(TRIG_F, ECHO_F);

    if (dist <= REVIVAL_APPROACH_DIST) break;

    setDrive(DRIVE_SPEED);
  }

  Serial.println("[REVIVAL] Approach zone — switching to crawl");

  // ── Phase 2: crawl at MIN_SPEED until bumper contact ─────────────────────
  // MIN_SPEED = 400 (defined in button_mode_dr.ino) is the stall floor.
  while (true) {
    messenger.loop();
    if (!enable) { stopMotors(); return; }

    if (anyBumperPressed()) {
      stopMotors();
      Serial.println("[REVIVAL] Contact detected via bumper button");
      break;
    }

    setDrive(MIN_SPEED);
  }

  // ── Phase 3: notify server ────────────────────────────────────────────────
  // target_team and target_board are placeholders — update if the stranded
  // robot's identity is known in advance, or read from a distress broadcast.
  sendReviveRequest("?", "?");

  Serial.println("[REVIVAL] Sequence complete — idling");
  revivalDone = true;
}
