// ══════════════════════════════════════════════════════════════════════════════
//  comms.ino — MiniMessenger WiFi/MQTT communication
//
//  Server message types handled (downlink):
//    type=heartbeat      enable=1/0, seq, time_left
//    type=disable        enabled=false
//    type=emergency      enabled=true
//    type=openAirlockReply  airlock, accepted, reason
//    type=reviveReply    status, target
//    6-byte binary       team status (queue/busy/emergency flags)
//
//  Outbound helpers (call from other files):
//    requestAirlock(id, tagId)  — send openAirlock to server; "A"=enter "B"=exit
//    waitForAirlockAccepted(ms) — block until accepted=true or timeout
//    sendReviveRequest()        — notify server of successful revival
//
//  Internal periodic tasks (called from loop via sendStatusUpdate):
//    register every 10 s
//    status broadcast every 2 s
//    heartbeat timeout watchdog
// ══════════════════════════════════════════════════════════════════════════════


// ── State ─────────────────────────────────────────────────────────────────────

static unsigned long lastHeartbeatMs = 0;
static unsigned long lastRegisterMs  = 0;
bool                 airlockAccepted = false;  // non-static: read by line_following.ino
static char          lastTagId[12]   = "";    // UID from last RFID scan

// 6-byte binary team status (updated on every server broadcast)
//   [0] queueExit      [1] airlockBBusy
//   [2] queueEnter     [3] airlockABusy
//   [4] emergency      [5] reEntryRequested
static uint8_t teamStatus[6] = {0};

// How long without a heartbeat before we stop. 3 s is more forgiving than
// the 1 s shown in examples — accounts for WiFi jitter on the lab network.
static const unsigned long HEARTBEAT_TIMEOUT_MS = 3000;


// ── Incoming message handler ──────────────────────────────────────────────────

void onCommsMessage(const MessageMetadata& meta,
                    const uint8_t* payload, size_t length) {

  // ── 6-byte binary team status broadcast ───────────────────────────────────
  if (length == 6) {
    memcpy(teamStatus, payload, 6);
    if (teamStatus[4]) {           // emergency byte set
      enable = 0;
      stopMotors();
      Serial.println("[COMMS] Emergency flag in team status — stopped");
    }
    return;
  }

  // ── 21-byte binary map — ignore for now ───────────────────────────────────
  if (length == 21) return;

  // ── Text messages ─────────────────────────────────────────────────────────
  char msg[MiniMessenger::kMaxPayloadSize + 1];
  size_t n = min(length, (size_t)MiniMessenger::kMaxPayloadSize);
  memcpy(msg, payload, n);
  msg[n] = '\0';

  Serial.print("[COMMS] from="); Serial.print(meta.fromBoardId);
  Serial.print(" | "); Serial.println(msg);

  // heartbeat — the server's keep-alive; drives the enable flag
  if (strstr(msg, "type=heartbeat")) {
    lastHeartbeatMs = millis();
    if      (strstr(msg, "enable=1")) { enable = 1; }
    else if (strstr(msg, "enable=0")) { enable = 0; stopMotors(); }
    return;
  }

  // hard disable or global emergency
  if (strstr(msg, "type=disable") || strstr(msg, "type=emergency")) {
    enable = 0;
    stopMotors();
    Serial.println("[COMMS] Disable/Emergency — motors stopped");
    return;
  }

  // airlock reply
  if (strstr(msg, "type=openAirlockReply")) {
    airlockAccepted = (strstr(msg, "accepted=true") != nullptr);
    if (airlockAccepted) {
      Serial.println("[DOOR] Airlock accepted");
      // Airlock A accepted → transition chain mode into arena line following
      if (strstr(msg, "airlock=A")) {
        chainState = LINE_FOLLOWING;
        inArena    = true;
        Serial.println("[DOOR] Airlock A open — entering arena, LINE_FOLLOWING");
      }
    } else {
      Serial.print("[DOOR] Airlock denied");
      char* reason = strstr(msg, "reason=");
      if (reason) { Serial.print(" — "); Serial.println(reason + 7); }
      else          Serial.println();
    }
    return;
  }

  // revive reply
  if (strstr(msg, "type=reviveReply")) {
    Serial.print("[REVIVE] Server reply: "); Serial.println(msg);
    return;
  }
}


// ── Heartbeat watchdog — call every loop() iteration ─────────────────────────
// If the server stops sending heartbeats (e.g. MQTT drop) we stop moving.

void commsHeartbeatCheck() {
  if (lastHeartbeatMs == 0) return;   // haven't received first heartbeat yet
  if (millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    if (enable) {
      enable = 0;
      stopMotors();
      Serial.println("[COMMS] Heartbeat timeout — stopped");
    }
  }
}


// ── Periodic outbound tasks ───────────────────────────────────────────────────

void sendStatusUpdate() {
  if (!messenger.isConnected()) return;

  // Register every 10 s so the server keeps us "Online"
  if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    char reg[64];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", group, board);
    messenger.sendToBoard("server", reg);
  }

  // Status broadcast every 2 s
  if (millis() - lastSend < 2000) return;
  lastSend = millis();

  const char* modeName;
  if      (presses == 0) modeName = "LINE_FOLLOW";
  else if (presses == 1) modeName = "CHAIN";
  else if (presses == 2) modeName = "LINE_FOLLOW_2";
  else if (presses == 3) modeName = "DEAD_RECKONING";
  else if (presses == 4) modeName = "REVIVAL";
  else if (presses == 5) modeName = "OBSTACLE";
  else                   modeName = "IDLE";

  char status[80];
  snprintf(status, sizeof(status), "STATUS:%s ENABLE:%d PRESSES:%d",
           modeName, enable, presses);
  messenger.sendToGroup(status);
}


// ── Airlock request helpers ───────────────────────────────────────────────────

// Send an openAirlock request. airlockId = "A" (enter) or "B" (exit).
// tagId should be the UID string read from the RFID tag at the airlock.
void requestAirlock(const char* airlockId, const char* tagId) {
  airlockAccepted = false;
  char msg[96];
  snprintf(msg, sizeof(msg),
           "type=openAirlock airlock=%s tag_id=%s board_id=%s",
           airlockId, tagId, board);
  messenger.sendToBoard("server", msg);
  Serial.print("[DOOR] Requested airlock "); Serial.print(airlockId);
  Serial.print(" with tag "); Serial.println(tagId);
}

// Block until the server accepts the request or timeoutMs elapses.
// Returns true if accepted.
bool waitForAirlockAccepted(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    messenger.loop();
    if (airlockAccepted) return true;
  }
  Serial.println("[DOOR] Timeout waiting for airlock reply");
  return false;
}

// Store the UID from the most recent RFID scan so requestAirlock can use it.
void setLastTagId(const char* uid) {
  strncpy(lastTagId, uid, sizeof(lastTagId) - 1);
  lastTagId[sizeof(lastTagId) - 1] = '\0';
}

const char* getLastTagId() { return lastTagId; }


// ── Revival notification ──────────────────────────────────────────────────────

void sendReviveRequest(const char* targetTeam, const char* targetBoard) {
  char msg[64];
  snprintf(msg, sizeof(msg),
           "type=reviveRequest target_team=%s target_board=%s",
           targetTeam, targetBoard);
  messenger.sendToBoard("server", msg);
  Serial.println("[REVIVE] Request sent to server");
}
