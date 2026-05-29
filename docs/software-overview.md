# Software Overview — Group 6 Robot

## File / Module Map

```
button_mode_dr.ino          ← entry point
│  Globals: all hardware objects, pin constants, path array,
│           chainState, robotRunning, enable, presses
│  setup()  — initialises all hardware, calibrates QTR + gyro,
│             connects MQTT
│  loop()   — pumps comms, checks kill switch, dispatches mode
│
├── helpers.ino
│     readDistance()       ultrasonic HC-SR04 single reading
│     checkButton()        debounced kill-switch + revival + mode-advance
│     updateLEDs()         non-blocking red blink / green solid
│     setupMotoron()       Motoron reinit + clear reset flag
│     dropSeed()           mc2 ch1 pulse
│     stopMotors()         zero all drive channels
│     findI2CAddress()     Wire2 scan for RFID
│
├── motion.ino
│     countEncoder1/2()    ISRs — RISING edge on A pins
│     readGyroZ_radps()    raw MPU-6050 Z read
│     rezeroGyroBias()     500-sample mean/variance check
│     updateHeading()      complementary filter (gyro 98% + enc 2%)
│     turnInPlace()        gyro-based closed-loop 90° pivot
│     turnLeft/Right()     wrappers → ±PI/2
│     driveToNode()        encoder odometry + heading correction
│     driveOneNode()       drive + RFID log + seed + bias re-zero
│     runDeadReckoning()   full pathx/pathy sequence (mode 3)
│
├── line_following.ino
│     onLine()             any QTR sensor > 100
│     turnLeftQTR()        spin CCW until line lost then re-found
│     turnRightQTR()       spin CW  until line lost then re-found
│     faceDirQTR()         absolute heading (QTR, arena only)
│     faceNorth/S/E/W()    wrappers → faceDirQTR
│     followLine()         proportional follow; RFID → update x/y,
│                          face next waypoint, 200-iter skip loop
│     goToPointB()         pre-programmed turn sequence + door stop
│
├── wall_following.ino
│     getTilt()            complementary filter pitch (MPU-6050)
│     waitForDoor()        block while front < 8 cm
│     navigateTunnel()     dual-wall PD + tilt; returns true in tunnel
│     faceDir()            encoder-based cardinal turn (tunnel phase)
│     getTagId()           RFID UID → uppercase hex String
│     runChainMode()       NAVIGATING → WAITING_AIRLOCK → LINE_FOLLOWING
│
├── obstacle_avoidance.ino
│     checkAllDirections() read all 3 ultrasonic into obsDist[]
│     obsTurnInPlace()     encoder-based pivot (OBS speeds)
│     obsLeft/Right90()    wrappers
│     driveForwardDist()   front-sensor odometer drive
│     avoidObstacle()      8-step 3-point box swerve (60 s timeout)
│     runObstacleMode()    mode 5 entry: drive + swerve on detect
│
├── comms.ino
│     onCommsMessage()     MQTT receive handler (heartbeat, disable,
│                          openAirlockReply, reviveReply, 6-byte status)
│     commsHeartbeatCheck() stop if no heartbeat for 3 s
│     sendStatusUpdate()   register every 10 s, status every 2 s
│     requestAirlock()     send openAirlock to server
│     waitForAirlockAccepted() blocking poll with timeout
│     sendReviveRequest()  notify server of revival
│
└── revival.ino
      runRevival()         phase 1: full speed to 40 cm
                           phase 2: MIN_SPEED crawl to bumper contact
                           → sendReviveRequest() to server
```

---

## Component Interaction Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      PHYSICAL INPUTS                            │
│                                                                 │
│  Encoder 1 (52/50) ─────┐                                      │
│  Encoder 2 (53/51) ─────┤──→ encoderCount1/2 (volatile)        │
│                          │                                      │
│  MPU-6050 (Wire) ────────┼──→ gyro Z, accel X/Y/Z              │
│                          │                                      │
│  HC-SR04 F/L/R ──────────┼──→ readDistance()                   │
│                          │                                      │
│  QTR x9 (23-31) ─────────┼──→ sensorValues[]                   │
│                          │                                      │
│  RFID (Wire2) ───────────┼──→ rfid->uid                        │
│                          │                                      │
│  Buttons 49/47/45 ───────┘──→ checkButton()                    │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                    button_mode_dr.ino
                    loop() dispatcher
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
  enable (MQTT)    robotRunning (HW)    presses (mode)
  comms.ino        checkButton()        checkButton()
        │                  │                  │
        └──────────┬────────┘                  │
                   ▼                           ▼
           if !enable OR             ┌─────────────────┐
           !robotRunning             │  Mode dispatch  │
           → stopMotors()            │  0: followLine  │
                                     │  1: runChainMode│
                                     │  2: followLine  │
                                     │  3: runDR       │
                                     │  4: runRevival  │
                                     │  5: runObstacle │
                                     └────────┬────────┘
                                              │
        ┌─────────────────────────────────────┼─────────────────────┐
        │                  │                  │                     │
        ▼                  ▼                  ▼                     ▼
  line_following      wall_following       motion.ino         obstacle_avoidance
  QTR sensor          MPU tilt            Gyro turns          Ultrasonic swerve
  RFID pos tracking   Dual-wall PD        Enc odometry        Encoder pivots
  QTR turns           Chain state m/c     DR path             3-point box path

        └─────────────────────────────────────┼─────────────────────┘
                                              │
                                     ┌────────▼────────┐
                                     │  MOTOR OUTPUTS  │
                                     │  mc1 ch2 = Left │
                                     │  mc1 ch3 = Right│
                                     │  mc2 ch2 = Left │
                                     │  mc2 ch3 = Right│
                                     │  mc2 ch1 = Seed │
                                     └─────────────────┘
```

---

## Enable / Safety Logic

Two independent flags must BOTH be true for the robot to move:

```
enable       — controlled by MQTT server (heartbeat, emergency messages)
robotRunning — controlled by physical buttons and kill switch

Robot moves iff:  enable == 1  AND  robotRunning == true

Kill switch (pin 45) → robotRunning = false   (instant stop)
Button 1 or 2       → robotRunning = true    (when stopped: revival/start)
MQTT heartbeat lost → enable = 0              (3 s watchdog)
MQTT type=disable   → enable = 0
```

---

## Chain Mode State Machine (Mode 1)

```
chainState: NAVIGATING ──────────────────────────────────┐
                │                                         │
                │ navigateTunnel() returns true           │
                ▼                                         │
          (loop in tunnel)                                │
                │                                         │
                │ tilt returns to flat, walls open        │
                ▼                                         │
          navigateTunnel() returns false                  │
                │                                         │
                ├── RFID == AIRLOCK_A_TAG_ID              │
                │       → requestAirlock("A")             │
                │       → chainState = WAITING_AIRLOCK    │
                │                                         │
                │             WAITING_AIRLOCK             │
                │               stopMotors()              │
                │               [wait for MQTT reply]     │
                │               openAirlockReply +        │
                │               accepted=true + airlock=A │
                │               → chainState=LINE_FOLLOWING│
                │               → inArena = true          │
                │                                         │
                └── no airlock tag found after ramp       │
                        → chainState = LINE_FOLLOWING ────┘
                        → inArena = true

chainState: LINE_FOLLOWING
  followLine() — QTR proportional, RFID position tracking
```

---

## MQTT Message API

### Inbound (server → robot)

| Message | Action |
|---------|--------|
| `type=heartbeat enable=1` | `enable=1`, update watchdog timestamp |
| `type=heartbeat enable=0` | `enable=0`, stop motors |
| `type=disable` | `enable=0`, stop motors |
| `type=emergency` | `enable=0`, stop motors |
| `type=openAirlockReply accepted=true airlock=A` | `chainState=LINE_FOLLOWING`, `inArena=true` |
| `type=openAirlockReply accepted=false` | Log denial reason |
| `type=reviveReply` | Log to Serial |
| 6-byte binary (byte 4 set) | Emergency flag → stop |

### Outbound (robot → server)

| Message | When sent |
|---------|-----------|
| `type=register team_id=6 board_id=LEAK` | Every 10 s |
| `STATUS:<mode> ENABLE:<n> PRESSES:<n>` | Every 2 s |
| `type=openAirlock airlock=A tag_id=<uid> board_id=LEAK` | Airlock A RFID detected |
| `type=reviveRequest target_team=? target_board=?` | Bumper contact in mode 4 |
