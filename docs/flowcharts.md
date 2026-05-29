# Behaviour Flowcharts — Group 6 Robot

---

## 1. Startup & Calibration

```
Power on
    │
    ▼
Serial.begin(115200)
Init LEDs (skip if pins = -1)
Init Wire / Wire1 / Wire2
    │
    ▼
Buttons + kill switch  INPUT_PULLUP (pins 49, 47, 45)
    │
    ▼
RFID scan Wire2 ──── found? ── yes ──→ PCD_Init()
    │ retry ×5           │                   │
    └── never found?      └───────────────────┘
         rfid = nullptr  (RFID-less mode)
    │
    ▼
Motoron mc1 (addr 18) + mc2 (addr 17) on Wire1
  reinitialise, disable CRC, clear reset flag
  accel=200  decel=300  all channels
    │
    ▼
Ultrasonic pins: TRIG OUTPUT, ECHO INPUT
    │
    ▼
MPU-6050 ── found? ── yes ──→ setRange ±8g, ±500°/s, 21Hz LPF
    │ retry ×5
    └── never found: WARNING, continue
    │
    ▼
Attach encoder ISRs (pins 52 / 53, RISING)
    │
    ▼
rezeroGyroBias()
  ├── delay 200 ms (let vibrations settle)
  ├── sample 500 gyro Z readings (~1.2 s)
  ├── variance > 0.05² rad²/s² ? → skip (robot moving)
  └── else gyroBiasZ = mean
    │
    ▼
QTR calibrate (200 samples × 20 ms = 4 s)
  ← move sensor over line during this window →
    │
    ▼
MiniMessenger.begin()  WiFi + MQTT connect
    │
    ▼
Serial: "=== Ready ==="
robotRunning = false  (waits for button press)
```

---

## 2. Main Loop & Mode Switching

```
loop() — runs forever
    │
    ├── messenger.loop()          pump MQTT queue
    ├── commsHeartbeatCheck()     no heartbeat >3 s → enable=0
    ├── sendStatusUpdate()        register/status to server
    ├── checkButton()             see flowchart 3
    └── updateLEDs()              see flowchart 4
    │
    ▼
enable=0 OR robotRunning=false?
    YES → stopMotors(), return
    │
    NO
    ▼
presses==0 → followLine()       Mode 0
presses==1 → runChainMode()     Mode 1
presses==2 → followLine()       Mode 2
presses==3 → runDeadReckoning() Mode 3
presses==4 → runRevival()       Mode 4
presses==5 → runObstacleMode()  Mode 5

Each runs inside:
  while (presses == N && enable && robotRunning) { ... }
If condition breaks → 1 s delay → fall through to next while
```

---

## 3. Kill Switch & Button Logic  (`checkButton()`)

```
Read pins: btn1(49), btn2(47), kill(45)
Only act on HIGH→LOW edge older than 50 ms (debounce)
    │
    ├── Kill switch (45) LOW?
    │     └──→ robotRunning = false
    │           stopMotors()
    │           Serial: "[BTN] Kill switch"
    │
    ├── Button 1 (49) LOW?
    │     ├── robotRunning = false?
    │     │     └──→ robotRunning = true  (start / revival)
    │     └── robotRunning = true AND presses ≠ 4?
    │           └──→ presses++            (mode advance)
    │                (guard: mode 4 = bumper contact,
    │                 don't accidentally skip mode)
    │
    └── Button 2 (47) LOW?
          └── robotRunning = false?
                └──→ robotRunning = true  (start / revival)
```

---

## 4. LED Status (`updateLEDs()`)

```
pins -1? → skip entirely
    │
    ├── robotRunning = false  → red BLINK 300 ms  (stopped)
    ├── enable = 0            → red SOLID          (MQTT disabled)
    ├── MQTT connected        → green SOLID        (running + comms)
    └── else                  → red BLINK 500 ms  (running, no MQTT)

All blinking is non-blocking (uses lastBlinkMs timestamp).
```

---

## 5. Line Following with RFID Position Tracking  (`followLine()`)

```
RFID card present?
    │
    YES:
    │   stopMotors(), delay 3 s
    │       │
    │       ▼
    │   inArena = true?
    │     ├── Update x,y from current direction heading
    │     │     direction==0 → x-=1  (West)
    │     │     direction==1 → y+=1  (North)
    │     │     direction==2 → x+=1  (East)
    │     │     direction==3 → y-=1  (South)
    │     │
    │     ├── step < 4 → step++
    │     │     compute dx = pathx[step]-x
    │     │             dy = pathy[step]-y
    │     │     dx==+1 → faceEast()   (QTR turn)
    │     │     dx==-1 → faceWest()
    │     │     dy==+1 → faceNorth()
    │     │     dy==-1 → faceSouth()
    │     │
    │     └── fertiles[step-1]? → dropSeed(), mark false
    │
    │   Drive 200 iterations past tag (avoid re-read)
    │   PICC_HaltA()
    │   return
    │
    NO (normal follow):
        Read 9 QTR sensors (calibrated)
        error = Σ(sensorValue[i] × weight[i])
        weights: [-0.15, -0.12, -0.08, -0.04, 0,
                  +0.04, +0.08, +0.12, +0.15]
        leftSpeed  = (400 + error) × 0.8
        rightSpeed = (400 - error) × 0.8
        set mc1 + mc2
```

---

## 6. QTR Cardinal Turns (`turnLeftQTR` / `faceNorth` etc.)

```
turnRightQTR():                     turnLeftQTR():
  Phase 1: spin CW                    Phase 1: spin CCW
    while onLine():                     while onLine():
      motors CW                           motors CCW
  Phase 2: re-acquire                 Phase 2: re-acquire
    while NOT onLine():                 while NOT onLine():
      motors CW                           motors CCW
  stopMotors()                        stopMotors()
  direction = (direction+1)%4         direction = (direction-1+4)%4

faceNorth/S/E/W() → faceDirQTR(target):
  diff = (target - direction + 4) % 4
  diff==1 → turnRightQTR()
  diff==3 → turnLeftQTR()
  diff==2 → turnRightQTR() × 2   (180°)
  diff==0 → already correct
```

---

## 7. Chain Mode State Machine  (`runChainMode()`, Mode 1)

```
chainState == LINE_FOLLOWING?
    └──→ followLine()  return   (permanent after airlock)

chainState == WAITING_AIRLOCK?
    └──→ stopMotors()  return
         [background MQTT]:
           openAirlockReply accepted=true airlock=A
           → chainState = LINE_FOLLOWING
           → inArena = true

chainState == NAVIGATING:
    │
    navigateTunnel()?
      YES → wasInTunnel=true, hadRamp=true, return
      │
      NO + wasInTunnel was true (just exited):
        wasInTunnel=false, stopMotors()
        hadRamp? → chainState=LINE_FOLLOWING, inArena=true, return
    │
    RFID card present?
      tagId == AIRLOCK_A_TAG_ID?
        → requestAirlock("A", tagId)
        → chainState = WAITING_AIRLOCK
        → stopMotors()
      else (normal waypoint):
        update x,y
        drop seed if fertile
        step++
        faceDir(next direction)  [encoder-based, tunnel context]
```

---

## 8. Tunnel Navigation (`navigateTunnel()`)

```
getTilt():
  read MPU-6050 accel + gyro
  pitchY = atan2(ay, √(ax²+az²)) × 180/π
  log pitchX, pitchY, pitchZ to Serial
  pitch = 0.90×(pitch + gyroX×dt×180/π) + 0.10×pitchY
  return pitch
    │
|pitch| ≤ 5° AND walls NOT close?
    └──→ return false  (not in tunnel)
    │
In tunnel:
  goingDown = (tilt < -20°)
  waitForDoor():
    front < 8 cm? → stopMotors, poll until clear
    door opened on decline → exitingThroughDoor=true
  baseSpeed = goingDown && !exitingThroughDoor
              ? 700-650=50   : 700
  error = distL - distR
  kP = goingDown ? 20.0 : 80.0
  correction = kP×error + 6.0×(error-lastError)
  correction = constrain(-450, 450)
  leftSpeed  = baseSpeed + correction
  rightSpeed = baseSpeed - correction
  return true
```

---

## 9. Gyro-Based Turn (`turnInPlace()`, used by Dead Reckoning)

```
target = |angleRad| × TURN_SCALE (0.90)
sign   = +1 (CCW/left) or -1 (CW/right)
accumulated = 0

loop while accumulated < target:
    dt = micros() delta
    skip if dt > 50 ms (stale reading)
    accumulated += |gyroZ - gyroBiasZ| × dt

    remaining = target - accumulated
    remaining > target/3 ?  spd = TURN_FAST (700)
    else                  :  spd tapers linearly → TURN_SLOW (600)
    spd = constrain(TURN_SLOW, TURN_FAST)
    mc1.setSpeed(2,  sign×spd)  ch2 forward
    mc1.setSpeed(3, -sign×spd)  ch3 reverse  (pivot)
    mc2 same

stopMotors()
update direction index
rezeroGyroBias()
```

---

## 10. Dead Reckoning (`runDeadReckoning()`, Mode 3)

```
(runs once; subsequent calls are no-ops)

rezeroGyroBias()
fertiles[0]? → dropSeed()

for i = 1 to 4:
    compute targetDir from pathx/pathy delta
    diff = (targetDir - direction + 4) % 4
    diff==1 → turnRight()
    diff==3 → turnLeft()
    diff==2 → turnRight() × 2

    driveOneNode(i):
        driveToNode():
            reset encoder counts, fusedHeading
            loop:
                updateHeading()
                  gyroAngle = (gyroZ - bias) × dt
                  encAngle  = (distR - distL) / trackWidth
                  fusedHeading += 0.98×gyroAngle + 0.02×encAngle
                avg = (|enc1| + |enc2|) / 2
                avg ≥ 176 counts (25 cm)? → stop
                RFID seen mid-drive?      → stop early
                correction = 80 × fusedHeading
                left  = DRIVE_SPEED - correction
                right = DRIVE_SPEED + correction
        log RFID tag if present
        dropSeed() if fertile
        rezeroGyroBias()

stopMotors()
drDone = true
```

---

## 11. Obstacle Avoidance (`avoidObstacle()`, Mode 5)

```
runObstacleMode():
  front dist ≤ 25 cm? → avoidObstacle()
  else → drive forward at OBS_DRIVE_SPEED

avoidObstacle():
  avoidStart = millis()
  checkAllDirections() → obsDist[FRONT/LEFT/RIGHT]
  goLeft = obsDist[LEFT] ≥ obsDist[RIGHT]

  CHECK_TIMEOUT() after each step (abort at 60 s)

  Step 1: turn 90° away from obstacle lane
  Step 2: driveForwardDist(25 cm)   — clear obstacle lane
  Step 3: turn 90° back to original heading
  Step 4: driveForwardDist(50 cm)   — pass obstacle (2 nodes)
  Step 5: turn 90° toward original lane
  Step 6: driveForwardDist(25 cm)   — return to path
  Step 7: turn 90° to face forward
  Step 8: driveForwardDist(25 cm)   — final node (3 total)

driveForwardDist(targetCm):
  startDist = front sensor reading
  drive until front sensor drops by targetCm
  fallback: timed drive if no wall ahead (sensor reads 999)
```

---

## 12. Revival Behaviour (`runRevival()`, Mode 4)

```
(runs once; subsequent calls are no-ops)

Phase 1 — full-speed approach:
  loop:
    readDistance(TRIG_F, ECHO_F)
    dist ≤ 40 cm? → break
    setDrive(DRIVE_SPEED = 600)

Phase 2 — slow crawl to contact:
  loop:
    anyBumperPressed()?
      (pin 49 OR pin 47 LOW)
      YES → stopMotors(), break
    setDrive(MIN_SPEED = 400)   ← stall floor

sendReviveRequest("?", "?")    ← fill in target team/board
revivalDone = true
```

---

## 13. MQTT Heartbeat & Emergency  (`comms.ino`)

```
Every loop() iteration:
  messenger.loop()  — pump MQTT receive queue

commsHeartbeatCheck():
  lastHeartbeatMs == 0? → skip (no first heartbeat yet)
  millis() - lastHeartbeatMs > 3000?
    → enable=0, stopMotors()
    → Serial: "Heartbeat timeout"

onCommsMessage():
  6-byte binary → check byte[4] (emergency) → enable=0
  type=heartbeat → update lastHeartbeatMs, set enable
  type=disable / type=emergency → enable=0, stopMotors()
  type=openAirlockReply:
    accepted=true + airlock=A
      → airlockAccepted=true
      → chainState=LINE_FOLLOWING
      → inArena=true
  type=reviveReply → log to Serial

sendStatusUpdate():
  every 10 s → type=register
  every  2 s → STATUS:<mode> ENABLE:<n> PRESSES:<n>
```
