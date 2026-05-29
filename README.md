# Robotics Challenge — Group 6 (COMP0204, 2025)

> ** Viva / test-run code: [`button_mode_dr/`](button_mode_dr/)**  
> This is the sketch for trial run 2. All behaviours demonstrated in the viva are in this folder.

---

## Table of Contents
1. [Repository Structure](#repository-structure)
2. [Hardware](#hardware)
3. [Required Libraries](#required-libraries)
4. [Setup Steps](#setup-steps)
5. [Upload & Run](#upload--run)
6. [Mode Guide](#mode-guide-button-presses)
7. [Key Constants to Tune](#key-constants-to-tune)
8. [Software Diagrams & Flowcharts](#software-diagrams--flowcharts)

---

## Repository Structure

```
Robotics_challenge/
│
├── button_mode_dr/            ← viva/trial run 2 code
│   ├── button_mode_dr.ino     # Globals, setup(), loop(), State enum, LEDs
│   ├── motion.ino             # Dead reckoning: gyro turns, encoder drive, bias cal
│   ├── line_following.ino     # QTR line follow, QTR turns, arena navigation
│   ├── wall_following.ino     # Tunnel PD wall-follow, tilt detection, chain mode
│   ├── obstacle_avoidance.ino # 3-point box swerve around obstacles
│   ├── comms.ino              # MiniMessenger WiFi/MQTT, heartbeat, airlock API
│   ├── helpers.ino            # Ultrasonic, Motoron init, seed drop, buttons
│   └── revival.ino            # Mode 4: approach & contact revival behaviour
│
├── electronics/               # Sensor integration test sketch (development)
├── gyro_turning/              # Standalone gyro turn calibration sketch
├── mechanical/                # Seed dispenser + bumper test sketch
├── mission/                   # Early mission prototype (line + RFID + seed)
│
└── docs/
    ├── software-overview.md   # Component diagram and data-flow description
    ├── flowcharts.md          # Detailed flowcharts for all key behaviours
    └── calibration.md         # Testing logs, what worked, what did not
```

> Arduino concatenates all `.ino` files in a folder before compiling, so all globals
> declared in `button_mode_dr.ino` are visible across every tab automatically.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Arduino Giga R1 WiFi |
| Motor drivers | 2× Pololu Motoron M3S256 (I²C on Wire1, addr 17 & 18) |
| Drive motors | 4× DC motors with Hall-effect encoders |
| Line sensor | Pololu QTRX-HD-09RC (9-channel RC, pins 23–31) |
| Ultrasonic | 3× HC-SR04 — Front 37/36, Left 41/40, Right 39/38 |
| RFID | M5Stack WS1850S (I²C on Wire2, address auto-scanned) |
| IMU | MPU-6050 (I²C on Wire, ±500 °/s gyro, 21 Hz LPF) |
| Buttons | Button 1 = pin 49, Button 2 = pin 47, Kill switch = pin 45 |
| LEDs | Red = pin TBD, Green = pin TBD (fill in `button_mode_dr.ino`) |

### Motoron Channel Map

| Controller | I²C addr | Channel | Function |
|------------|----------|---------|----------|
| mc1 | 18 | ch2 | Left drive |
| mc1 | 18 | ch3 | Right drive |
| mc2 | 17 | ch2 | Left drive |
| mc2 | 17 | ch3 | Right drive |
| mc2 | 17 | ch1 | Seed dispenser |

### Encoder Pins

| Encoder | Pin A (interrupt) | Pin B (direction) |
|---------|-------------------|-------------------|
| Encoder 1 | 52 | 50 |
| Encoder 2 | 53 | 51 |

---

## Required Libraries

Install all via **Arduino IDE → Tools → Manage Libraries**, or via `.zip`:

| Library | Source |
|---------|--------|
| `Motoron` | Library Manager → "Motoron" by Pololu |
| `Adafruit MPU6050` | Library Manager → "Adafruit MPU6050" |
| `Adafruit Unified Sensor` | Library Manager (dependency of above) |
| `QTRSensors` | Library Manager → "QTRSensors" by Pololu |
| `MFRC522_I2C` | Library Manager → "MFRC522_I2C" |
| `MiniMessenger` | Library Manager → "MiniMessenger" |

---

## Setup Steps

### 1. Hardware connections
Wire all sensors and motors per the pin table above and connect the Arduino Giga R1 WiFi via USB.

### 2. Set LED pins
In `button_mode_dr.ino`, fill in once the wires are known:
```cpp
const int redPin   = -1;   // replace with actual pin
const int greenPin = -1;   // replace with actual pin
```

### 4. Set Airlock A RFID tag UID
In `wall_following.ino`, scan the tag once, read the UID from Serial, then paste it:
```cpp
const String AIRLOCK_A_TAG_ID = "XXXXXXXX";
```

### 5. QTR sensor calibration
The robot automatically calibrates the line sensor for ~4 seconds on startup.
During this window, **slowly move the sensor array back and forth across the line**
so all 9 sensors see both black tape and white floor. The Serial monitor will print
`"Calibration done!"` when finished.

### 6. Gyro bias calibration
Keep the robot **completely still** for ~1.5 seconds after power-on while the gyro
samples 500 readings. The Serial monitor prints `[GYRO] Bias: <value>` when done.
If the robot is moving, calibration is skipped and retried at the next turn.

---

## Upload & Run

1. Open `button_mode_dr/button_mode_dr.ino` in **Arduino IDE 2.x** — all 8 tabs load automatically.
2. Select **Board: Arduino Giga R1 WiFi** and the correct COM port.
3. Click **Upload**.
4. Open **Serial Monitor at 115200 baud** to watch startup progress and mode output.
5. Wait for `=== Ready ===` in Serial.
6. Press **Button 1 (pin 49)** to start the robot. The green LED comes on.
7. Press **Button 1** again while running to advance to the next mode.
8. Press **Kill switch (pin 49)** at any time to stop all motors immediately.

---

## Mode Guide (Button Presses)

| Presses | Mode | Description |
|---------|------|-------------|
| 0 | **Line Follow** | Proportional QTR line following; RFID updates grid position |
| 1 | **Chain Mode** | Tunnel wall-follow → Airlock A request → Arena line follow |
| 2 | **Line Follow 2** | Same as mode 0 (second pass / return) |
| 3 | **Dead Reckoning** | Gyro-guided 90° turns + encoder odometry along path array |
| 4 | **Revival** | Full-speed approach to 40 cm, then crawl to bumper contact |
| 5 | **Obstacle Avoidance** | Forward drive with automated 3-point box swerve |

> **Kill switch (pin 45)** — stops all motors instantly, regardless of mode.  
> **Button 1 or 2** — restarts robot after kill-switch stop (revival / start).

---

## Key Constants to Tune

| Constant | File | Value | Notes |
|----------|------|-------|-------|
| `TURN_SCALE` | `button_mode_dr.ino` | `0.90` | Gyro brake point. ↑ if undershooting, ↓ if overshooting. Step by 0.03 |
| `DRIVE_SPEED` | `button_mode_dr.ino` | `600` | Forward speed for dead reckoning |
| `DRIVE_KP` | `button_mode_dr.ino` | `80.0` | Heading correction P-gain during straight drive |
| `TRACK_WIDTH_CM` | `button_mode_dr.ino` | `17.0` | Measure axle-to-axle and update |
| `COUNTS_PER_REV` | `button_mode_dr.ino` | `144` | Encoder ticks per wheel revolution |
| `MIN_SPEED` | `button_mode_dr.ino` | `400` | Stall floor — never command motors below this |
| `WK_P` | `wall_following.ino` | `80.0` | Tunnel wall-follow P gain (uphill/flat) |
| `WK_P_DOWN` | `wall_following.ino` | `20.0` | Tunnel P gain (downhill — gentler) |
| `W_BASE` | `wall_following.ino` | `700` | Tunnel base speed |
| `OBS_DRIVE_SPEED` | `obstacle_avoidance.ino` | `500` | Obstacle mode forward speed |
| `HEARTBEAT_TIMEOUT_MS` | `comms.ino` | `3000` | ms without heartbeat before motors stop |

---

## Software Diagrams & Flowcharts

- [`docs/software-overview.md`](docs/software-overview.md) — Component interaction diagram and data flow
- [`docs/flowcharts.md`](docs/flowcharts.md) — Flowcharts for every key behaviour:
  - Startup & sensor calibration
  - Main loop and mode switching (kill switch, enable, robotRunning)
  - Line following with RFID position tracking
  - Chain mode state machine (NAVIGATING → WAITING_AIRLOCK → LINE_FOLLOWING)
  - Dead reckoning with gyro turns and encoder odometry
  - Obstacle avoidance 3-point swerve
  - Revival approach and bumper contact
  - MQTT comms, heartbeat watchdog, and airlock API
- [`docs/calibration.md`](docs/calibration.md) — Testing logs, calibration runs, what worked and what did not
