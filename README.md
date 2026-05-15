# Robotics Challenge — Group 6

Arduino firmware for the Term 3 Robotics Challenge. The robot is a four-wheel skid-steer platform built on an Arduino Giga R1 WiFi, capable of line-following, gyro-guided turning, RFID waypoint detection, seed dispersal, and obstacle avoidance via TF-Luna LiDAR.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Arduino Giga R1 WiFi |
| Motor drivers | 2× Pololu Motoron M3S550 (I2C on Wire1) |
| Drive motors | 4× SparkFun DG01D-E (metal gearbox + Hall encoder) |
| Line sensor | Pololu QTRX-HD-09RC (9-channel RC, pins 24–40 even) |
| LiDAR | Benewake TF-Luna (I2C on Wire1, 0x10) |
| RFID | M5Stack RFID2 WS1850S (I2C on Wire2, auto-scanned) |
| IMU | MPU-6050 (I2C on Wire, ±500 °/s, 21 Hz LPF) |
| Battery | Ansmann 10.9 V 3500 mAh 3S1P Li-ion |

## Repository Structure

```
Robotics_challenge/
├── electronics/
│   └── electronics.ino   # Full sensor integration test
├── mechanical/
│   └── mechanical.ino    # Seed dispenser + revival button test
├── gyro_turning/
│   └── gyro_turning.ino  # Gyro-guided 90° and 180° turn calibration
└── mission/
    └── mission.ino       # Full mission: line-follow, RFID, seed drop, turns
```

## Sketches

### `electronics/`
Exercises every sensor and actuator in a single loop. On startup it:
1. Scans Wire2 for the RFID module (dynamic address discovery).
2. Scans Wire1 for the two Motoron shields (dynamic address discovery).
3. Initialises the TF-Luna LiDAR on Wire1.
4. Initialises the MPU-6050 on Wire.
5. Runs a 2-second QTR calibration sweep.

Once started with the button (pin 45), it drives forward, executes right/left 90° turns and a U-turn, polling all sensors continuously. The LED blinks red when stopped and is solid red when running.

### `mechanical/`
Standalone test for the revolver seed dispenser. Press the button (pin 45) to trigger a 6-pulse dispense cycle (710 ms on / 2000 ms off per pulse). The interface button (pin 49) lights the LED green while held, confirming the revival contact circuit.

### `gyro_turning/`
Calibration sketch used to determine gyro integration thresholds. Executes a 90° turn (threshold `|z| ≥ 0.9 rad`) followed by a 180° U-turn (threshold `|z| ≥ 1.5 rad`), printing the accumulated yaw to Serial at 115200 baud. Used to validate the thresholds carried into the mission code.

### `mission/`
Full mission loop. On each iteration:
- If an RFID tag is detected, drops a seed and turns right 90°, then line-follows for 200 cycles to clear the waypoint before scanning again.
- Otherwise, runs proportional error line-following using weighted sensor readings across the 9-channel QTRX array.

## Pin Assignments

| Pin | Function |
|-----|----------|
| 24, 26, 28, 30, 32, 34, 36, 38, 40 | QTRX-HD-09RC channels 1–9 |
| 45 | Start/stop button (INPUT_PULLUP) |
| 47 | Revival button A (INPUT_PULLUP) |
| 49 | Revival button B / interface button (INPUT_PULLUP) |
| 51 | RGB LED green channel |
| 53 | RGB LED red channel |
| Wire SDA/SCL | MPU-6050 |
| Wire1 SDA/SCL | Motoron MC1 (0x12), Motoron MC2 (0x11), TF-Luna (0x10) |
| Wire2 SDA/SCL | M5Stack RFID2 (auto-scanned, typically 0x28) |

## Dependencies

Install all libraries via the Arduino Library Manager:

- [Motoron](https://github.com/pololu/motoron-arduino) — Pololu Motoron motor controller
- [TFLI2C](https://github.com/budryerson/TFLi2C) — TF-Luna LiDAR
- [MFRC522_I2C](https://github.com/arozcan/MFRC522-I2C-Library) — RFID reader
- [QTRSensors](https://github.com/pololu/qtr-sensors-arduino) — Pololu QTR line sensors
- [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050) — IMU
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) — sensor abstraction layer

## Usage

1. Open the desired sketch folder in the Arduino IDE.
2. Select **Arduino Giga R1 WiFi** as the board.
3. Upload and open Serial Monitor at **115200 baud**.
4. For `electronics` and `mission`: slowly sweep the QTRX array over the line during the 2-second calibration window.
5. Press the button on pin 45 to start.