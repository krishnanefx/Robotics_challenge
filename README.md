# Robotics Challenge - Group 6 Programming Submission

This repository is the programming submission for the COMP0204 2026 Robotics Challenge.

## Final Code for Finals Day

Use these sketches for the final robot:

| Sketch | Purpose |
|---|---|
| `Finals_game_mode/game_mode/game_mode.ino` | Main finals code: chain/ramp entry, arena RFID fertility checks, seed planting, emergency return, and revival support. |
| `Finals_game_mode/game_mode_hard/game_mode_hard.ino` | Hard-difficulty version: same behaviour as `game_mode`, plus arena-only obstacle avoidance. |

The Trial 2 tuning sketches are kept as evidence and calibration history. 

## Repository Structure

```text
Robotics_challenge/
|-- Finals_game_mode/
|   |-- game_mode/game_mode.ino
|   `-- game_mode_hard/game_mode_hard.ino
|-- Trial 1/
|   |-- electronics/electronics.ino
|   `-- mechanical/mechanical.ino
|-- Trial 2/
|   |-- button_mode.ino
|   `-- Tuning code/
|       |-- line_tracking_tune/
|       |-- chain_mode_tune/
|       |-- dead_reckoning_test/
|       |-- gate_rfid_ramp_test/
|       |-- revival_test/
|       `-- other subsystem test sketches
`-- docs/
    |-- software-overview.md
    |-- flowcharts.md
    `-- calibration.md
```

## Hardware Pin Map

| Subsystem | Pins / bus | Notes |
|---|---:|---|
| Arduino | GIGA R1 WiFi | Main controller. |
| Motoron drive controllers | Wire1, addresses 18 and 17 | `mc1` and `mc2`, channels 2/3 drive both sides. |
| Seed dispenser | Motoron 17, channel 1 | Final seed speed `150`, duration `550 ms`. |
| Raw IR line sensors | 23-31 | RC timing reads, no QTRSensors library needed. |
| RFID reader | Wire2, reset `-1` | Address auto-scanned at startup. |
| MPU6050 | Wire | Gyro turns and ramp tilt. |
| Ultrasonic front | trig 37, echo 36 | Door, obstacle, revival approach. |
| Ultrasonic left | trig 41, echo 40 | Ramp/tunnel wall following. |
| Ultrasonic right | trig 39, echo 38 | Ramp/tunnel wall following. |
| Bumper/contact inputs | 22 and 33 | Revival contact; green LED while pressed. |
| Kill switch | 49 | Highest-priority immediate stop. |
| LEDs | red 46, green 47 | Red default; green on bumper contact or calibration. |
| Encoders | 52/50 and 53/51 | Node-distance odometry. |

## Required Libraries

Install these in Arduino IDE Library Manager or equivalent:

- `Motoron` by Pololu
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `MFRC522_I2C`
- `MiniMessenger`

## Upload and Run

1. Open `Finals_game_mode/game_mode/game_mode.ino` in Arduino IDE for normal difficulty, or `Finals_game_mode/game_mode_hard/game_mode_hard.ino` for hard difficulty.
2. Select board `Arduino GIGA R1 WiFi` and the correct serial port.
3. Upload the sketch.
4. Keep the robot still during gyro calibration.
5. During raw IR calibration, move the sensor bar by hand across black line and white floor while the green LED is on.
6. The robot auto-starts the game sequence after setup.
7. Pin 49 is the hardware kill switch. Pressing it stops all motors and prevents automatic resume.

## Final Behaviour Summary

1. Chain entry: raw IR base approach, RFID airlock tag detection, existing MiniMessenger airlock request format, server accept, ultrasonic door-clear confirmation.
2. Ramp/tunnel: once the door is open, the robot drives through the door and uses ultrasonic wall following plus tilt detection for the ramp.
3. Arena planting: 9x9 node grid, encoder/gyro node movement, RFID fertility checks, one seed per fertile unplanted tag, max five seeds.
4. Emergency: server emergency interrupts the mission and routes to the top tunnel/exit node unless the hardware kill switch is active.
5. Revival: server distress request interrupts planting, the robot approaches with ultrasonic, taps by bumper contact, sends `reviveRequest`, then resumes.
6. Hard mode: obstacle avoidance runs only during arena node movement, using the same dead-reckoning node turns.

## MiniMessenger Messages Used

| Direction | Message | Purpose |
|---|---|---|
| robot -> server | `type=register` | Periodic registration. |
| robot -> server | `STATUS:GAME_MODE state=<state> enable=<n> running=<n>` | Periodic status broadcast. |
| robot -> server | `type=openAirlock airlock=A tag_id=<uid> team_id=6 board_id=LEAK` | Existing tested airlock request format. |
| server -> robot | `type=openAirlockReply ... accepted=true` | Door permission. |
| robot -> server | `type=isFertile tag_id=<uid> board_id=LEAK` | Ask if a node is fertile and unplanted. |
| server -> robot | `type=isFertileReply fertile=true planted=false ...` | Planting decision. |
| robot -> server | `type=seedPlanted tag_id=<uid> board_id=LEAK` | Notify server after seed drop. |
| server -> robot | `type=distress ... robot0=team.board,x,y` | Revival target. |
| robot -> server | `type=reviveRequest target_team=<team> target_board=<board>` | Notify successful revival. |
| server -> robot | `type=emergency` or 6-byte emergency flag | Return-to-base/top-tunnel behaviour. |

## Testing Evidence Summary

The final game sketches are compile-verified. The full final arena game has not been fully tested end-to-end. Trial 2 subsystem tests provide evidence for the major behaviours integrated into the finals code:

- raw IR line following and branch sensing
- RFID reader and airlock tag read
- MiniMessenger airlock request/reply
- gyro 90-degree turns
- encoder counts per 25 cm node
- dead-reckoning route
- ramp/tunnel wall following
- seed dispenser duration
- revival approach and bumper contact
- button/LED diagnostic

See `docs/calibration.md` and the `Trial 2/Tuning code/*/serial_output.txt` files for logs.

## Supporting Documentation

- `docs/software-overview.md`: architecture and component interactions.
- `docs/flowcharts.md`: behaviour flowcharts matching the final implementation.
- `docs/calibration.md`: testing/calibration evidence and honest limitations.
- `docs/viva_answers.md`: removable viva preparation notes.
