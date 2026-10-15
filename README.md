# Robotics Challenge: an autonomous seed-planting robot

An Arduino GIGA robot that crosses an arena on its own: it follows a line where there is one, switches to gyro and encoder dead reckoning where there isn't, checks RFID tags for fertile ground and plants seeds, while talking to the game server over MQTT. Built by Group 6 for UCL's COMP0204 Robotics Challenge 2026, where we placed **4th of 14 teams**. This repository is our code.

## Final Code for Finals Day

Use these sketches for the final robot:

| Sketch | Purpose |
|---|---|
| `Finals_game_mode/game_mode/game_mode.ino` | Main finals code: chain/ramp entry, fixed hybrid line/dead-reckoning arena course with RFID fertility checks and seed planting, revival support. |
| `Finals_game_mode/game_mode_hard/game_mode_hard.ino` | Hard-difficulty version, built on an earlier revision of the game code: chain/ramp entry, 9x9 serpentine sweep with arena-only obstacle avoidance, five-seed cap, and emergency route to the top tunnel node. |

The two sketches have diverged and are not the same code with a flag: they differ in route, speeds, calibration timing, seed motor wiring and start/kill behaviour (see below).

The Trial 1 sketches and `Trial 2/button_mode.ino` (a pin-49 mode cycle wrapping the tuning sketches) are earlier milestones. The Trial 2 tuning sketches are kept as evidence and calibration history; see `Trial 2/Tuning code/README.md`.

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
|       |-- README.md
|       |-- airlock_rfid_reader/        |-- gyro_turns/
|       |-- arena_rfid_route_test/      |-- line_tracking_tune/
|       |-- button_led_diagnostic/      |-- minimum_motor_speed/
|       |-- chain_mode_tune/            |-- raw_ir_reader/
|       |-- dead_reckoning_test/        `-- revival_test/
|       |-- drop_seed_duration/
|       |-- encoder_counts_per_node/
|       `-- gate_rfid_ramp_test/
|           (each folder: <name>.ino + serial_output.txt)
`-- docs/
    |-- software-overview.md
    |-- flowcharts.md
    `-- calibration.md
```

## Hardware Pin Map

Shared by both finals sketches unless noted.

| Subsystem | Pins / bus | Notes |
|---|---:|---|
| Arduino | GIGA R1 WiFi | Main controller. |
| Motoron drive controllers | Wire1, addresses 18 and 17 | `mc1` and `mc2`, channels 2/3 drive. |
| Seed dispenser | Motoron channel 1 | Speed `150` in both. `game_mode`: `mc1` (address 18), `650 ms`. `game_mode_hard`: `mc2` (address 17), `550 ms`. |
| Raw IR line sensors | 23-31 (9 sensors) | RC timing reads, no QTRSensors library needed. |
| RFID reader (MFRC522, I2C) | Wire2, reset `-1` | Address auto-scanned at startup. |
| MPU6050 | Wire | Gyro turns and ramp tilt. |
| Ultrasonic front | trig 37, echo 36 | Door, obstacle, revival approach. |
| Ultrasonic left | trig 41, echo 40 | Ramp/tunnel wall following. |
| Ultrasonic right | trig 39, echo 38 | Ramp/tunnel wall following. |
| Bumper/contact inputs | 22 and 33 | Revival contact; green LED while pressed. |
| Start / kill button | 49 | Highest-priority immediate stop (see Upload and Run). |
| LEDs | red 46, green 47 | Red while running, green on bumper contact, red blinking when killed. In `game_mode`, green also means connected and ready to start. |
| Encoders | 52/50 and 53/51 | Node-distance odometry. |

## Required Libraries

Install these in Arduino IDE Library Manager or equivalent:

- `Motoron` by Pololu
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `MFRC522_I2C`
- `MiniMessenger`

The Trial 1 sketches additionally use `QTRSensors` (Pololu) and `TFLI2C` (TF-Luna LiDAR).

WiFi credentials live in `arduino_secrets.h` next to each sketch (gitignored): copy `arduino_secrets.h.example`, rename it, and fill in `SECRET_SSID` / `SECRET_PASS`. The broker address, group (`6`) and board ID (`LEAK`) are hard-coded near the top of each sketch.

## Upload and Run

1. Open `Finals_game_mode/game_mode/game_mode.ino` in Arduino IDE for normal difficulty, or `Finals_game_mode/game_mode_hard/game_mode_hard.ino` for hard difficulty.
2. Select board `Arduino GIGA R1 WiFi` and the correct serial port.
3. Upload the sketch.
4. Keep the robot still during gyro calibration.
5. During raw IR calibration (both LEDs on in `game_mode`, 2 s; 6 s in `game_mode_hard`), move the sensor bar by hand across the black line and white floor.
6. Start behaviour differs between the sketches:
   - `game_mode`: after IR calibration the LEDs go off for 3 s; put the robot down for gyro calibration. It then waits; the green LED shows it is connected to the server and ready. Press pin 49 to start. Once the button has been released, the next press latches a kill (red LED blinks); a fresh press clears the latch.
   - `game_mode_hard`: auto-starts the game sequence after setup. Pin 49 is a kill switch only; the latch is not cleared by the button, so reset the board to run again.
7. A server `type=disable`/`type=kill` or heartbeat `enable=0` also latches a stop; a heartbeat timeout (3 s) stops the robot and holds.
8. The serial monitor runs at 115200 baud and accepts single-key commands (`s` stops); see `printHelp()` in each sketch.

## Final Behaviour Summary

Both sketches:

1. Chain entry: raw IR base approach, RFID airlock tag detection (`C2834BF4`), MiniMessenger airlock A request, server accept, then ultrasonic door-clear confirmation (3 consecutive readings) before entering the ramp.
2. Ramp/tunnel: once the door is open, the robot drives through the door and uses left/right ultrasonic wall following plus MPU6050 tilt detection for the ramp, with a reduced downhill speed.
3. Planting: at each node the RFID tag is read and `isFertile` is sent; a seed is dropped only if the reply is `fertile=true` and `planted=false`, followed by `seedPlanted`.
4. Revival: a server `distress` message (or a bumper press in `game_mode`) interrupts the arena run; the robot approaches with ultrasonic, stops on bumper contact, sends `reviveRequest`, then resumes.

`game_mode`:

5. Arena course: fixed route of 9 nodes forward, turn left, 2 nodes over the incline, then 4 nodes forward and stop. The first 4 nodes are line-tracked; the rest use encoder/gyro dead reckoning with RFID node detection. One seed per fertile tag; no overall seed cap in this sketch.
6. Emergency: a server emergency stops the motors and ends the run (no automatic route out).

`game_mode_hard`:

5. Arena sweep: a short initial pattern, then a serpentine sweep of the 9x9 node grid using encoder/gyro node moves, until five seeds are planted or the sweep ends. Tags already seen or planted are skipped.
6. Obstacle avoidance runs only during arena node movement (front ultrasonic <= 25 cm), using the same dead-reckoning node turns.
7. Emergency, five seeds planted, or sweep complete: routes to the top tunnel node and stops.

## MiniMessenger Messages Used

| Direction | Message | Purpose |
|---|---|---|
| robot -> server | `type=register` | Registration every 10 s. |
| robot -> group | `STATUS:GAME_MODE state=<state> enable=<n> running=<n>` (`game_mode` sends `mode=<mode>` instead of `state=`) | Status broadcast every 2 s. |
| robot -> server | `type=openAirlock airlock=A tag_id=<uid> team_id=6 board_id=LEAK` | Airlock request (retried every 3 s if denied). |
| server -> robot | `type=openAirlockReply ... accepted=true` | Door permission. |
| robot -> server | `type=isFertile tag_id=<uid> board_id=LEAK` | Ask if a node is fertile and unplanted. |
| server -> robot | `type=isFertileReply fertile=true planted=false ...` | Planting decision. |
| robot -> server | `type=seedPlanted tag_id=<uid> board_id=LEAK` | Notify server after seed drop. |
| server -> robot | `type=distress ... robot0=team.board,x,y` | Revival target. |
| robot -> server | `type=reviveRequest target_team=<team> target_board=<board>` | Notify successful revival. |
| server -> robot | `type=emergency` or 6-byte emergency flag | Stop (`game_mode`) or route to top tunnel (`game_mode_hard`). |
| server -> robot | `type=heartbeat enable=<0/1>`, `type=disable`, `type=kill`, `type=enable` | Heartbeat watchdog and remote kill/enable. |

## Testing Evidence Summary

The final game sketches are compile-verified. 

For the final full arena run (`game_mode`), several limitations were found during real robot testing.

The main issue was that the robot did not reliably stay aligned when moving node-to-node using only dead reckoning. It drifted between nodes, especially over longer straight sections, so the code was changed to use a hybrid approach: the robot line-tracks for the first four nodes where the line is available, then switches to gyro/dead-reckoning with RFID node detection for the remaining route. A full line-search recovery was also added, but because the arena does not always provide a usable line, the robot now falls back to RFID/encoder/gyro movement instead of stopping when the line cannot be reacquired.

To reduce the rightward drift during dead-reckoning movement, a drive trim value was added. The final code uses driveTrim = 18, which slightly increases the left-side motor speed and slightly reduces the right-side motor speed during straight node driving. This compensates for the robot’s tendency to drift right, which may be caused by small differences in motor output, wheel grip, chassis alignment, or weight distribution.

The arena route was changed to the current final strategy: 9 nodes forward, turn left, 2 nodes over the incline, then 4 nodes forward and stop. The first node is driven slower at 330 motor speed to reduce early misalignment. This was added because the robot was most sensitive to drift immediately after starting, before it had stabilised on the route. After the first node, the normal base speed is 440, and the incline section uses a boosted speed of 520, so the robot can climb more reliably.

Seeding was also changed to reduce uncertainty. Instead of blindly planting at every node, the robot now reads the RFID tag at the node and sends a fertility request to the server before dropping a seed. It only plants if the reply confirms that the node is fertile and not already planted. This prevents wasting seeds on invalid or already-used nodes, while still using the RFID tag to link each seed action to a specific arena location.

Ramp and tunnel behaviour was adjusted because the robot was not equally stable on flat ground and slopes. The downhill ramp speed was reduced separately from the normal tunnel speed, because moving too quickly downhill made the robot harder to control. Once the robot detects that it is inside the tunnel, it uses the left and right ultrasonic sensors for wall following. This lets it correct its position relative to the tunnel walls instead of relying only on dead reckoning while travelling through the ramp section.

Trial 2 subsystem tests provide evidence for the major behaviours integrated into the finals code:

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

- [`docs/software-overview.md`](docs/software-overview.md): architecture and component interactions.
- [`docs/flowcharts.md`](docs/flowcharts.md): behaviour flowcharts matching the final implementation.
- [`docs/calibration.md`](docs/calibration.md): testing/calibration evidence and honest limitations.
- [`Trial 2/Tuning code/README.md`](<Trial 2/Tuning code/README.md>): what each tuning sketch tests and the values settled on.
