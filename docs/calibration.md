# Testing and Calibration Evidence

This file summarises the evidence used for the final game sketches. The final integrated game modes were compile-verified. The individual subsystems were tested during Trial 2 using the sketches and serial logs under `Trial 2/Tuning code/`.

## Final Sketch Compile Evidence

Both finals sketches compile for `arduino:mbed_giga:giga`:

```text
game_mode:      Sketch uses 321380 bytes (16%)
game_mode_hard: Sketch uses 322004 bytes (16%)
```

The final arena game has not been claimed as fully end-to-end arena-tested. It is integrated from the tested subsystems below.

## Raw IR Line Tracking

Evidence files:
- `Trial 2/Tuning code/raw_ir_reader/serial_output.txt`
- `Trial 2/Tuning code/line_tracking_tune/serial_output.txt`

Final code uses raw RC timing reads on pins 23-31, not QTRSensors. The tuning sketches were used to confirm sensor ordering, branch-side detection, and threshold behaviour. Hand calibration samples min/max values while the robot is moved over black line and white floor.

Important final values:

```cpp
RAW_SENSOR_TIMEOUT_US = 2500
rawLineThreshold = 200
lineBaseSpeed = 300
lineTurnGain = 1.0
branchSearchSpeed = 200
```

## Minimum Motor Speeds

Evidence file:
- `Trial 2/Tuning code/minimum_motor_speed/serial_output.txt`

Observed reliable movement thresholds:

```cpp
MIN_FORWARD_SPEED = 200
MIN_TURN_SPEED = 400
MAX_MOTOR_SPEED = 660
```

These constants prevent commands below the motor stall floor during dead-reckoning and turns.

## Encoder Counts Per Node

Evidence file:
- `Trial 2/Tuning code/encoder_counts_per_node/serial_output.txt`

The arena grid uses 25 cm node spacing. With wheel diameter 6.5 cm and 144 encoder counts per revolution, the final value is approximately:

```cpp
COUNTS_PER_NODE = 176
DRIVE_SPEED = 300
DRIVE_KP = 80.0
```

RFID early-stop was also tested in the dead-reckoning sketch to reduce node overshoot when tags are detected.

## Gyro Turns

Evidence files:
- `Trial 2/Tuning code/gyro_turns/serial_output.txt`
- `Trial 2/Tuning code/dead_reckoning_test/serial_output.txt`

The final code uses MPU6050 Z gyro integration for 90-degree turns. The motor stops before 90 degrees so wheel coast carries the robot to the target.

```cpp
TURN_SCALE_LEFT = 0.96
TURN_SCALE_RIGHT = 0.90
TURN_FAST = 660
TURN_SLOW = 566
```

The gyro bias is recalibrated while stationary before/after key turns to reduce drift.

## Airlock RFID and Ramp

Evidence files:
- `Trial 2/Tuning code/airlock_rfid_reader/serial_output.txt`
- `Trial 2/Tuning code/chain_mode_tune/serial_output.txt`
- `Trial 2/Tuning code/gate_rfid_ramp_test/serial_output.txt`

The known airlock tag used by the working server setup is:

```cpp
AIRLOCK_A_TAG_ID = "C2834BF4"
```

The final code uses the tested airlock message format:

```text
type=openAirlock airlock=A tag_id=<uid> team_id=6 board_id=LEAK
```

The robot does not enter the ramp just because the server accepts. It waits for stable ultrasonic door clearance:

```cpp
doorOpenCm = 18
required clear readings = 3
```

Ramp wall-following was softened to avoid wall impacts:

```cpp
tunnelBaseSpeed = 400
wallKpUp = 25.0
wallKpDown = 10.0
wallKd = 3.0
correction cap = +/-220
```

## Seed Dispenser

Evidence file:
- `Trial 2/Tuning code/drop_seed_duration/serial_output.txt`

The final game-mode seed speed was changed for finals integration:

```cpp
SEED_SPEED = 150
SEED_DURATION_MS = 550
```

The final game logic drops at most one seed per fertile unplanted RFID tag and sends `seedPlanted` after the local drop.

## Revival

Evidence file:
- `Trial 2/Tuning code/revival_test/serial_output.txt`

Final behaviour:

- red LED by default
- green LED while bumper 22 or 33 is pressed
- fast approach until front ultrasonic sees the target inside approximately 40 cm
- crawl into bumper contact
- smooth motor stop
- send `type=reviveRequest target_team=... target_board=...`

## Obstacle Avoidance

Evidence source:
- old obstacle mode and hard-mode integration

Hard mode only checks obstacles during arena node movement. It does not interfere with chain approach, door wait, or ramp traversal. The swerve uses the same dead-reckoning node turns as the arena game.

## Honest Limitations

- The final `game_mode` and `game_mode_hard` sketches have been compile-verified but not claimed as full arena end-to-end tested.
- RFID detection depends on the reader passing close enough over each tag.
- Gyro bias calibration requires the robot to be still.
- Ultrasonic readings can miss angled surfaces; door opening uses three stable clear readings to reduce false starts.
- The emergency return uses the robot's internal grid estimate, so accumulated drift can affect how accurately it reaches the top tunnel node.
