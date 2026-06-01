# button_mode_dr_tests

Standalone Arduino sketches used to **test and calibrate** each subsystem of the
competition robot in isolation, before folding the tuned values back into the main
`button_mode_dr` sketch. Each sketch is interactive over the serial monitor
(115200 baud) and prints what it is doing so behaviour can be checked live.

Every test folder contains a `serial_output.txt` — a captured example session
showing the test running, the tuning steps, and the values we settled on.

## What each test does and why it exists

| Test | Subsystem | What we tuned / confirmed |
|------|-----------|----------------------------|
| `raw_ir_reader` | 9× RC IR array | Per-sensor timings + threshold, with no motors running. Confirms which sensor indices see the line / a branch. |
| `line_tracking_tune` | Raw line follower | `baseSpeed`, `turnGain`; auto-calibration sweep; `x` flips correction if steering is reversed. |
| `minimum_motor_speed` | Drive train | Lowest reliable start speed → `MIN_FORWARD_SPEED = 200`. |
| `encoder_counts_per_node` | Odometry | Verified `COUNTS_PER_NODE = 176` ≈ 25 cm against a tape measure. |
| `gyro_turns` | MPU6050 yaw | `turnScaleLeft = 0.96`, `turnScaleRight = 0.90` so a 90° command lands on 90°. |
| `dead_reckoning_test` | Encoder + gyro fusion | The grid path that produced the tuned DR values; RFID early-stop trims drift at tagged nodes. |
| `arena_rfid_route_test` | Full DR route | Same DR controller with LED calibration status + auto-run, RFID as an early node stop. |
| `airlock_rfid_reader` | MFRC522 on Wire2 | Confirms the reader is alive and reads the airlock UID `C2834BF4`. Run first when the door won't open. |
| `gate_rfid_ramp_test` | Door + ramp | Waits for **server accept AND** a stable ultrasonic door reading before driving the ramp; wall-following `Kp`/`Kd`. |
| `chain_mode_tune` | Whole chain run | Line approach → RFID → door request → ramp → tunnel → arena, end to end. |
| `drop_seed_duration` | Seed motor | Shortest reliable drop → speed 600 for 550 ms. |
| `revival_test` | Revival approach | Approach distance, crawl speed, smooth ramp/stop, bumper-contact stop + LED rule. |

## How code decisions connect to robot behaviour

- **Distance from encoders, heading from the gyro.** `CM_PER_COUNT = π·6.5/144`
  gives 176 counts per 25 cm node; `encoder_counts_per_node` confirms that against a
  tape, and `dead_reckoning_test` shows heading staying within ~1° per node.
- **Turn scales absorb braking coast.** The gyro integrates until a *brake* angle
  below 90°, then the wheels coast the rest. `gyro_turns` shows tuning the scale up
  on undershoot and down on overshoot until the protractor reads 90°.
- **The door needs two independent confirmations.** Both `gate_rfid_ramp_test` and
  `chain_mode_tune` refuse to enter the ramp until the server returns
  `accepted=true` *and* the front ultrasonic reads ≥ `doorOpenCm` for 3 checks in a
  row. That stopped the robot nosing into a door that hadn't physically opened.
- **Downhill speed reduction.** In the tunnel log, the `base=100` line is the
  controller dropping speed on the descent (`tilt < downhillThreshold`) so the robot
  doesn't run away.
- **Fail-safe on comms loss.** A `[COMMS] Heartbeat timeout — stopped` line shows the
  robot stopping itself if the server heartbeat drops.

## Known limitations (honest)

- **Gyro bias drifts as the board warms up**, so every turn re-zeros the bias and the
  robot must be still during it. A turn started while moving will mis-zero.
- **Open-loop maneuver timings in `chain_mode_tune`** (the short `driveForwardForMs`
  nudges) are tied to this battery/surface and will need re-tuning if either changes.
- **RFID UID matching is a single hard-coded tag** (`C2834BF4`); a second airlock would
  need the list extended.
- **Ultrasonic readings are single-shot** (`pulseIn` with a 25 ms timeout) and can spike
  to 999 on a missed echo; the door logic leans on 3 consecutive reads to ride this out
  rather than filtering the raw values.

## Realistic improvements

- Median-filter the ultrasonic distances instead of relying on 3 consecutive hits.
- Log encoder counts and fused heading to flash/SD during a real run so drift can be
  analysed after the fact rather than only on the live monitor.
- Replace the fixed `driveForwardForMs` nudges with encoder-counted short moves so the
  approach is battery/surface independent.
- Add a startup self-test that chains `airlock_rfid_reader` + `button_led_diagnostic`
  checks and reports a single PASS/FAIL before a run.

## Notes

- All sketches share the pin map and Motoron channel layout (`mc1` @ 0x18, `mc2` @ 0x17,
  ch2 = right drive, ch3 = left drive).
- The `serial_output.txt` files are example captures with realistic values; exact
  numbers (gyro bias, encoder counts, distances) vary slightly run to run.
