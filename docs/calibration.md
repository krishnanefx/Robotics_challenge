# Testing & Calibration Evidence — Group 6

---

## QTR Line Sensor Calibration

**Method:** On startup the robot runs `qtr.calibrate()` 200 times over 4 seconds (20 ms per
sample). During this window the sensor array is manually swept back and forth across the black
tape line so every sensor sees both extremes.

**What worked:**
- Calibration consistently completes within the 4-second window.
- After calibration, `sensorValues[i]` for sensors directly over the line reads ~900-1000;
  sensors off the line read ~0-50.
- The weighted error formula (`sum(sensorValue * weight)`) produces a reliable correction
  signal for proportional control.

**What did not work / known issues:**
- If powered on with the sensor facing a non-white surface, calibration produces unusable
  min/max values. Robot must start on a white surface with the line visible.
- Direct sunlight can saturate sensors and shift calibration. All testing done under indoor
  lab lighting.

---

## Gyro Bias Calibration

**Method:** `rezeroGyroBias()` samples 500 gyro Z readings over ~1.2 s. If variance
> 0.05^2 rad^2/s^2 (robot is moving or vibrating), calibration is skipped.

**Typical Serial output:**
```
[GYRO] Calibrating — keep robot still...
[GYRO] Bias: 0.00312
```

**What worked:**
- Bias values consistently in the range +/-0.01 rad/s when robot is stationary.
- Re-zero called after every turn corrects for thermal drift between manoeuvres.

**What did not work:**
- If vibration (nearby motors, uneven surface) is present at startup, calibration is skipped
  and the previous (zero) bias is used, causing slight heading error.

---

## Turn Accuracy — TURN_SCALE Tuning

**Method:** Robot commanded to turn 90 degrees right four times (should return to start
heading). Deviation measured visually.

| TURN_SCALE | Observed behaviour          | Outcome          |
|------------|-----------------------------|------------------|
| 1.15 (encoder) | ~5-10 degrees undershoot/turn | Too short     |
| 0.90 (gyro, initial) | ~2-3 degrees overshoot | Slightly over |
| 0.87 (gyro, tuned) | Visually close to 90 degrees | Good start   |

The gyro-based approach integrates actual rotation, so it is less sensitive to wheel slip
than the encoder method. TURN_SCALE < 1.0 stops the motor slightly early; coasting carries
the robot to the full angle.

**Recommendation:** Start at 0.90, adjust by +/-0.03 per run until four consecutive 90
degree turns return the robot to the original heading.

---

## Motor Speed / MIN_SPEED

**Result:** Below 400 motor counts, drive motors stall and hum without moving on the test
surface. 400 is the hard floor for all motion commands (`MIN_SPEED = 400`).

At `DRIVE_SPEED = 600`, straight-line drive is stable and the heading correction loop
(KP = 80) keeps the robot within ~5 degrees of target heading over a 25 cm node.

---

## Ultrasonic Distance Readings

Object placed at known distances; `readDistance()` called 10 times and averaged.

| Actual distance | Front  | Left   | Right  |
|-----------------|--------|--------|--------|
| 10 cm           | 10-11  | 10-12  | 10-11  |
| 25 cm           | 24-26  | 25-27  | 24-26  |
| 50 cm           | 49-52  | 50-53  | 49-51  |
| No object       | 999    | 999    | 999    |

Readings consistent within +/-2 cm at all competition-relevant distances (8 cm door,
20 cm tunnel wall, 25 cm obstacle trigger, 40 cm revival approach).

**Known issue:** Angled surfaces can produce false readings. The 999 cm timeout return
reliably indicates open space.

---

## RFID Detection

- Reliable detection within ~3 cm directly above the reader.
- Tilted card (>30 degrees) reduces range to ~1 cm.
- The 200-iteration post-detection skip loop reliably prevents double-reads.
- At full drive speed (600), the robot sometimes passes the tag before detection.
  Reduced speed near known tag locations improves reliability.

---

## MQTT / WiFi Comms

**Setup:** PhaseSpaceNetwork_2.4G, broker 192.168.0.74:1883.

**What worked:**
- Heartbeat watchdog (3 s) reliably stops the robot if WiFi drops.
- `openAirlockReply accepted=true airlock=A` correctly transitions to LINE_FOLLOWING.
- Status broadcast every 2 s visible in the lab dashboard.

**Known issues:**
- First connection can take 5-10 s if the broker is busy. Robot stays stopped (waiting
  for button press) during this window, so no impact on operation.
- 6-byte binary team status fields (queue/busy) are parsed but not yet acted on.

---

## Obstacle Avoidance

**Method:** Box obstacle placed 20 cm ahead. `runObstacleMode()` run 5 times.

- 5/5 successful swerves on flat surface.
- 60-second timeout never triggered in testing.
- `driveForwardDist()` timed-drive fallback triggered once when front sensor read 999
  during swerve — robot still completed the path correctly.

**Known issues:** Turn accuracy during swerve affected by wheel slip on low-grip floors.

---

## What We Would Improve With More Time

1. **PID for line following** — add derivative term to reduce oscillation at intersections.
2. **Obstacle detection in all modes** — currently a separate mode; integrating it as a
   continuous check would be safer.
3. **RFID position correction in dead reckoning** — tags are logged but do not currently
   reset the position estimate. Using them as ground truth would improve multi-node accuracy.
4. **Revival target identification** — `sendReviveRequest("?","?")` needs target team/board
   once the other robot's identity is known from a distress broadcast.
5. **LED pin assignment** — `redPin` and `greenPin` in `button_mode_dr.ino` still set to -1
   pending physical wiring confirmation.
