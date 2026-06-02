# Software Overview - Finals Game Mode

## High-Level Architecture

```text
setup()
  |-- initialise WiFi/MiniMessenger, Motoron, RFID, MPU6050, ultrasonics, encoders, LEDs
  |-- raw IR hand calibration
  |-- gyro yaw calibration
  `-- start GAME_CHAIN_ENTRY

loop()
  |-- messenger.loop()
  |-- messenger.loop() parses WiFi disable/emergency/distress
  |-- killSwitchActive()       highest priority, pin 49 or WiFi disable latch
  |-- commsHeartbeatCheck()
  |-- updateLEDs()             red blink while killed
  |-- emergency/revival checks
  `-- game state dispatcher
```

The final sketches are standalone `.ino` files so Arduino IDE can upload them directly. The code is organised into functions rather than one large loop: comms parsing, raw line sensing, chain/ramp behaviour, dead-reckoning movement, fertility/seed logic, revival, and obstacle avoidance are separated.

## Main Components

| Component | Responsibilities |
|---|---|
| Raw IR line sensing | Reads pins 23-31 using RC discharge timing. Used for base line approach and branch detection. |
| MiniMessenger comms | Handles heartbeat, airlock reply, emergency, distress, fertility replies, and outbound status/messages. |
| Chain/ramp state machine | Moves from base line approach to door wait, then ramp/tunnel wall following, then arena game. |
| Dead-reckoning movement | Uses encoders for 25 cm node distance and gyro Z for 90-degree turns. |
| Fertility/seed subsystem | Reads RFID tags, asks server if fertile, drops one seed, sends `seedPlanted`, caches tag IDs. |
| Safety subsystem | Pin 49 kill latch, WiFi disable kill latch, red blink kill indicator, server emergency return, heartbeat timeout stop. |
| Revival subsystem | Parses distress target, approaches with ultrasonic, detects bumper contact, sends `reviveRequest`. |
| Hard mode obstacle subsystem | Checks front ultrasonic during arena node moves and runs a node-based box swerve. |

## State Machines

### Game State

```text
GAME_CHAIN_ENTRY
  -> existing chain/ramp code completes
  -> GAME_INITIAL_PATTERN
  -> GAME_SERPENTINE
  -> GAME_EMERGENCY_EXIT when 5 seeds planted, sweep complete, or emergency
  -> GAME_TOP_TUNNEL_STOP
```

### Chain State

```text
CH_APPROACH
  raw line following + RFID tag read
  requestAirlock("A", tag)
  continue line route until final line loss

CH_WAIT_ENTRY
  stop motors
  wait for server accepted=true and stable ultrasonic door-open reading

CH_TUNNEL
  drive through door and ramp/tunnel using tilt + side ultrasonics

CH_ARENA
  hand off to arena game path
```

## Data Flow

```text
Sensors
  raw IR -> line error / branch detection
  RFID -> airlock tag or arena fertility tag
  MPU6050 -> gyro turn integration and ramp tilt
  encoders -> 25 cm node distance
  ultrasonics -> door/ramp/obstacle/revival decisions
  bumpers -> revival contact and LED state

Decision logic
  safety priority -> game state -> movement command

Actuators
  Motoron ch2/ch3 -> drive motors
  Motoron 17 ch1 -> seed dispenser
  LEDs 46/47 -> calibration, bumper-contact status, and blinking red kill indication
```

## Safety Priority

1. Kill switch: pin 49 LOW or MiniMessenger WiFi disable/heartbeat `enable=0` stops all Motoron channels immediately, latches stopped, and blinks the red LED.
2. Server emergency: stop current task and route to the top tunnel node.
3. Heartbeat timeout: stop and hold.
4. Revival distress: interrupt planting unless emergency/kill is active.
5. Hard-mode obstacle avoidance.
6. Normal planting/game logic.

## WiFi Disable Kill

MiniMessenger documents `type=disable enabled=false reason=stranded` as the server command for disabling a robot, and heartbeat `enable=0` as a movement stop. In the final code both are treated as a WiFi kill switch, not just a pause. The robot stops every Motoron channel, sets `wifiKillLatched = true`, ignores later heartbeat `enable=1`, and blinks the red LED until power/reset. This gives the same visible state whether the robot was killed by pin 49 or by WiFi. Server `type=emergency` remains different: it routes to the top tunnel node instead of latching dead.

## Final Code Variants

- `game_mode`: normal finals behaviour without obstacle swerve.
- `game_mode_hard`: identical code path with `HARD_MODE_OBSTACLES = true`, enabling obstacle checks only during arena node movement.
