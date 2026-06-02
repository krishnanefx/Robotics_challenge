# Finals Behaviour Flowcharts

## Startup and Calibration

```text
Power on
  -> Serial.begin(115200)
  -> MiniMessenger.begin(ssid, pass, broker, port, group, board)
  -> configure pins: ultrasonics, bumpers, kill switch 49, LEDs, encoders
  -> Wire/Wire1/Wire2 begin
  -> Motoron setup and stopMotors()
  -> scan RFID on Wire2 and initialise reader if found
  -> initialise MPU6050 if found
  -> attach encoder interrupts
  -> raw IR hand calibration
       green LED on while sampling
       red LED when done
  -> gyro bias calibration while robot is still
  -> start GAME_CHAIN_ENTRY
```

## Main Loop Safety Priority

```text
loop()
  -> messenger.loop()
  -> kill switch pin 49 LOW OR WiFi disable latched?
       yes -> stopMotors(), blink red LED, latch stopped, return
  -> heartbeat timeout?
       yes -> stopMotors(), hold position, return
  -> server emergency requested?
       yes -> GAME_EMERGENCY_EXIT
  -> distress target available?
       yes -> runRevivalStep()
  -> dispatch current game state
```

## Chain Entry and Ramp

```text
GAME_CHAIN_ENTRY
  CH_APPROACH:
    follow raw IR line
    read RFID during approach
    if tag == C2834BF4 -> send existing openAirlock format
    continue route until final line loss

  CH_WAIT_ENTRY:
    stop motors
    wait for openAirlockReply accepted=true
    require front ultrasonic >= doorOpenCm for 3 readings

  CH_TUNNEL:
    drive through opened door
    detect ramp/tunnel by tilt or close walls
    wall-follow with left/right ultrasonics
    downhill uses lower P gain and reduced base speed
    when ramp/tunnel clears -> GAME_INITIAL_PATTERN
```

## Arena Planting

```text
GAME_INITIAL_PATTERN
turn right
go down
turn left
go forward
  -> GAME_SERPENTINE

GAME_SERPENTINE
  at each node:
    read RFID tag if present
    tag already seen/planted? -> skip
    send isFertile request
    wait briefly, retry if timeout
    fertile=true and planted=false?
      yes -> seed motor speed 150 for 550 ms
             send seedPlanted
             increment local seed count
      no -> continue

  if not at vertical edge:
    move one node in vertical direction
  else if more columns available:
    move one node east
    reverse vertical direction
  else:
    route to top tunnel node

  if seedsPlanted == 5:
    route to top tunnel node
```

## Emergency Return

```text
server type=emergency OR binary emergency flag
  -> different from WiFi disable kill
  -> stopMotors()
  -> emergencyExitRequested = true
  -> GAME_EMERGENCY_EXIT

GAME_EMERGENCY_EXIT
  use dead-reckoning node moves
  move north until y == top row
  move west until x == 0
  face west toward top tunnel node
  stopMotors()
  -> GAME_TOP_TUNNEL_STOP
```

## Kill Switches

```text
pin 49 LOW
  -> killLatched = true
  -> running = false
  -> stop all Motoron channels 1..3 on both controllers
  -> blink red LED
  -> no automatic resume

MiniMessenger heartbeat enable=0 OR type=disable enabled=false
  -> wifiKillLatched = true
  -> enable = 0
  -> running = false
  -> stop all Motoron channels 1..3 on both controllers
  -> blink red LED
  -> ignore later enable=1 until reset
```

## Revival

```text
type=distress received
  -> parse robot0=team.board,x,y
  -> store target team and board

if target available and no emergency/kill:
  front ultrasonic > 40 cm?
    yes -> ramp toward approachSpeed
    no  -> ramp toward crawlSpeed
  bumper 22 or 33 pressed?
    yes -> smooth stop
           send reviveRequest target_team/target_board
           clear target
           resume game path
```

## Hard Mode Obstacle Avoidance

```text
game_mode_hard only, arena movement only
  before node move:
    front ultrasonic <= obstacle threshold?
      yes -> choose side with more clearance
             turn 90 away
             drive 1 node
             turn back
             drive 2 nodes
             turn toward lane
             drive 1 node
             turn forward
             drive 1 node
      no  -> normal node move
```
