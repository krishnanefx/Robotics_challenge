// ── Includes ──────────────────────────────────────────────────────────────────
#include <Motoron.h>
#include <Wire.h>
#include <MFRC522_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <QTRSensors.h>
#include <MiniMessenger.h>
#include <math.h>
#include <stdbool.h>


// ── RFID config ───────────────────────────────────────────────────────────────
#define RFID_WIRE      Wire2
#define RFID_RESET_PIN -1


// ── Comms config ──────────────────────────────────────────────────────────────
const char*    ssid   = "PhaseSpaceNetwork_2.4G";
const char*    pass   = "8igMacNet";
const char*    broker = "192.168.0.74";
const uint16_t port   = 1883;
const char*    group  = "6";
const char*    board  = "LEAK";

volatile int         enable   = 1;
static unsigned long lastSend = 0;

MiniMessenger messenger;


// ── Hardware objects ──────────────────────────────────────────────────────────
MFRC522_I2C     *rfid = nullptr;
Adafruit_MPU6050 mpu;
MotoronI2C       mc;
QTRSensors       qtr;
MotoronI2C       mc1(18);   // ch2 = right drive, ch3 = left drive
MotoronI2C       mc2(17);   // ch2 = right drive, ch3 = left drive, ch1 = seed motor


// ── Pin assignments ───────────────────────────────────────────────────────────
const int button1Pin    = 49;   // mode-cycle button / front bumper
const int button2Pin    = 47;   // secondary bumper
const int killSwitchPin = 45;   // hardware kill — stops robot instantly

// ⚠️  TODO: fill in actual pins once wired on the robot next week.
const int redPin   = -1;     // red   LED — blinks when stopped / emergency  (TBD)
const int greenPin = -1;     // green LED — solid when running               (TBD)

const int encoder1PinA = 52;
const int encoder1PinB = 50;
const int encoder2PinA = 53;
const int encoder2PinB = 51;

const uint8_t TRIG_F = 37, ECHO_F = 36;
const uint8_t TRIG_L = 41, ECHO_L = 40;
const uint8_t TRIG_R = 39, ECHO_R = 38;
const long    ECHO_TIMEOUT = 25000;


// ── Dead-reckoning physical constants ─────────────────────────────────────────
const float WHEEL_DIAM_CM   = 6.5f;
const float TRACK_WIDTH_CM  = 17.0f;
const float COUNTS_PER_REV  = 144.0f;
const float CM_PER_COUNT    = (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;
const int   COUNTS_PER_NODE = (int)(25.0f / CM_PER_COUNT);  // ~176 counts = 25 cm

const int   DRIVE_SPEED = 600;
const float DRIVE_KP    = 80.0f;
const int   TURN_FAST   = 700;
const int   TURN_SLOW   = 600;
// TURN_SCALE for gyro-based turns: < 1.0 stops the motors before reaching the
// full angle so coasting carries the robot to 90°. Decrease if overshooting,
// increase if falling short. Start at 0.90 and tune in 0.03 steps.
const float TURN_SCALE  = 0.90f;

// Minimum motor speed before stall — never command drive motors below this.
const int   MIN_SPEED   = 400;


// ── Complementary-filter heading state ────────────────────────────────────────
const float ALPHA = 0.98f;

float         fusedHeading  = 0.0f;
float         gyroBiasZ     = 0.0f;
unsigned long lastHeadingUs = 0;
long          prevEncL      = 0;
long          prevEncR      = 0;


// ── Encoder counts (written by ISRs in motion.ino) ────────────────────────────
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;


// ── Robot enable state ────────────────────────────────────────────────────────
// robotRunning: physical on/off (buttons / kill switch)
// enable:       MQTT-level enable (heartbeat / server commands)
// Robot moves only when both are true.
volatile bool robotRunning = false;

const unsigned long DEBOUNCE_MS = 50;
const unsigned long BLINK_MS    = 300;


// ── QTR sensor config ─────────────────────────────────────────────────────────
const uint8_t SensorCount = 9;
uint16_t      sensorValues[SensorCount];
uint8_t       sensorPins[SensorCount] = {23, 24, 25, 26, 27, 28, 29, 30, 31};
float         weights[SensorCount]    = {0.15, 0.12, 0.08, 0.04, 0, -0.04, -0.08, -0.12, -0.15};


// ── Path planning ─────────────────────────────────────────────────────────────
// direction index: 0=W  1=N  2=E  3=S
// CW (right) increments direction; CCW (left) decrements direction.
int  x         = 7;
int  y         = -1;
int  direction = 1;   // 0=W  1=N  2=E  3=S  — starts facing North
int  step      = 0;
int  pathx[5]  = {7, 7, 8, 8, 8};
int  pathy[5]  = {0, 1, 1, 2, 3};
char dirNames[4] = {'W', 'N', 'E', 'S'};
bool fertiles[5] = {true, true, false, true, false};
int  turns = 0;
bool inArena = false;  // set true when robot enters the arena (after airlock A)


// ── Chain-mode state machine ──────────────────────────────────────────────────
enum State { NAVIGATING, WAITING_AIRLOCK, LINE_FOLLOWING };
State chainState = NAVIGATING;


// ── Button / mode state ───────────────────────────────────────────────────────
int presses = 0;


// ── LED update (non-blocking) ─────────────────────────────────────────────────
// Green solid  = running + MQTT connected
// Red solid    = MQTT-disabled (server stop)
// Red blinking = physically stopped (kill switch / not yet started)

static unsigned long lastBlinkMs = 0;
static bool          redBlink    = false;

void updateLEDs() {
  if (redPin < 0 && greenPin < 0) return;   // pins TBD — skip entirely
  unsigned long now = millis();

  if (!robotRunning) {
    // Physically stopped — blink red
    if (greenPin >= 0) digitalWrite(greenPin, LOW);
    if (redPin   >= 0 && now - lastBlinkMs >= BLINK_MS) {
      lastBlinkMs = now;
      redBlink = !redBlink;
      digitalWrite(redPin, redBlink ? HIGH : LOW);
    }
  } else if (!enable) {
    // MQTT-disabled — red solid
    if (redPin   >= 0) digitalWrite(redPin,   HIGH);
    if (greenPin >= 0) digitalWrite(greenPin, LOW);
  } else if (messenger.isConnected()) {
    // Running + connected — green solid
    if (redPin   >= 0) digitalWrite(redPin,   LOW);
    if (greenPin >= 0) digitalWrite(greenPin, HIGH);
  } else {
    // Running but no MQTT yet — blink red slowly
    if (redPin >= 0 && now - lastBlinkMs >= 500) {
      lastBlinkMs = now;
      redBlink = !redBlink;
      digitalWrite(redPin, redBlink ? HIGH : LOW);
    }
    if (greenPin >= 0) digitalWrite(greenPin, LOW);
  }
}


// ══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  // LEDs (skipped until pins are assigned)
  if (redPin   >= 0) { pinMode(redPin,   OUTPUT); digitalWrite(redPin,   LOW); }
  if (greenPin >= 0) { pinMode(greenPin, OUTPUT); digitalWrite(greenPin, LOW); }

  Wire.begin();
  Wire1.begin();
  RFID_WIRE.begin();
  delay(100);

  // Buttons + kill switch
  pinMode(button1Pin,    INPUT_PULLUP);
  pinMode(button2Pin,    INPUT_PULLUP);
  pinMode(killSwitchPin, INPUT_PULLUP);

  // RFID — non-fatal; retries 5× then continues without it
  for (int attempt = 0; attempt < 5 && rfid == nullptr; attempt++) {
    byte rfidAddress = findI2CAddress();
    if (rfidAddress != 0) {
      rfid = new MFRC522_I2C(rfidAddress, RFID_RESET_PIN, &RFID_WIRE);
      rfid->PCD_Init();
      Serial.print("RFID at 0x");
      if (rfidAddress < 0x10) Serial.print("0");
      Serial.println(rfidAddress, HEX);
    } else {
      Serial.print("RFID not found, attempt "); Serial.println(attempt + 1);
      delay(500);
    }
  }
  if (rfid == nullptr) Serial.println("WARNING: Running without RFID");

  // Motoron — set bus before any other calls
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);
  for (int ch = 1; ch <= 3; ch++) {
    mc1.setMaxAcceleration(ch, 200);  mc1.setMaxDeceleration(ch, 300);
    mc2.setMaxAcceleration(ch, 200);  mc2.setMaxDeceleration(ch, 300);
  }

  // Ultrasonic
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  // MPU6050 — non-fatal; retries 5× then continues without it
  bool mpuOk = false;
  for (int attempt = 0; attempt < 5 && !mpuOk; attempt++) {
    if (mpu.begin()) { mpuOk = true; }
    else { Serial.print("MPU6050 not found, attempt "); Serial.println(attempt + 1); delay(500); }
  }
  if (!mpuOk) Serial.println("WARNING: Running without MPU6050");
  if (mpuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
  }

  // Encoders
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), countEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), countEncoder2, RISING);

  // Initial gyro bias calibration
  Serial.println("[GYRO] Calibrating — keep robot still...");
  rezeroGyroBias();
  lastHeadingUs = micros();
  Serial.print("COUNTS_PER_NODE = "); Serial.println(COUNTS_PER_NODE);

  // QTR
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setTimeout(2500);
  delay(500);
  Serial.println("Calibrating QTR — move sensor over line...");
  for (uint16_t i = 0; i < 200; i++) { qtr.calibrate(); delay(20); }
  Serial.println("Calibration done!");

  // Comms
  messenger.onMessage(onCommsMessage);
  messenger.begin(ssid, pass, broker, port, group, board);

  Serial.println("=== Ready ===");
  Serial.println("  0: Line follow   1: Chain (tunnel→airlock→line)   2: Line follow");
  Serial.println("  3: Dead reckoning   4: Robot revival   5: Obstacle avoidance");
}


// ══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
//  checkButton() handles: revival (btn1/btn2 when stopped), kill switch,
//                          mode advance (btn1 when running).
//  Robot only moves when enable && robotRunning are both true.
// ══════════════════════════════════════════════════════════════════════════════

void loop() {
  messenger.loop();
  commsHeartbeatCheck();
  sendStatusUpdate();
  checkButton();
  updateLEDs();

  if (!enable || !robotRunning) {
    stopMotors();
    return;
  }

  #define RUN_CONDITION(p) (presses == (p) && enable && robotRunning)

  while (RUN_CONDITION(0)) { messenger.loop(); checkButton(); updateLEDs(); followLine(); }
  if (!enable || !robotRunning) return;
  delay(1000);

  while (RUN_CONDITION(1)) { messenger.loop(); checkButton(); updateLEDs(); runChainMode(); }
  if (!enable || !robotRunning) return;
  delay(1000);

  while (RUN_CONDITION(2)) { messenger.loop(); checkButton(); updateLEDs(); followLine(); }
  if (!enable || !robotRunning) return;
  delay(1000);

  while (RUN_CONDITION(3)) { messenger.loop(); checkButton(); updateLEDs(); runDeadReckoning(); }
  if (!enable || !robotRunning) return;
  delay(1000);

  while (RUN_CONDITION(4)) { messenger.loop(); checkButton(); updateLEDs(); runRevival(); }
  if (!enable || !robotRunning) return;
  delay(1000);

  while (RUN_CONDITION(5)) { messenger.loop(); checkButton(); updateLEDs(); runObstacleMode(); }
  delay(1000);

  #undef RUN_CONDITION
}
