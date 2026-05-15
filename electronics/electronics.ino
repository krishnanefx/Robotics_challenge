#include <Wire.h>
#include <Motoron.h>
#include <TFLI2C.h>
#include <MFRC522_I2C.h>
#include <QTRSensors.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// ── Pins ──────────────────────────────────────────────────────
const int START_STOP_PIN = 45;
const int RED_PIN        = 53;
const int GREEN_PIN      = 51;


// ── Robot state ───────────────────────────────────────────────
bool robotRunning    = false;
bool lastButtonState = HIGH;
bool blinkState      = false;
unsigned long lastBlinkTime = 0;


// ── Motoron (dynamic address scan on Wire1) ───────────────────
uint8_t SHIELD1_ADDRESS = 0;
uint8_t SHIELD2_ADDRESS = 0;
MotoronI2C mc1(0), mc2(0);


// ── TFLuna LiDAR (Wire1) ──────────────────────────────────────
TFLI2C  tflI2C;
int16_t tfDist = 0, tfFlux = 0, tfTemp = 0;
unsigned long lastLidarRead = 0;


// ── RFID (Wire2 — dynamic address scan) ──────────────────────
MFRC522_I2C *rfid = nullptr;
unsigned long lastRfidRead = 0;
const unsigned long RFID_COOLDOWN = 500;


// ── MPU6050 gyro (Wire) ───────────────────────────────────────
Adafruit_MPU6050 mpu;


// ── QTR line sensors ──────────────────────────────────────────
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 9;
uint16_t      sensorValues[SENSOR_COUNT];
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {24, 26, 28, 30, 32, 34, 36, 38, 40};
unsigned long lastQTRRead = 0;


// ── LED helpers ───────────────────────────────────────────────
void setRed() {
 digitalWrite(RED_PIN, HIGH);
 digitalWrite(GREEN_PIN, LOW);
}


void ledOff() {
 digitalWrite(RED_PIN, LOW);
 digitalWrite(GREEN_PIN, LOW);
}


void updateBlinkRed() {
 unsigned long now = millis();
 if (now - lastBlinkTime >= 300) {
   lastBlinkTime = now;
   blinkState = !blinkState;
   blinkState ? setRed() : ledOff();
 }
}


// ── Button ────────────────────────────────────────────────────
void checkButton() {
 bool cur = digitalRead(START_STOP_PIN);
 if (lastButtonState == HIGH && cur == LOW) {
   delay(50);
   robotRunning = !robotRunning;
   if (!robotRunning) {
     mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
     mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);
   }
 }
 lastButtonState = cur;
}


// ── Motoron ───────────────────────────────────────────────────
void setupMotoron(MotoronI2C &mc) {
 mc.reinitialize();
 mc.disableCrc();
 mc.clearResetFlag();
 mc.disableCommandTimeout();
 mc.clearMotorFaultUnconditional();
 mc.setMaxAcceleration(2, 80);  mc.setMaxDeceleration(2, 300);
 mc.setMaxAcceleration(3, 80);  mc.setMaxDeceleration(3, 300);
}


void stopMotors() {
 mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
 mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);
}


// ── Sensor polling ────────────────────────────────────────────
void pollSensors() {
 unsigned long now = millis();


 // LiDAR every 50 ms
 if (now - lastLidarRead >= 50) {
   lastLidarRead = now;
   if (tflI2C.getData(tfDist, tfFlux, tfTemp, TFL_DEF_ADR)) {
     Serial.print("LiDAR: "); Serial.print(tfDist); Serial.println(" cm");
   } else {
     Serial.print("LiDAR err: "); tflI2C.printStatus(); Serial.println();
   }
 }


 // RFID (with cooldown to avoid re-reading the same card)
 if (rfid != nullptr && now - lastRfidRead >= RFID_COOLDOWN) {
   if (rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
     lastRfidRead = now;
     Serial.print("RFID UID: ");
     for (byte i = 0; i < rfid->uid.size; i++) {
       if (rfid->uid.uidByte[i] < 0x10) Serial.print("0");
       Serial.print(rfid->uid.uidByte[i], HEX);
       Serial.print(" ");
     }
     Serial.println();
     rfid->PICC_HaltA();
     rfid->PCD_StopCrypto1();
   }
 }


 // Line sensors every 100 ms
 if (now - lastQTRRead >= 100) {
   lastQTRRead = now;
   qtr.readCalibrated(sensorValues);
   Serial.print("Line: ");
   for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
     Serial.print(sensorValues[i]);
     Serial.print(" ");
   }
   Serial.println();
 }
}


// ── Movement ──────────────────────────────────────────────────
bool forwardFor(int speed, unsigned long duration) {
 mc1.setSpeed(2, speed); mc1.setSpeed(3, speed);
 mc2.setSpeed(2, speed); mc2.setSpeed(3, speed);
 unsigned long start = millis();
 while (millis() - start < duration) {
   checkButton();
   if (!robotRunning) return false;
   pollSensors();
 }
 return true;
}


// Spin in place using gyro integration; ch2 = one side, ch3 = other side.
// Swap leftSpeed/rightSpeed signs if physical turn direction is reversed.
bool turnGyro(int leftSpeed, int rightSpeed, float threshold) {
 float z = 0;
 sensors_event_t a, g, temp;
 while (true) {
   checkButton();
   if (!robotRunning) { stopMotors(); return false; }
   mc1.setSpeed(2, leftSpeed);  mc1.setSpeed(3, rightSpeed);
   mc2.setSpeed(2, leftSpeed);  mc2.setSpeed(3, rightSpeed);
   mpu.getEvent(&a, &g, &temp);
   z += g.gyro.z * 0.01f;
   pollSensors();
   if (abs(z) >= threshold) break;
   delay(10);
 }
 stopMotors();
 return true;
}


bool turnRight90() { return turnGyro( 600, -600, 0.9f); }
bool turnLeft90()  { return turnGyro(-600,  600, 0.9f); }
bool uTurn()       { return turnGyro( 600, -600, 1.5f); }


// ── Initialisation helpers ────────────────────────────────────
void initRFID() {
 Wire2.begin();
 delay(100);
 Serial.println("Scanning Wire2 for RFID...");
 byte foundAddr = 0;
 for (byte addr = 1; addr < 127; addr++) {
   Wire2.beginTransmission(addr);
   if (Wire2.endTransmission() == 0) {
     Serial.print("  Found device at 0x");
     if (addr < 0x10) Serial.print("0");
     Serial.println(addr, HEX);
     if (foundAddr == 0) foundAddr = addr;
   }
 }
 if (foundAddr == 0) {
   Serial.println("ERROR: No RFID found on Wire2!");
   while (1);
 }
 rfid = new MFRC522_I2C(foundAddr, -1, &Wire2);
 rfid->PCD_Init();
 Serial.print("RFID ready at 0x");
 if (foundAddr < 0x10) Serial.print("0");
 Serial.println(foundAddr, HEX);
}


void initMotors() {
 Wire1.begin();
 uint8_t found = 0;
 uint8_t addrs[10];
 for (uint8_t a = 1; a < 127 && found < 10; a++) {
   Wire1.beginTransmission(a);
   if (Wire1.endTransmission() == 0) addrs[found++] = a;
 }
 if (found < 2) {
   Serial.println("ERROR: Need 2 motor shields on Wire1!");
   while (1);
 }
 SHIELD1_ADDRESS = addrs[0];
 SHIELD2_ADDRESS = addrs[1];
 mc1 = MotoronI2C(SHIELD1_ADDRESS); mc1.setBus(&Wire1);
 mc2 = MotoronI2C(SHIELD2_ADDRESS); mc2.setBus(&Wire1);
 setupMotoron(mc1);
 setupMotoron(mc2);
 Serial.print("Motor shields at 0x"); Serial.print(SHIELD1_ADDRESS, HEX);
 Serial.print(", 0x"); Serial.println(SHIELD2_ADDRESS, HEX);
}


void initLidar() {
 tflI2C.setBus(&Wire1);
 tflI2C.Soft_Reset(TFL_DEF_ADR);
 delay(500);
 uint16_t fps = TFL_DEF_FPS;
 tflI2C.Set_Frame_Rate(fps, TFL_DEF_ADR);
 delay(500);
 Serial.println("TFLuna ready");
}


void initMPU() {
 if (!mpu.begin()) {
   Serial.println("MPU6050 not found!");
   while (1);
 }
 mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
 mpu.setGyroRange(MPU6050_RANGE_500_DEG);
 mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
 Serial.println("MPU6050 ready");
}


void calibrateQTR() {
 qtr.setTypeRC();
 qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
 qtr.setTimeout(2500);
 Serial.println("Calibrating QTR — slowly move sensor array over the line...");
 for (uint16_t i = 0; i < 200; i++) {
   qtr.calibrate();
   delay(20);
 }
 Serial.println("QTR calibration done");
}


// ── Setup ─────────────────────────────────────────────────────
void setup() {
 Serial.begin(115200);
 delay(2000);


 pinMode(START_STOP_PIN, INPUT_PULLUP);
 pinMode(RED_PIN,   OUTPUT);
 pinMode(GREEN_PIN, OUTPUT);
 setRed(); // solid red on load


 initRFID();     // Wire2 — dynamic address scan
 initMotors();   // Wire1 — dynamic address scan
 initLidar();    // Wire1
 Wire.begin();
 initMPU();      // Wire
 calibrateQTR(); // digital RC sensors, no I2C


 Serial.println("=== READY — press button to start ===");
}


// ── Main loop ─────────────────────────────────────────────────
void loop() {
 checkButton();


 if (robotRunning) {
   setRed();
   if (!forwardFor(800, 5000)) return;
   if (!forwardFor(400, 5000)) return;
   if (!turnRight90())         return;
   if (!turnLeft90())          return;
   if (!uTurn())               return;
 } else {
   stopMotors();
   updateBlinkRed();
 }
}
