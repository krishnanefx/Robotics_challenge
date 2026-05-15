// include relevant libraries
#include <Motoron.h>
#include <Wire.h>
#include <MFRC522_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <QTRSensors.h>
#include <math.h>


// define RFID parameters
#define RFID_WIRE Wire2
#define RFID_RESET_PIN -1


// set variables for QTR
const uint8_t SensorCount = 9;
uint16_t sensorValues[SensorCount];
uint8_t sensorPins[SensorCount] = {24, 26, 28, 30, 32, 34, 36, 38, 40};
float weights[SensorCount] = {0.15, 0.12, 0.08, 0.04, 0, -0.04, -0.08, -0.12, -0.15};


MFRC522_I2C *rfid = nullptr;
Adafruit_MPU6050 mpu;
MotoronI2C mc;
QTRSensors qtr;


MotoronI2C mc1(18);
MotoronI2C mc2(17);


// function for error proportional line following
void followLine() {
  qtr.readCalibrated(sensorValues);

  float error = 0;
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    error = error + (sensorValues[i] * weights[i]);
  }
  int error1 = floor(error);
  Serial.println("Error: ");
  Serial.print(error1);
  mc1.setSpeed(2, (400 + error1));
  mc1.setSpeed(3, (400 - error1));
  mc2.setSpeed(2, (400 + error1));
  mc2.setSpeed(3, (400 - error1));
}


// function for deploying seed
void dropSeed() {
  mc2.setSpeed(1, 400);
  delay(450);
  mc2.setSpeed(1, 0);
}


// functions for rotating 90 degrees
void turnLeft() {
  bool turnedNinety = false;
  float z = 0;
  while (!turnedNinety) {
    mc1.setSpeed(2, 600);
    mc1.setSpeed(3, -600);
    mc2.setSpeed(2, 600);
    mc2.setSpeed(3, -600);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    z = z + g.gyro.z * 0.01;
    Serial.println("Turned: ");
    Serial.print(z);
    if (z >= 0.9 || z <= -0.9) {
      turnedNinety = true;
    }
    delay(10);
  }
}


void turnRight() {
  bool turnedNinety = false;
  float z = 0;
  while (!turnedNinety) {
    mc1.setSpeed(2, -600);
    mc1.setSpeed(3, 600);
    mc2.setSpeed(2, -600);
    mc2.setSpeed(3, 600);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    z = z + g.gyro.z * 0.01;
    Serial.println("Turned: ");
    Serial.print(z);
    if (z >= 0.9 || z <= -0.9) {
      turnedNinety = true;
    }
    delay(10);
  }
}


// function for RFID
byte findI2CAddress() {
  byte foundAddress = 0;
  byte foundCount = 0;

  Serial.println("Scanning Wire2 I2C bus...");

  for (byte address = 1; address < 127; address++) {
    RFID_WIRE.beginTransmission(address);
    byte error = RFID_WIRE.endTransmission();

    if (error == 0) {
      if (foundAddress == 0) {
        foundAddress = address;
      }
      foundCount++;

      Serial.print("Found I2C device at 0x");
      if (address < 0x10) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (foundCount == 0) {
    Serial.println("ERROR: No I2C devices found on Wire2.");
    return 0;
  }

  if (foundCount > 1) {
    Serial.println("WARNING: Multiple I2C devices found. Using the first detected address.");
  }

  return foundAddress;
}


void setupMotoron(MotoronI2C & mc)
{
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
}


void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Wire.begin();
  Wire1.begin();
  RFID_WIRE.begin();
  delay(100);

  // set up RFID
  byte rfidAddress = findI2CAddress();
  if (rfidAddress == 0) {
    while (true) {
      delay(1000);
    }
  }

  rfid = new MFRC522_I2C(rfidAddress, RFID_RESET_PIN, &RFID_WIRE);
  rfid->PCD_Init();

  Serial.println("WS1850S RFID reader ready");
  Serial.print("Using RFID reader at I2C address 0x");
  if (rfidAddress < 0x10) {
    Serial.print("0");
  }
  Serial.println(rfidAddress, HEX);
  Serial.println("Place card near reader...");

  // set up MPU6050
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  // set up motors
  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);
  setupMotoron(mc1);
  setupMotoron(mc2);

  mc1.setMaxAcceleration(1, 80);
  mc1.setMaxDeceleration(1, 300);

  mc1.setMaxAcceleration(2, 80);
  mc1.setMaxDeceleration(2, 300);

  mc1.setMaxAcceleration(3, 80);
  mc1.setMaxDeceleration(3, 300);

  mc2.setMaxAcceleration(1, 80);
  mc2.setMaxDeceleration(1, 300);

  mc2.setMaxAcceleration(2, 80);
  mc2.setMaxDeceleration(2, 300);

  mc2.setMaxAcceleration(3, 80);
  mc2.setMaxDeceleration(3, 300);

  // set up QTR
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setTimeout(2500);
  delay(500);
  Serial.println("Calibrating... move sensor over line");
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibration done!");
}


void loop()
{
  // drop seed and turn 90 degrees if RFID detected
  if (rfid != nullptr && rfid->PICC_IsNewCardPresent() && rfid->PICC_ReadCardSerial()) {
    dropSeed();
    turnRight();
    // line follow for a while afterward to avoid repeatedly scanning the same RFID and repeatedly turning 90 degrees
    for (int i = 0; i < 200; i++) {
      followLine();
    }
  }
  // follow the line otherwise
  followLine();
}
