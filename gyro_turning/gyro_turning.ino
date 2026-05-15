#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Motoron.h>


MotoronI2C mc;
Adafruit_MPU6050 mpu;


MotoronI2C mc1(16);
MotoronI2C mc2(17);


void setupMotoron(MotoronI2C & mc)
{
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
}


void setup()
{
  Wire.begin();
  Wire1.begin();

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


  Serial.begin(115200);
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
}


void loop() {
  bool turnedNinety = false;
  bool turnedHalf = false;
  float x = 0;
  float y = 0;
  float z = 0;
  while (!turnedNinety) {
    mc1.setSpeed(2, -600);
    mc1.setSpeed(3, 600);
    mc2.setSpeed(2, -600);
    mc2.setSpeed(3, 600);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    z = z + g.gyro.z * 0.01;
    Serial.print(z);
    Serial.println();
    if (z >= 0.9 || z <= -0.9) {
      turnedNinety = true;
    }
    delay(10);
  }
  z = 0;
  while (!turnedHalf) {
    mc1.setSpeed(2, 600);
    mc1.setSpeed(3, -600);
    mc2.setSpeed(2, 600);
    mc2.setSpeed(3, -600);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    z = z + g.gyro.z * 0.01;
    Serial.print(z);
    Serial.println();
    if (z >= 1.5 || z <= -1.5) {
      turnedHalf = true;
    }
    delay(10);
  }
  delay(5000);
}
