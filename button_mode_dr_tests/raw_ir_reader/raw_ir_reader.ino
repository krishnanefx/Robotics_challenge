// Raw IR sensor reader.
// Prints all 9 RC sensor timings so intersections, branches, and threshold
// choices can be checked without running any motor code. Higher raw values mean
// the sensor took longer to discharge and is more likely over the line.

#include <Arduino.h>

const uint8_t SensorCount = 9;
uint8_t sensorPins[SensorCount] = {23, 24, 25, 26, 27, 28, 29, 30, 31};
uint16_t sensorValues[SensorCount];

const uint16_t RAW_SENSOR_TIMEOUT_US = 2500;
uint16_t lineThreshold = 250;
bool printBars = true;

void readRawLineSensors() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);

  unsigned long start = micros();
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = RAW_SENSOR_TIMEOUT_US;
  }

  bool done = false;
  while (!done && micros() - start < RAW_SENSOR_TIMEOUT_US) {
    unsigned long elapsed = micros() - start;
    done = true;
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == RAW_SENSOR_TIMEOUT_US && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = elapsed;
      }
      if (sensorValues[i] == RAW_SENSOR_TIMEOUT_US) done = false;
    }
  }
}

bool sensorOnLine(uint8_t i) {
  return sensorValues[i] > lineThreshold;
}

void printSensorBars() {
  Serial.print("bars=");
  for (uint8_t i = 0; i < SensorCount; i++) {
    int bars = map(sensorValues[i], 0, RAW_SENSOR_TIMEOUT_US, 0, 10);
    bars = constrain(bars, 0, 10);
    Serial.print(i);
    Serial.print(":");
    for (int j = 0; j < bars; j++) Serial.print("#");
    for (int j = bars; j < 10; j++) Serial.print(".");
    if (i < SensorCount - 1) Serial.print(" ");
  }
  Serial.println();
}

void printReadout() {
  readRawLineSensors();

  Serial.print("raw_us=");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensorValues[i]);
    if (i < SensorCount - 1) Serial.print(" ");
  }

  bool left = sensorOnLine(0) || sensorOnLine(1) || sensorOnLine(2);
  bool center = sensorOnLine(3) || sensorOnLine(4) || sensorOnLine(5);
  bool right = sensorOnLine(6) || sensorOnLine(7) || sensorOnLine(8);

  Serial.print(" | L=");
  Serial.print(left ? "1" : "0");
  Serial.print(" C=");
  Serial.print(center ? "1" : "0");
  Serial.print(" R=");
  Serial.print(right ? "1" : "0");
  Serial.print(" threshold=");
  Serial.println(lineThreshold);

  if (printBars) printSensorBars();
}

void printHelp() {
  Serial.println();
  Serial.println("Raw IR reader");
  Serial.println("Pins/indexes:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("  ");
    Serial.print(i);
    Serial.print(" -> pin ");
    Serial.println(sensorPins[i]);
  }
  Serial.println("Commands:");
  Serial.println("  + / -  threshold up/down");
  Serial.println("  b      toggle bar graph");
  Serial.println("  ?      help");
  Serial.println();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '+') {
      lineThreshold = constrain(lineThreshold + 50, 50, RAW_SENSOR_TIMEOUT_US);
      Serial.print("threshold=");
      Serial.println(lineThreshold);
    } else if (c == '-') {
      lineThreshold = constrain(lineThreshold - 50, 50, RAW_SENSOR_TIMEOUT_US);
      Serial.print("threshold=");
      Serial.println(lineThreshold);
    } else if (c == 'b' || c == 'B') {
      printBars = !printBars;
      Serial.print("printBars=");
      Serial.println(printBars ? "true" : "false");
    } else if (c == '?') {
      printHelp();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);
  printHelp();
}

void loop() {
  handleSerial();
  printReadout();
  delay(200);
}
