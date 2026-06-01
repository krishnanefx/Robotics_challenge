// Seed-drop duration tuner.
// Runs only the seed motor on Motoron 17 channel 1. Increase duration/speed
// until one seed drops reliably, then copy the shortest reliable values into
// the main dropSeed() implementation.

#include <Motoron.h>
#include <Wire.h>

MotoronI2C mc2(17);  // ch1 = seed motor

const int SEED_CHANNEL = 1;
int seedSpeed = 600;
unsigned long seedDurationMs = 700;

void setupMotoron(MotoronI2C& mc) {
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.setMaxAcceleration(SEED_CHANNEL, 200);
  mc.setMaxDeceleration(SEED_CHANNEL, 300);
}

void stopSeedMotor() {
  mc2.setSpeed(SEED_CHANNEL, 0);
}

void runSeedMotor(int speed, unsigned long durationMs) {
  Serial.print("Seed motor speed=");
  Serial.print(speed);
  Serial.print(" durationMs=");
  Serial.println(durationMs);

  mc2.setSpeed(SEED_CHANNEL, speed);
  delay(durationMs);
  stopSeedMotor();
  Serial.println("Done");
}

void printHelp() {
  Serial.println();
  Serial.println("Drop seed duration test");
  Serial.println("Commands:");
  Serial.println("  t      test one seed drop");
  Serial.println("  + / -  duration up/down by 50 ms");
  Serial.println("  u / d  speed up/down by 50");
  Serial.println("  b      run backward for same duration, useful for clearing jams");
  Serial.println("  s      stop seed motor");
  Serial.println("Tune dropSeed() to the shortest duration that drops exactly one seed.");
  Serial.println();
}

void printStatus() {
  Serial.print("seedSpeed=");
  Serial.print(seedSpeed);
  Serial.print(" seedDurationMs=");
  Serial.println(seedDurationMs);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  Wire1.begin();
  mc2.setBus(&Wire1);
  setupMotoron(mc2);
  stopSeedMotor();

  printHelp();
  printStatus();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == 't' || c == 'T') {
    runSeedMotor(seedSpeed, seedDurationMs);
  } else if (c == '+') {
    seedDurationMs += 50;
    printStatus();
  } else if (c == '-') {
    if (seedDurationMs >= 100) seedDurationMs -= 50;
    printStatus();
  } else if (c == 'u' || c == 'U') {
    seedSpeed = constrain(seedSpeed + 50, 0, 800);
    printStatus();
  } else if (c == 'd' || c == 'D') {
    seedSpeed = constrain(seedSpeed - 50, 0, 800);
    printStatus();
  } else if (c == 'b' || c == 'B') {
    runSeedMotor(-seedSpeed, seedDurationMs);
  } else if (c == 's' || c == 'S') {
    stopSeedMotor();
    Serial.println("Stopped");
  } else if (c == '?') {
    printHelp();
  }
}
