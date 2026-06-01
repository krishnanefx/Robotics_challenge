// Airlock RFID reader diagnostic.
// Use this first when the door does not open: it confirms the MFRC522 is present
// on Wire2 and prints the UID string that should be used as AIRLOCK_A_TAG_ID.
// Expected airlock A tag from tuning: C2834BF4.

#include <MFRC522_I2C.h>
#include <Wire.h>

#define RFID_WIRE Wire2
#define RFID_RESET_PIN -1

MFRC522_I2C* rfid = nullptr;

byte findI2CAddress() {
  byte foundAddress = 0;
  byte foundCount = 0;

  Serial.println("Scanning Wire2 I2C bus...");
  for (byte address = 1; address < 127; address++) {
    RFID_WIRE.beginTransmission(address);
    if (RFID_WIRE.endTransmission() == 0) {
      if (foundAddress == 0) foundAddress = address;
      foundCount++;
      Serial.print("Found I2C device at 0x");
      if (address < 0x10) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (foundCount == 0) {
    Serial.println("ERROR: No I2C devices found on Wire2.");
    return 0;
  }
  if (foundCount > 1) Serial.println("WARNING: Multiple devices found; using first.");
  return foundAddress;
}

void printUid() {
  Serial.print("AIRLOCK_A_TAG_ID = \"");
  for (byte i = 0; i < rfid->uid.size; i++) {
    if (rfid->uid.uidByte[i] < 0x10) Serial.print("0");
    Serial.print(rfid->uid.uidByte[i], HEX);
  }
  Serial.println("\"");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  RFID_WIRE.begin();
  delay(100);

  byte rfidAddress = findI2CAddress();
  if (rfidAddress == 0) {
    while (true) delay(1000);
  }

  rfid = new MFRC522_I2C(rfidAddress, RFID_RESET_PIN, &RFID_WIRE);
  rfid->PCD_Init();
  Serial.println("Hold the airlock RFID tag near the reader.");
}

void loop() {
  if (rfid == nullptr) return;
  if (!rfid->PICC_IsNewCardPresent()) return;
  if (!rfid->PICC_ReadCardSerial()) return;

  printUid();
  rfid->PICC_HaltA();
  delay(500);
}
