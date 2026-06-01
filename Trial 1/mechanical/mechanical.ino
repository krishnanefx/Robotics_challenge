#include <Wire.h>
#include <Motoron.h>


// ── Pins ──────────────────────────────────────────────────────
const int START_STOP_PIN = 45;  // triggers seed dispenser
const int INTERFACE_PIN  = 49;  // LED only — no motor effect
const int RED_PIN        = 53;
const int GREEN_PIN      = 51;


// ── Motoron (fixed addresses) ─────────────────────────────────
MotoronI2C mc1(16);
MotoronI2C mc2(17);


// ── State ─────────────────────────────────────────────────────
bool dispenserRunning   = false;
bool lastStartStopState = HIGH;
unsigned long phaseStart  = 0;
int  dispenseCount        = 0;
bool motorOn              = false;


const int   TOTAL_DISPENSES  = 6;
const int   DISPENSE_ON_MS   = 710;
const int   DISPENSE_OFF_MS  = 2000;


// ── LED helpers ───────────────────────────────────────────────
void setRed()   { digitalWrite(RED_PIN, HIGH); digitalWrite(GREEN_PIN, LOW); }
void setGreen() { digitalWrite(RED_PIN, LOW);  digitalWrite(GREEN_PIN, HIGH); }


// ── Motoron setup ─────────────────────────────────────────────
void setupMotoron(MotoronI2C &mc) {
 mc.reinitialize();
 mc.disableCrc();
 mc.clearResetFlag();
 mc.disableCommandTimeout();
 mc.clearMotorFaultUnconditional();
 mc.setMaxAcceleration(1, 80);  mc.setMaxDeceleration(1, 300);
}


// ── Setup ─────────────────────────────────────────────────────
void setup() {
 Serial.begin(115200);


 pinMode(START_STOP_PIN, INPUT_PULLUP);
 pinMode(INTERFACE_PIN,  INPUT_PULLUP);
 pinMode(RED_PIN,   OUTPUT);
 pinMode(GREEN_PIN, OUTPUT);
 setRed();


 Wire1.begin();
 mc1.setBus(&Wire1);
 mc2.setBus(&Wire1);
 setupMotoron(mc1);
 setupMotoron(mc2);


 Serial.println("Mechanical ready");
}


// ── Main loop ─────────────────────────────────────────────────
void loop() {
 unsigned long now = millis();


 // Start button — falling edge starts 6-dispense cycle
 bool curStartStop = digitalRead(START_STOP_PIN);
 if (lastStartStopState == HIGH && curStartStop == LOW) {
   delay(50);
   if (!dispenserRunning) {
     dispenserRunning = true;
     dispenseCount    = 0;
     motorOn          = true;
     phaseStart       = now;
     mc2.setSpeed(1, 400);
     Serial.print("Dispense 1/"); Serial.println(TOTAL_DISPENSES);
   }
 }
 lastStartStopState = curStartStop;


 // Dispense state machine: on 710 ms → off 2000ms → repeat x6
 if (dispenserRunning) {
   if (motorOn && now - phaseStart >= DISPENSE_ON_MS) {
     mc2.setSpeed(1, 0);
     motorOn    = false;
     phaseStart = now;
     dispenseCount++;
     if (dispenseCount >= TOTAL_DISPENSES) {
       dispenserRunning = false;
       Serial.println("All 6 dispenses done");
     }
   } else if (!motorOn && now - phaseStart >= DISPENSE_OFF_MS) {
     motorOn    = true;
     phaseStart = now;
     mc2.setSpeed(1, 400);
     Serial.print("Dispense "); Serial.print(dispenseCount + 1);
     Serial.print("/"); Serial.println(TOTAL_DISPENSES);
   }
 }


 // Interface button — green while held, red when released
 digitalWrite(GREEN_PIN, digitalRead(INTERFACE_PIN) == LOW ? HIGH : LOW);
 digitalWrite(RED_PIN,   digitalRead(INTERFACE_PIN) == LOW ? LOW  : HIGH);
}
