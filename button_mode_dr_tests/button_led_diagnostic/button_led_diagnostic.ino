// Button and LED diagnostic.
// Prints raw states for the mode button and both bumper inputs, while driving
// the LEDs using the same rule as the robot: red when released, green on bumper
// contact. Use this before competition runs to catch swapped pins or wiring.

const int modeButtonPin = 49;
const int button1Pin = 22;
const int button2Pin = 33;

const int redPin = 46;
const int greenPin = 47;

const unsigned long PRINT_INTERVAL_MS = 250;

bool lastModeButton = HIGH;
bool lastButton1 = HIGH;
bool lastButton2 = HIGH;
unsigned long lastPrintMs = 0;

void printState(const char* reason) {
  bool modeButton = digitalRead(modeButtonPin);
  bool button1 = digitalRead(button1Pin);
  bool button2 = digitalRead(button2Pin);

  Serial.print(reason);
  Serial.print(" | pin49 mode=");
  Serial.print(modeButton == LOW ? "LOW/PRESSED" : "HIGH/released");
  Serial.print(" pin22 bumper1=");
  Serial.print(button1 == LOW ? "LOW/PRESSED" : "HIGH/released");
  Serial.print(" pin33 bumper2=");
  Serial.println(button2 == LOW ? "LOW/PRESSED" : "HIGH/released");
}

void updateLeds(bool pressed) {
  if (redPin >= 0) digitalWrite(redPin, pressed ? LOW : HIGH);
  if (greenPin >= 0) digitalWrite(greenPin, pressed ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  pinMode(modeButtonPin, INPUT_PULLUP);
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  if (redPin >= 0) {
    pinMode(redPin, OUTPUT);
    digitalWrite(redPin, HIGH);
  }
  if (greenPin >= 0) {
    pinMode(greenPin, OUTPUT);
    digitalWrite(greenPin, LOW);
  }

  Serial.println("Button/LED diagnostic");
  Serial.println("Pins use INPUT_PULLUP, so pressed should read LOW.");
  Serial.println("Pin 49 = mode button, pin 22 = bumper 1, pin 33 = bumper 2.");
  Serial.println("LED output follows bumpers only: green while pin 22/33 is pressed, red otherwise.");
  if (redPin < 0 || greenPin < 0) {
    Serial.println("One or both LED pins are -1; set them to test red/green output.");
  }
  printState("BOOT");
}

void loop() {
  unsigned long now = millis();

  bool modeButton = digitalRead(modeButtonPin);
  bool button1 = digitalRead(button1Pin);
  bool button2 = digitalRead(button2Pin);
  bool anyBumperPressed = (button1 == LOW || button2 == LOW);

  updateLeds(anyBumperPressed);

  if (modeButton != lastModeButton) {
    printState("CHANGE");
    lastModeButton = modeButton;
  }

  if (button1 != lastButton1) {
    printState("CHANGE");
    lastButton1 = button1;
  }

  if (button2 != lastButton2) {
    printState("CHANGE");
    lastButton2 = button2;
  }

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printState("POLL");
  }

}
