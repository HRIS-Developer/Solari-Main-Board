#define BUTTON_PIN D10

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button input
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);
  Serial.println(buttonState);  // Just print state (LOW = pressed, HIGH = released)
  delay(100);
}
