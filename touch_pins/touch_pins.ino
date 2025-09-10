const int touch_pin = A5;        // Touch sensor pin
const int led_pin = LED_BUILTIN; // On-board LED

bool ledState = false;     // Current LED state
bool lastTouch = false;    // Previous touch state
int threshold = 2000;      // Adjust based on your sensor values

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);
}

void loop() {
  int touchValue = analogRead(touch_pin);

  Serial.print("Touch value: ");
  Serial.println(touchValue);

  // Consider "touched" if above threshold
  bool isTouched = (touchValue > threshold);

  // Detect rising edge (not touched -> touched)
  if (isTouched && !lastTouch) {
    ledState = !ledState;              // Toggle LED state
    digitalWrite(led_pin, ledState);   // Update LED
  }

  lastTouch = isTouched; // Remember state for next loop
  delay(50);             // Small debounce delay
}
