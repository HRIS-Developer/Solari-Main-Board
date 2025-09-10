TaskHandle_t Task1;
TaskHandle_t Task2;

// Task 1: Blink LED on pin 21 (you can change to onboard LED if available)
void TaskBlink(void *pvParameters) {
  const int ledPin = 21;  
  pinMode(ledPin, OUTPUT);

  while (true) {
    digitalWrite(ledPin, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(ledPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task 2: Print to serial
void TaskSerial(void *pvParameters) {
  while (true) {
    Serial.println("Hello from Task 2 running on core: " + String(xPortGetCoreID()));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give time for Serial Monitor to open

  // Create Task 1 on Core 0
  xTaskCreatePinnedToCore(
    TaskBlink,      // Task function
    "Task1",        // Name
    2048,           // Stack size
    NULL,           // Parameters
    1,              // Priority
    &Task1,         // Task handle
    0               // Core 0
  );

  // Create Task 2 on Core 1
  xTaskCreatePinnedToCore(
    TaskSerial,     // Task function
    "Task2",        // Name
    4096,           // Stack size
    NULL,           // Parameters
    1,              // Priority
    &Task2,         // Task handle
    1               // Core 1
  );
}

void loop() {
  // Empty because FreeRTOS tasks are running independently
}
