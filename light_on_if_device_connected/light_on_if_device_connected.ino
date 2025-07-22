#include <WiFi.h>

const char* ssid = "XIAO-CAM";
const char* password = "12345678";  // Minimum 8 characters

#define ORANGE_LED_PIN 21  // On XIAO ESP32S3, the orange LED is on pin 21

void setup() {
  Serial.begin(115200);

  // Set orange LED pin as output
  pinMode(ORANGE_LED_PIN, OUTPUT);
  digitalWrite(ORANGE_LED_PIN, LOW); // Start off

  // Start Wi-Fi Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("📡 Access Point IP: ");
  Serial.println(IP);
}

void loop() {
  int connected = WiFi.softAPgetStationNum(); // Get number of clients
  Serial.print("Connected devices: ");
  Serial.println(connected);

  if (connected > 0) {
    digitalWrite(ORANGE_LED_PIN, LOW); // Turn on LED
  } else {
    digitalWrite(ORANGE_LED_PIN, HIGH);  // Turn off LED
  }

  delay(1000); // Check every second
}
