#include <WiFi.h>

String ssid = "";
String password = "";

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== WiFi Setup ===");
  ssid = readSerialLine("Enter WiFi SSID: ");
  password = readSerialLine("Enter WiFi Password: ");

  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid.c_str(), password.c_str());

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected!");
    Serial.print("📡 IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Failed to connect.");
  }
}

void loop() {
  // Nothing needed here for now
}

String readSerialLine(String prompt) {
  Serial.print(prompt);
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') break;
      if (c != '\r') input += c;
    }
  }
  return input;
}
