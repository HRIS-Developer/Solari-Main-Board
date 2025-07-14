#include <ESP_I2S.h>
I2SClass I2S;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect

  I2S.setPinsPdmRx(42, 41);

  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  Serial.println("Ready to stream audio...");
}

void loop() {
  int sample = I2S.read();

  if (sample != -1) {
    // Send 16-bit little-endian audio sample
    Serial.write(sample & 0xFF);         // Low byte
    Serial.write((sample >> 8) & 0xFF);  // High byte
  }
}
