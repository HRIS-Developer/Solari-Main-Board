#include "ESP_I2S.h"

// Xiao ESP32-S3 pin names (works fine)
#define I2S_BCLK D1  // BCLK
#define I2S_LRC  D0  // LRC / WS
#define I2S_DOUT D2  // DIN

#define SAMPLE_RATE 44100
#define TONE_FREQUENCY 440  // Hz

int16_t amplitude = 30000;  // Default amplitude (can be changed via Serial)

// Create I2S instance
I2SClass i2s;

void setup() {
  Serial.begin(115200);
  Serial.println("\n[DEBUG] Setting up I2S with ESP_I2S library...");
  setupI2S();
  Serial.println("[DEBUG] I2S setup complete!");
  Serial.println("[DEBUG] Type a number (0-32767) in Serial to change amplitude.");
}

void loop() {
  handleSerialInput();

  Serial.print("[DEBUG] Playing clean mono tone... Amplitude = ");
  Serial.println(amplitude);
  playTone(TONE_FREQUENCY, 1000);
  delay(500);
}

void setupI2S() {
  // Set the pins for I2S TX (output to MAX98357A)
  i2s.setPins(I2S_BCLK, I2S_LRC, I2S_DOUT);
  
  // Begin I2S in TX mode, mono, 16-bit, at the specified sample rate
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("[ERROR] Failed to initialize I2S!");
    while (true) delay(1000);
  }
  
  Serial.println("[DEBUG] ESP_I2S initialized successfully");
}

void playTone(float frequency, int duration_ms) {
  int samples = SAMPLE_RATE * duration_ms / 1000;
  int16_t sample;
  float phase = 0.0;
  float phaseIncrement = 2.0 * PI * frequency / SAMPLE_RATE;

  for (int i = 0; i < samples; i++) {
    sample = (int16_t)(sin(phase) * amplitude);  // Use adjustable amplitude
    
    // Write single sample using ESP_I2S library
    size_t bytesWritten = i2s.write((uint8_t*)&sample, sizeof(sample));
    
    if (bytesWritten != sizeof(sample)) {
      Serial.println("[WARNING] I2S write incomplete");
    }

    phase += phaseIncrement;
    if (phase >= 2.0 * PI) phase -= 2.0 * PI;
  }
}

// Listen for serial input and update amplitude
void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int newAmp = input.toInt();

    if (newAmp >= 0 && newAmp <= 32767) {
      amplitude = newAmp;
      Serial.print("[DEBUG] Amplitude updated to ");
      Serial.println(amplitude);
    } else {
      Serial.println("[ERROR] Please enter a number between 0 and 32767.");
    }
  }
}
