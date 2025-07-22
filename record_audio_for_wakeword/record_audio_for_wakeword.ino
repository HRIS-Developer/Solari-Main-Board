#include "ESP_I2S.h"
#include "FS.h"
#include "SD.h"

I2SClass i2s;
String currentLabel = "default";
int recordingCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("Initializing I2S bus...");

  i2s.setPinsPdmRx(42, 41);

  if (!i2s.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  Serial.println("I2S bus initialized.");
  Serial.println("Initializing SD card...");

  if (!SD.begin(21)) {
    Serial.println("Failed to mount SD Card!");
    while (1);
  }

  Serial.println("SD card initialized.");
  Serial.println("Enter label name or type 'rec' to record:");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("rec")) {
      recordAudio();
    } else {
      currentLabel = input;
      recordingCount = 0; // Reset count for new label
      Serial.print("Label changed to: ");
      Serial.println(currentLabel);
    }

    Serial.println("Enter label name or type 'rec' to record:");
  }
}

void recordAudio() {
  Serial.println("Recording 10 seconds of audio...");

  size_t wav_size;
  uint8_t *wav_buffer = i2s.recordWAV(10, &wav_size); // 10 seconds

  String filename = "/" + currentLabel + "_" + String(recordingCount++) + ".wav";
  File file = SD.open(filename, FILE_WRITE);
  
  if (!file) {
    Serial.println("Failed to open file for writing!");
    return;
  }

  Serial.print("Saving to ");
  Serial.println(filename);

  if (file.write(wav_buffer, wav_size) != wav_size) {
    Serial.println("Failed to write audio data to file!");
  } else {
    Serial.println("Audio saved successfully.");
  }

  file.close();
}
