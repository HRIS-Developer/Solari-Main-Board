#include <driver/i2s.h>

// Xiao ESP32-S3 pin names (works fine)
#define I2S_BCLK D1  // BCLK
#define I2S_LRC  D0  // LRC / WS
#define I2S_DOUT D2  // DIN

#define SAMPLE_RATE 44100
#define TONE_FREQUENCY 440  // Hz

int16_t amplitude = 30000;  // Default amplitude (can be changed via Serial)

void setup() {
  Serial.begin(115200);
  Serial.println("\n[DEBUG] Setting up I2S (Mono)...");
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
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void playTone(float frequency, int duration_ms) {
  int samples = SAMPLE_RATE * duration_ms / 1000;
  int16_t sample;
  float phase = 0.0;
  float phaseIncrement = 2.0 * PI * frequency / SAMPLE_RATE;

  for (int i = 0; i < samples; i++) {
    sample = (int16_t)(sin(phase) * amplitude);  // Use adjustable amplitude
    size_t written;
    i2s_write(I2S_NUM_0, &sample, sizeof(sample), &written, portMAX_DELAY);

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
