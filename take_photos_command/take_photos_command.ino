#include "esp_camera.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

unsigned long lastCaptureTime = 0;
int imageCount = 1;
bool camera_sign = false;
bool commandRecv = false;

// Send image via Serial
void sendImage(camera_fb_t *fb) {
  Serial.println("IMG"); // Sync header
  uint32_t imgLen = fb->len;
  Serial.write((uint8_t *)&imgLen, 4); // Send length as 4 bytes
  Serial.write(fb->buf, imgLen);       // Send JPEG bytes
  Serial.flush();                      // Ensure all bytes are sent
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  camera_sign = true;

  Serial.println("XIAO ESP32S3 Sense Camera Image Capture");
  Serial.println("Send 'capture' to initiate an image capture\n");
}

void loop() {
  if (camera_sign) {
    String command;

    while (Serial.available()) {
      char c = Serial.read();
      if ((c != '\n') && (c != '\r')) {
        command.concat(c);
      } else if (c == '\n') {
        commandRecv = true;
        command.toLowerCase();
      }
    }

    if (commandRecv && command == "capture") {
      commandRecv = false;
      Serial.println("\nPicture Capture Command Received");

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        return;
      }

      sendImage(fb);  // Send image over Serial
      esp_camera_fb_return(fb);
    }
  }
}
