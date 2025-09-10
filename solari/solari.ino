#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_camera.h"
#include "ESP_I2S.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define CAPTURE_QUEUE_LEN   1
#define CAPTURE_DEBOUNCE_MS 500
#define SEND_DELAY_BETWEEN_CHUNKS_MS 15



// ============================================================================
// BLE Globals
// ============================================================================

bool deviceConnected = false;
BLECharacteristic* pCharacteristic;
int negotiatedChunkSize = 23;
I2SClass i2s;

// Task handles and queues
QueueHandle_t imageQueue = nullptr;
QueueHandle_t audioQueue = nullptr;
TaskHandle_t imageTaskHandle = nullptr;
TaskHandle_t audioTaskHandle = nullptr;



// ============================================================================
// Logging Helper
// ============================================================================
void logMsg(const String &msg) {
  Serial.printf("[%lu] %s\n", millis(), msg.c_str());
}



// ============================================================================
// BLE Server Callbacks
// ============================================================================

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    logMsg("[BLE] Device connected!");
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    logMsg("[BLE] Device disconnected!");
    deviceConnected = false;
    BLEDevice::startAdvertising();
    logMsg("[BLE] Restarting advertising...");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = String(characteristic->getValue().c_str());
    value.trim();
    value.toUpperCase();

    logMsg("[BLE] Received message: " + value);

    // MTU Change
    if (value.startsWith("MTU:")) {
      int mtu = value.substring(4).toInt();
      negotiatedChunkSize = max(20, mtu - 6);
      logMsg("[BLE] MTU negotiated: " + String(mtu) +
             ", chunkSize=" + String(negotiatedChunkSize));
    } 

    // Commands
    else if (value == "IMAGE") {
      uint8_t token = 1;
      if (imageQueue && xQueueSend(imageQueue, &token, 0) == pdTRUE) {
        logMsg("[BLE] Image capture request queued");
      } else {
        logMsg("[BLE] Image capture request ignored (already queued)");
      }
    }
    else if (value == "AUDIO") {
      uint8_t token = 1;
      if (audioQueue && xQueueSend(audioQueue, &token, 0) == pdTRUE) {
        logMsg("[BLE] Audio recording request queued");
      } else {
        logMsg("[BLE] Audio request ignored (already queued)");
      }
    }
  }
};



// ============================================================================
// Setup Camera
// ============================================================================

#include "camera_pins.h"
void initCamera() {
  camera_config_t config = {};
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
  
  // ------------------------------------------ 
  // Image Resolution 
  // ------------------------------------------ 
  // FRAMESIZE_QVGA - 320x240 
  // FRAMESIZE_CIF - 400x296 
  // FRAMESIZE_VGA - 640x480 
  // FRAMESIZE_SVGA - 800x600 
  // FRAMESIZE_XGA - 1024x768 
  // FRAMESIZE_HD - 1280x720 
  // FRAMESIZE_UXGA - 1600x1200 
  config.frame_size = FRAMESIZE_HD; 
  // ------------------------------------------

  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = psramFound() ? 10 : 12;
  config.fb_count = psramFound() ? 2 : 1;

  if (esp_camera_init(&config) != ESP_OK) {
    logMsg("[ERROR] Camera initialization failed");
    while (true) delay(100);
  }
  logMsg("Camera ready");
}



// ============================================================================
// Setup Microphone / I2S
// ============================================================================

void initMicrophone() {
    // Set the RX pins for PDM microphone
    i2s.setPinsPdmRx(42, 41);

    // Begin I2S in PDM RX mode, 16-bit mono, 16kHz
    if (!i2s.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        logMsg("[ERROR] I2S initialization failed");
        while (true) delay(100);
    }
    logMsg("Microphone ready");
}



// ============================================================================
// Setup BLE
// ============================================================================

void initBLE() {
    BLEDevice::init("XIAO_ESP32S3");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    logMsg("BLE ready and advertising started");
}



// ============================================================================
// FreeOTS Tasks
// ============================================================================

// Image Task
void imageTask(void *param) {
  uint8_t req;
  unsigned long lastCaptureMs = 0;

  for (;;) {
    // Wait for image capture request
    if (xQueueReceive(imageQueue, &req, portMAX_DELAY) == pdTRUE) {

      // Debounce too fast command requests, avoid spamming
      unsigned long now = millis();
      if (now - lastCaptureMs < CAPTURE_DEBOUNCE_MS) {
        logMsg("[SENDER] Debounced duplicate capture");
        continue;
      }
      lastCaptureMs = now;

      // BLE Connection Check
      if (!deviceConnected) {
        logMsg("[SENDER] No BLE device; skipping capture");
        continue;
      }

      // Take picture
      camera_fb_t *fb = esp_camera_fb_get();

      // Force fresh frame. It retries once to get a camera frame and skips if capture still fails.
      if (!fb) {
        logMsg("[SENDER] Failed to get camera frame, retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
        fb = esp_camera_fb_get();
        if (!fb) {
          logMsg("[SENDER] Still failed to get frame; skipping");
          continue;
        }
      }

      // Takes the frame captured by the camera, extracts its size and data buffer, and then logs the size for debugging.
      size_t totalSize = fb->len;
      uint8_t *buffer = fb->buf;
      logMsg("[SENDER] Captured frame, size=" + String(totalSize));

      // ------------------------------------------------------------------------------------------
      // Send Image Data
      // ------------------------------------------------------------------------------------------
      // Header
      String header = "IMG_START:" + String(totalSize);
      pCharacteristic->setValue((uint8_t*)header.c_str(), header.length());
      pCharacteristic->notify();
      vTaskDelay(pdMS_TO_TICKS(20));

      // Chunks
      size_t sentBytes = 0;
      for (size_t i = 0; i < totalSize; i += negotiatedChunkSize) {
        int len = (i + negotiatedChunkSize > totalSize) ? (totalSize - i) : negotiatedChunkSize;
        pCharacteristic->setValue(buffer + i, len);
        pCharacteristic->notify();
        sentBytes += len;
        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
        logMsg("[BLE] Sent " + String(sentBytes) + "/" + String(totalSize) + " bytes");
      }

      // Footer
      String footer = "IMG_END";
      pCharacteristic->setValue((uint8_t*)footer.c_str(), footer.length());
      pCharacteristic->notify();
      // ------------------------------------------------------------------------------------------

      // Release memory when done
      esp_camera_fb_return(fb);

      // Drain extra queued requests
      uint8_t drain;
      while (xQueueReceive(imageQueue, &drain, 0) == pdTRUE) {}
    }
  }
  // Task cleanly stop itself when it no longer needs to run, freeing resources
  vTaskDelete(NULL);
}

// ============================================================================
// Audio Task
// ============================================================================

void audioTask(void *param) {
  uint8_t req;
  unsigned long lastCaptureMs = 0;

  for (;;) {
      // Wait for audio capture request
      if (xQueueReceive(audioQueue, &req, portMAX_DELAY) == pdTRUE) {

        // Debounce too fast command requests, avoid spamming
        unsigned long now = millis();
        if (now - lastCaptureMs < CAPTURE_DEBOUNCE_MS) {
            logMsg("[AUDIO] Debounced duplicate capture");
            continue;
        }
        lastCaptureMs = now;

        // BLE Connection Check
        if (!deviceConnected) {
            logMsg("[AUDIO] No BLE device; skipping audio capture");
            continue;
        }

        // Record audio for 5 seconds
        size_t wav_size = 0;
        uint8_t* wav_buffer = i2s.recordWAV(5, &wav_size);
        if (!wav_buffer) {
            logMsg("[AUDIO] Failed to record audio");
            continue;
        }
        logMsg("[AUDIO] Recorded audio size=" + String(wav_size));

        // ------------------------------------------------------------------------------------------
        // Send Audio Data
        // ------------------------------------------------------------------------------------------
        // Header
        String header = "AUD_START:" + String(wav_size);
        pCharacteristic->setValue((uint8_t*)header.c_str(), header.length());
        pCharacteristic->notify();
        vTaskDelay(pdMS_TO_TICKS(20));

        // Chunks
        size_t sentBytes = 0;
        for (size_t i = 0; i < wav_size; i += negotiatedChunkSize) {
            int len = (i + negotiatedChunkSize > wav_size) ? (wav_size - i) : negotiatedChunkSize;
            pCharacteristic->setValue(wav_buffer + i, len);
            pCharacteristic->notify();
            sentBytes += len;
            vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
            logMsg("[AUDIO] Sent " + String(sentBytes) + "/" + String(wav_size) + " bytes");
        }

        // Footer
        String footer = "AUD_END";
        pCharacteristic->setValue((uint8_t*)footer.c_str(), footer.length());
        pCharacteristic->notify();
        // ------------------------------------------------------------------------------------------

        // Free recorded audio memory
        delete[] wav_buffer;

        // Drain extra queued requests
        uint8_t drain;
        while (xQueueReceive(audioQueue, &drain, 0) == pdTRUE) {}
      }
  }
  // Task cleanly stop itself when it no longer needs to run, freeing resources
  vTaskDelete(NULL);
}



// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    logMsg("Starting ESP32 BLE Camera & Audio Server...");

    // Initialize BLE
    initBLE();

    // Initialize Camera
    initCamera();

    // Initialize Microphone
    initMicrophone();

    // Create Queues
    imageQueue = xQueueCreate(CAPTURE_QUEUE_LEN, sizeof(uint8_t)); logMsg("Image Queue Created");
    audioQueue = xQueueCreate(CAPTURE_QUEUE_LEN, sizeof(uint8_t)); logMsg("Audio Queue Created");

    // Create Tasks
    xTaskCreatePinnedToCore(imageTask, "ImageTask", 16384, nullptr, 1, &imageTaskHandle, 1);  logMsg("Image Task Created");
    xTaskCreatePinnedToCore(audioTask, "AudioTask", 16384, nullptr, 1, &audioTaskHandle, 1);  logMsg("Audio Task Created");
}



// ============================================================================
// Main Loop
// ============================================================================

void loop() {





  // !! TEMPORARY !!
  // Serial Commands
  if (deviceConnected && Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "IMAGE") {
      uint8_t token = 1;
      xQueueSend(imageQueue, &token, 0);
      logMsg("[CMD] Image capture request queued");
    } 
    else if (cmd == "AUDIO") {
      uint8_t token = 1;
      xQueueSend(audioQueue, &token, 0);
      logMsg("[CMD] Audio capture request queued");
    }
    else {
      logMsg("[CMD] Unknown command: " + cmd);
    }
  }
  delay(10);
}
