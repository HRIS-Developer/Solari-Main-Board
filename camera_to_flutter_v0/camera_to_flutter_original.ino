#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_camera.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CAPTURE_QUEUE_LEN 1
#define CAPTURE_DEBOUNCE_MS 500
#define SEND_DELAY_BETWEEN_CHUNKS_MS 10
#define LED_PIN LED_BUILTIN

// ============================================================================
// BLE Globals
// ============================================================================
volatile bool deviceConnected = false;
BLECharacteristic* pCharacteristic;
volatile int negotiatedChunkSize = 23;
volatile bool isSending = false;

// Task handle for sender
QueueHandle_t captureQueue = NULL;
TaskHandle_t senderTaskHandle = NULL;
static uint32_t frameIdCounter = 0;

// ============================================================================
// Logging Helper
// ============================================================================
void log(const String &msg) {
  Serial.printf("[%lu] %s\n", millis(), msg.c_str());
}

// ============================================================================
// BLE Server Callbacks
// ============================================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    log("[BLE] Device connected!");
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    log("[BLE] Device disconnected!");
    deviceConnected = false;
    BLEDevice::startAdvertising();
    log("[BLE] Restarting advertising...");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = String(pCharacteristic->getValue().c_str());
    value.trim();
    value.toUpperCase();

    log("[BLE] Received message: " + value);

    if (value.startsWith("MTU:")) {
      int mtu = value.substring(4).toInt();
      negotiatedChunkSize = max(20, mtu - 6);
      log("[BLE] MTU negotiated: " + String(mtu) +
          ", chunkSize=" + String(negotiatedChunkSize));
    }
    else if (value == "CAPTURE") {
      uint8_t token = 1;
      // non-blocking send, queue length = 1 -> coalesces duplicates
      if (captureQueue && xQueueSend(captureQueue, &token, 0) == pdTRUE) {
        log("[BLE] Capture request queued");
      } else {
        log("[BLE] Capture request ignored (already queued)");
      }
    }
  }
};

// ============================================================================
// Setup Camera
// ============================================================================
#include "camera_pins.h"

void initCamera() {
  log("Initializing camera...");
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
  config.frame_size = FRAMESIZE_HD;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    log("PSRAM detected, using higher frame buffer settings");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    log("[ERROR] Camera init failed with code: " + String(err));
    while (true) delay(100);
  }
  log("Camera ready");
}

// ============================================================================
// Send Image via BLE Task
// ============================================================================
void senderTask(void *param) {
  uint8_t req;
  unsigned long lastCaptureMs = 0;

  for (;;) {
    // Wait for a capture request
    if (xQueueReceive(captureQueue, &req, portMAX_DELAY) == pdTRUE) {

      // debounce
      unsigned long now = millis();
      if (now - lastCaptureMs < CAPTURE_DEBOUNCE_MS) {
        log("[SENDER] Debounced duplicate capture");
        continue;
      }
      lastCaptureMs = now;

      // Ensure client present
      if (!deviceConnected) {
        log("[SENDER] No BLE device; skipping capture");
        continue;
      }

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        log("[SENDER] Camera capture failed");
        continue;
      }

      size_t totalSize = fb->len;
      uint8_t *buffer = fb->buf;
      log("[SENDER] Captured frame, size=" + String(totalSize));

      unsigned long startTime = millis();

      isSending = true;

      // --- Send header (old format) ---
      String header = "IMG_START:" + String(totalSize);
      pCharacteristic->setValue((uint8_t*)header.c_str(), header.length());
      pCharacteristic->notify();
      vTaskDelay(pdMS_TO_TICKS(20));
      log("[BLE] Sent image header");

      // --- Send chunks ---
      size_t sentBytes = 0;
      for (size_t i = 0; i < totalSize; i += negotiatedChunkSize) {
        int len = (i + negotiatedChunkSize > totalSize) ? (totalSize - i) : negotiatedChunkSize;
        pCharacteristic->setValue(buffer + i, len);
        pCharacteristic->notify();
        sentBytes += len;

        // Delay to avoid overflow
        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));

        // Log progress every ~500 bytes
        if (sentBytes % 500 <= negotiatedChunkSize) {
          log("[BLE] Sent " + String(sentBytes) + "/" + String(totalSize) + " bytes");
        }
      }

      // --- Send footer (old format) ---
      String footer = "IMG_END";
      pCharacteristic->setValue((uint8_t*)footer.c_str(), footer.length());
      pCharacteristic->notify();
      log("[BLE] Sent image footer, total size: " + String(totalSize));

      esp_camera_fb_return(fb);
      log("[SENDER] fb returned");

      unsigned long endTime = millis();
      log("[TIME] Total time to send image over BLE: " + String(endTime - startTime) + " ms");

      // Drain extra queued requests (coalesce duplicates)
      uint8_t drain;
      while (xQueueReceive(captureQueue, &drain, 0) == pdTRUE) {
        log("[SENDER] Coalescing extra queued capture (dropped)");
      }

      isSending = false;
    }
  }
  vTaskDelete(NULL); // never reached
}
// ============================================================================
// Beep Task (Blink LED while sending)
// ============================================================================
void beepTask(void *param) {
  pinMode(LED_PIN, OUTPUT);

  for (;;) {
    if (isSending) {
      digitalWrite(LED_PIN, LOW);   // LED ON
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(LED_PIN, HIGH);  // LED OFF
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      digitalWrite(LED_PIN, HIGH);  // Ensure LED is OFF when idle
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  log("Starting ESP32 BLE Camera Server...");

  BLEDevice::init("XIAO_ESP32S3");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setValue("Hello World!");
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  log("[BLE] Advertising started...");

  initCamera();

  captureQueue = xQueueCreate(CAPTURE_QUEUE_LEN, sizeof(uint8_t));
  if (captureQueue == NULL) {
    log("[ERROR] Failed to create capture queue");
  } else {
    xTaskCreatePinnedToCore(
      senderTask,
      "SenderTask",
      16384,     // give it a reasonable stack (increase if needed)
      NULL,
      1,
      &senderTaskHandle,
      1
    );
    log("[SYSTEM] SenderTask created");
        xTaskCreatePinnedToCore(
      beepTask,
      "BeepTask",
      2048,      // small stack is fine
      NULL,
      1,
      NULL,
      1
    );
    log("[SYSTEM] BeepTask created");
  }
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  if (deviceConnected && Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    if (cmd == "CAPTURE") {
      uint8_t token = 1;
      if (captureQueue && xQueueSend(captureQueue, &token, 0) == pdTRUE) {
        log("[CMD] Capture request queued");
      } else {
        log("[CMD] Capture request ignored (already queued)");
      }
    } else {
      log("[CMD] Unknown command: " + cmd);
    }
  }
  delay(10);
}
