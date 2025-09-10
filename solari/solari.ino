#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_camera.h"
#include "ESP_I2S.h"
#include "esp_adc_cal.h"

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
QueueHandle_t vqaQueue = nullptr;
TaskHandle_t imageTaskHandle = nullptr;
TaskHandle_t audioTaskHandle = nullptr;
TaskHandle_t vqaTaskHandle = nullptr;

// VQA state management
struct VQAState {
  bool imageTransmissionComplete = false;
  bool audioRecordingInProgress = false;
  bool audioRecordingComplete = false;
  uint8_t* audioBuffer = nullptr;
  size_t audioSize = 0;
  unsigned long audioRecordingStartTime = 0;
  TaskHandle_t backgroundAudioTaskHandle = nullptr;
};
VQAState vqaState;



// ============================================================================
// Enhanced Logging System
// ============================================================================

// Log levels
enum LogLevel {
  LOG_DEBUG = 0,
  LOG_INFO = 1,
  LOG_WARN = 2,
  LOG_ERROR = 3
};

// Current log level (change to LOG_DEBUG for more verbose output)
LogLevel currentLogLevel = LOG_INFO;

// Log level strings
const char* logLevelStr[] = {"DEBUG", "INFO", "WARN", "ERROR"};

// Enhanced logging function
void log(LogLevel level, const String &component, const String &msg) {
  if (level >= currentLogLevel) {
    Serial.printf("[%8lu] [%s] [%s] %s\n", 
                  millis(), 
                  logLevelStr[level], 
                  component.c_str(), 
                  msg.c_str());
  }
}

// Convenience functions for different log levels
void logInfo(const String &component, const String &msg) {
  log(LOG_INFO, component, msg);
}

void logWarn(const String &component, const String &msg) {
  log(LOG_WARN, component, msg);
}

void logError(const String &component, const String &msg) {
  log(LOG_ERROR, component, msg);
}

void logDebug(const String &component, const String &msg) {
  log(LOG_DEBUG, component, msg);
}

// Memory info helper
void logMemory(const String &component) {
  if (currentLogLevel <= LOG_DEBUG) {
    log(LOG_DEBUG, component, "Free heap: " + String(ESP.getFreeHeap()) + 
        " bytes, PSRAM: " + String(ESP.getFreePsram()) + " bytes");
  }
}

// Progress logger for data transfers with visual progress bar
void logProgress(const String &component, size_t current, size_t total) {
  if (currentLogLevel <= LOG_INFO) {
    int percent = (current * 100) / total;
    
    // Create progress bar
    const int barWidth = 20;
    int filled = (percent * barWidth) / 100;
    String progressBar = "[";
    
    for (int i = 0; i < barWidth; i++) {
      if (i < filled) {
        progressBar += "█";
      } else {
        progressBar += "░";
      }
    }
    progressBar += "]";
    
    log(LOG_INFO, component, progressBar + " (" + String(percent) + "%) " + 
        String(current) + "/" + String(total) + " bytes");
  }
}

// Enhanced progress logger with transfer rate
void logProgressWithRate(const String &component, size_t current, size_t total, unsigned long startTime) {
  if (currentLogLevel <= LOG_INFO) {
    int percent = (current * 100) / total;
    unsigned long elapsed = millis() - startTime;
    float rate = 0;
    if (elapsed > 0) {
      rate = (current / 1024.0) / (elapsed / 1000.0); // KB/s
    }
    
    // Create progress bar
    const int barWidth = 20;
    int filled = (percent * barWidth) / 100;
    String progressBar = "[";
    
    for (int i = 0; i < barWidth; i++) {
      if (i < filled) {
        progressBar += "█";
      } else {
        progressBar += "░";
      }
    }
    progressBar += "]";
    
    log(LOG_INFO, component, progressBar + " (" + String(percent) + "%) " + 
        String(current) + "/" + String(total) + " bytes - " + String(rate, 1) + " KB/s");
  }
}



// ============================================================================
// BLE Server Callbacks
// ============================================================================

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    logInfo("BLE", "Client connected");
    deviceConnected = true;
    logMemory("BLE");
  }
  void onDisconnect(BLEServer* pServer) override {
    logWarn("BLE", "Client disconnected");
    deviceConnected = false;
    BLEDevice::startAdvertising();
    logInfo("BLE", "Advertising restarted");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = String(characteristic->getValue().c_str());
    value.trim();
    value.toUpperCase();

    logDebug("BLE", "Raw received: '" + value + "'");

    // MTU Change
    if (value.startsWith("MTU:")) {
      int mtu = value.substring(4).toInt();
      if (mtu >= 23 && mtu <= 512) {
        negotiatedChunkSize = max(20, mtu - 6);
        logInfo("BLE", "MTU negotiated: " + String(mtu) + 
               " bytes, chunk size: " + String(negotiatedChunkSize) + " bytes");
      } else {
        logWarn("BLE", "Invalid MTU value: " + String(mtu));
      }
    } 

    // Commands
    else if (value == "IMAGE") {
      uint8_t token = 1;
      if (imageQueue && xQueueSend(imageQueue, &token, 0) == pdTRUE) {
        logInfo("CMD", "Image capture queued");
      } else {
        logWarn("CMD", "Image capture ignored - queue full");
      }
    }
    else if (value == "AUDIO") {
      uint8_t token = 1;
      if (audioQueue && xQueueSend(audioQueue, &token, 0) == pdTRUE) {
        logInfo("CMD", "Audio recording queued");
      } else {
        logWarn("CMD", "Audio capture ignored - queue full");
      }
    }
    else if (value == "VQA") {
      uint8_t token = 1;
      if (vqaQueue && xQueueSend(vqaQueue, &token, 0) == pdTRUE) {
        logInfo("CMD", "VQA operation queued");
      } else {
        logWarn("CMD", "VQA operation ignored - queue full");
      }
    }
    else {
      logWarn("CMD", "Unknown command received: '" + value + "'");
    }
  }
};



// ============================================================================
// Setup Camera
// ============================================================================

#include "camera_pins.h"
void initCamera() {
  logInfo("CAM", "Initializing camera...");
  
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

  logDebug("CAM", "PSRAM found: " + String(psramFound() ? "Yes" : "No"));
  logDebug("CAM", "Frame size: HD (1280x720), JPEG quality: " + String(config.jpeg_quality));

  if (esp_camera_init(&config) != ESP_OK) {
    logError("CAM", "Camera initialization failed!");
    while (true) delay(100);
  }
  
  logInfo("CAM", "Camera ready");
  logMemory("CAM");
}



// ============================================================================
// Setup Microphone / I2S
// ============================================================================

void initMicrophone() {
    logInfo("MIC", "Initializing microphone...");
    
    // Set the RX pins for PDM microphone
    i2s.setPinsPdmRx(42, 41);
    logDebug("MIC", "PDM pins configured: CLK=42, DATA=41");

    // Begin I2S in PDM RX mode, 4-bit mono, 4kHz to 16-bit mono, 16kHz
    if (!i2s.begin(I2S_MODE_PDM_RX, 8000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        logError("MIC", "I2S initialization failed!");
        while (true) delay(100);
    }
    
    logInfo("MIC", "Microphone ready");
    logMemory("MIC");
}



// ============================================================================
// Setup BLE
// ============================================================================

void initBLE() {
    logInfo("BLE", "Initializing Bluetooth LE...");
    
    BLEDevice::init("XIAO_ESP32S3");
    BLEDevice::setMTU(517);
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

    logInfo("BLE", "Ready and advertising as 'XIAO_ESP32S3'");
    logDebug("BLE", "Service UUID: " + String(SERVICE_UUID));
    logMemory("BLE");
}



// ============================================================================
// FreeOTS Tasks
// ============================================================================

// Image Task
void imageTask(void *param) {
  uint8_t req;
  unsigned long lastCaptureMs = 0;

  logInfo("TASK", "Image task started");

  for (;;) {
    // Wait for image capture request
    if (xQueueReceive(imageQueue, &req, portMAX_DELAY) == pdTRUE) {

      // Debounce too fast command requests, avoid spamming
      unsigned long now = millis();
      if (now - lastCaptureMs < CAPTURE_DEBOUNCE_MS) {
        logWarn("IMG", "Request debounced (too frequent)");
        continue;
      }
      lastCaptureMs = now;

      // BLE Connection Check
      if (!deviceConnected) {
        logWarn("IMG", "No BLE client connected - skipping capture");
        continue;
      }

      logInfo("IMG", "Starting image capture...");
      logMemory("IMG");

      // Take picture
      camera_fb_t *fb = esp_camera_fb_get();

      // Force fresh frame. It retries once to get a camera frame and skips if capture still fails.
      if (!fb) {
        logWarn("IMG", "First capture failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
        fb = esp_camera_fb_get();
        if (!fb) {
          logError("IMG", "Camera capture failed after retry");
          continue;
        }
      }

      // Takes the frame captured by the camera, extracts its size and data buffer, and then logs the size for debugging.
      size_t totalSize = fb->len;
      uint8_t *buffer = fb->buf;
      logInfo("IMG", "Captured " + String(totalSize) + " bytes (" + 
              String(totalSize/1024) + " KB)");

      // ------------------------------------------------------------------------------------------
      // Send Image Data
      // ------------------------------------------------------------------------------------------
      logInfo("IMG", "Starting BLE transmission...");
      
      // Header
      String header = "IMG_START:" + String(totalSize);
      pCharacteristic->setValue((uint8_t*)header.c_str(), header.length());
      pCharacteristic->notify();
      vTaskDelay(pdMS_TO_TICKS(20));
      logDebug("IMG", "Header sent: " + header);

      // Chunks
      size_t sentBytes = 0;
      size_t chunkCount = 0;
      unsigned long transferStart = millis();
      
      for (size_t i = 0; i < totalSize; i += negotiatedChunkSize) {
        int len = (i + negotiatedChunkSize > totalSize) ? (totalSize - i) : negotiatedChunkSize;
        pCharacteristic->setValue(buffer + i, len);
        pCharacteristic->notify();
        sentBytes += len;
        chunkCount++;
        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
        
        // Log progress every 10 chunks or at the end with visual progress bar
        if (chunkCount % 10 == 0 || sentBytes >= totalSize) {
          logProgressWithRate("IMG", sentBytes, totalSize, transferStart);
        }
      }

      unsigned long transferTime = millis() - transferStart;
      float transferRate = (totalSize / 1024.0) / (transferTime / 1000.0);
      
      // Footer
      String footer = "IMG_END";
      pCharacteristic->setValue((uint8_t*)footer.c_str(), footer.length());
      pCharacteristic->notify();
      logDebug("IMG", "Footer sent: " + footer);
      // ------------------------------------------------------------------------------------------

      logInfo("IMG", "Transfer complete in " + String(transferTime) + 
              "ms (" + String(transferRate, 1) + " KB/s)");

      // Release memory when done
      esp_camera_fb_return(fb);
      logMemory("IMG");

      // Drain extra queued requests
      uint8_t drain;
      int drained = 0;
      while (xQueueReceive(imageQueue, &drain, 0) == pdTRUE) {
        drained++;
      }
      if (drained > 0) {
        logDebug("IMG", "Drained " + String(drained) + " queued requests");
      }
    }
  }
  // Task cleanly stop itself when it no longer needs to run, freeing resources
  vTaskDelete(NULL);
}

// Audio Task
void audioTask(void *param) {
  uint8_t req;
  unsigned long lastCaptureMs = 0;

  logInfo("TASK", "Audio task started");

  for (;;) {
      // Wait for audio capture request
      if (xQueueReceive(audioQueue, &req, portMAX_DELAY) == pdTRUE) {

        // Debounce too fast command requests, avoid spamming
        unsigned long now = millis();
        if (now - lastCaptureMs < CAPTURE_DEBOUNCE_MS) {
            logWarn("AUD", "Request debounced (too frequent)");
            continue;
        }
        lastCaptureMs = now;

        // BLE Connection Check
        if (!deviceConnected) {
            logWarn("AUD", "No BLE client connected - skipping recording");
            continue;
        }

        logInfo("AUD", "Starting 5-second audio recording...");
        logMemory("AUD");

        // Record audio for 5 seconds
        unsigned long recordStart = millis();
        size_t wav_size = 0;
        uint8_t* wav_buffer = i2s.recordWAV(5, &wav_size);
        unsigned long recordTime = millis() - recordStart;
        
        if (!wav_buffer) {
            logError("AUD", "Audio recording failed");
            continue;
        }
        
        logInfo("AUD", "Recorded " + String(wav_size) + " bytes (" + 
                String(wav_size/1024) + " KB) in " + String(recordTime) + "ms");

        // ------------------------------------------------------------------------------------------
        // Send Audio Data
        // ------------------------------------------------------------------------------------------
        logInfo("AUD", "Starting BLE transmission...");
        
        // Header
        String header = "AUD_START:" + String(wav_size);
        pCharacteristic->setValue((uint8_t*)header.c_str(), header.length());
        pCharacteristic->notify();
        vTaskDelay(pdMS_TO_TICKS(20));
        logDebug("AUD", "Header sent: " + header);

        // Chunks
        size_t sentBytes = 0;
        size_t chunkCount = 0;
        unsigned long transferStart = millis();
        
        for (size_t i = 0; i < wav_size; i += negotiatedChunkSize) {
            int len = (i + negotiatedChunkSize > wav_size) ? (wav_size - i) : negotiatedChunkSize;
            pCharacteristic->setValue(wav_buffer + i, len);
            pCharacteristic->notify();
            sentBytes += len;
            chunkCount++;
            vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
            
            // Log progress every 10 chunks or at the end with visual progress bar
            if (chunkCount % 10 == 0 || sentBytes >= wav_size) {
              logProgressWithRate("AUD", sentBytes, wav_size, transferStart);
            }
        }

        unsigned long transferTime = millis() - transferStart;
        float transferRate = (wav_size / 1024.0) / (transferTime / 1000.0);

        // Footer
        String footer = "AUD_END";
        pCharacteristic->setValue((uint8_t*)footer.c_str(), footer.length());
        pCharacteristic->notify();
        logDebug("AUD", "Footer sent: " + footer);
        // ------------------------------------------------------------------------------------------

        logInfo("AUD", "Transfer complete in " + String(transferTime) + 
                "ms (" + String(transferRate, 1) + " KB/s)");

        // Free recorded audio memory
        delete[] wav_buffer;
        logMemory("AUD");

        // Drain extra queued requests
        uint8_t drain;
        int drained = 0;
        while (xQueueReceive(audioQueue, &drain, 0) == pdTRUE) {
          drained++;
        }
        if (drained > 0) {
          logDebug("AUD", "Drained " + String(drained) + " queued requests");
        }
      }
  }
  // Task cleanly stop itself when it no longer needs to run, freeing resources
  vTaskDelete(NULL);
}

// Background Audio Recording Task for VQA
void backgroundAudioRecordingTask(void *param) {
  logInfo("VQA-AUD", "Background audio recording task started");
  
  // Record audio for 5 seconds
  size_t audioBufferSize = 0;
  vqaState.audioBuffer = i2s.recordWAV(3, &audioBufferSize);
  vqaState.audioSize = audioBufferSize;
  
  unsigned long audioRecordTime = millis() - vqaState.audioRecordingStartTime;
  vqaState.audioRecordingInProgress = false;
  vqaState.audioRecordingComplete = true;
  
  if (vqaState.audioBuffer) {
    logInfo("VQA-AUD", "Background audio recording completed: " + String(vqaState.audioSize) + 
            " bytes (" + String(vqaState.audioSize/1024) + " KB) in " + String(audioRecordTime) + "ms");
  } else {
    logError("VQA-AUD", "Background audio recording failed");
  }
  
  // Task auto-cleanup
  vqaState.backgroundAudioTaskHandle = nullptr;
  vTaskDelete(NULL);
}

// VQA Task - Coordinates Image + Audio
void vqaTask(void *param) {
  uint8_t req;
  unsigned long lastCaptureMs = 0;

  logInfo("TASK", "VQA task started");

  for (;;) {
    // Wait for VQA request
    if (xQueueReceive(vqaQueue, &req, portMAX_DELAY) == pdTRUE) {

      // Debounce too fast command requests
      unsigned long now = millis();
      if (now - lastCaptureMs < CAPTURE_DEBOUNCE_MS) {
        logWarn("VQA", "Request debounced (too frequent)");
        continue;
      }
      lastCaptureMs = now;

      // BLE Connection Check
      if (!deviceConnected) {
        logWarn("VQA", "No BLE client connected - skipping VQA");
        continue;
      }

      logInfo("VQA", "Starting VQA operation (Image + Audio)...");
      logMemory("VQA");

      // Reset VQA state
      vqaState.imageTransmissionComplete = false;
      vqaState.audioRecordingInProgress = false;
      vqaState.audioRecordingComplete = false;
      if (vqaState.audioBuffer) {
        delete[] vqaState.audioBuffer;
        vqaState.audioBuffer = nullptr;
      }
      vqaState.audioSize = 0;

      // Step 1: Start background audio recording task
      logInfo("VQA", "Starting background audio recording task...");
      vqaState.audioRecordingInProgress = true;
      vqaState.audioRecordingStartTime = millis();
      
      BaseType_t audioTaskResult = xTaskCreatePinnedToCore(
        backgroundAudioRecordingTask, 
        "VQA_AudioTask", 
        8192, 
        nullptr, 
        2, // Higher priority than VQA task
        &vqaState.backgroundAudioTaskHandle, 
        0  // Run on core 0
      );
      
      if (audioTaskResult != pdPASS) {
        logError("VQA", "Failed to create background audio recording task");
        continue;
      }

      // Step 2: Capture image while audio is recording in background
      logInfo("VQA", "Starting image capture (audio recording in background)...");
      
      // Take picture
      camera_fb_t *fb = esp_camera_fb_get();
      
      // Retry once if first capture fails
      if (!fb) {
        logWarn("VQA", "First capture failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
        fb = esp_camera_fb_get();
        if (!fb) {
          logError("VQA", "Camera capture failed after retry");
          // Wait for audio task to complete and clean up
          while (vqaState.audioRecordingInProgress) {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          if (vqaState.audioBuffer) {
            delete[] vqaState.audioBuffer;
            vqaState.audioBuffer = nullptr;
          }
          continue;
        }
      }

      size_t imageSize = fb->len;
      uint8_t *imageBuffer = fb->buf;
      logInfo("VQA", "Captured " + String(imageSize) + " bytes (" + 
              String(imageSize/1024) + " KB) - Audio still recording in background");

      // Step 3: Send image data via BLE (while audio continues recording)
      logInfo("VQA", "Starting image transmission (audio recording in background)...");
      
      // Image header
      String imageHeader = "VQA_IMG_START:" + String(imageSize);
      pCharacteristic->setValue((uint8_t*)imageHeader.c_str(), imageHeader.length());
      pCharacteristic->notify();
      vTaskDelay(pdMS_TO_TICKS(20));
      logDebug("VQA", "Image header sent: " + imageHeader);

      // Send image chunks
      size_t sentBytes = 0;
      size_t chunkCount = 0;
      unsigned long transferStart = millis();
      
      for (size_t i = 0; i < imageSize; i += negotiatedChunkSize) {
        int len = (i + negotiatedChunkSize > imageSize) ? (imageSize - i) : negotiatedChunkSize;
        pCharacteristic->setValue(imageBuffer + i, len);
        pCharacteristic->notify();
        sentBytes += len;
        chunkCount++;
        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
        
        // Log progress every 10 chunks or at the end with visual progress bar
        if (chunkCount % 10 == 0 || sentBytes >= imageSize) {
          logProgressWithRate("VQA-IMG", sentBytes, imageSize, transferStart);
        }
      }

      unsigned long imageTransferTime = millis() - transferStart;
      float imageTransferRate = (imageSize / 1024.0) / (imageTransferTime / 1000.0);
      
      // Image footer
      String imageFooter = "VQA_IMG_END";
      pCharacteristic->setValue((uint8_t*)imageFooter.c_str(), imageFooter.length());
      pCharacteristic->notify();
      vTaskDelay(pdMS_TO_TICKS(20));
      logDebug("VQA", "Image footer sent: " + imageFooter);

      logInfo("VQA", "Image transfer complete in " + String(imageTransferTime) + 
              "ms (" + String(imageTransferRate, 1) + " KB/s)");

      // Release camera frame buffer
      esp_camera_fb_return(fb);
      vqaState.imageTransmissionComplete = true;

      // Step 4: Wait for audio recording to complete (if still in progress)
      if (vqaState.audioRecordingInProgress) {
        logInfo("VQA", "Waiting for audio recording to complete...");
        while (vqaState.audioRecordingInProgress && !vqaState.audioRecordingComplete) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
      }

      // Step 5: Send audio data via BLE
      if (vqaState.audioRecordingComplete && vqaState.audioBuffer && vqaState.audioSize > 0) {
        logInfo("VQA", "Starting audio transmission...");
        
        // Audio header
        String audioHeader = "VQA_AUD_START:" + String(vqaState.audioSize);
        pCharacteristic->setValue((uint8_t*)audioHeader.c_str(), audioHeader.length());
        pCharacteristic->notify();
        vTaskDelay(pdMS_TO_TICKS(20));
        logDebug("VQA", "Audio header sent: " + audioHeader);

        // Send audio chunks
        sentBytes = 0;
        chunkCount = 0;
        transferStart = millis();
        
        for (size_t i = 0; i < vqaState.audioSize; i += negotiatedChunkSize) {
          int len = (i + negotiatedChunkSize > vqaState.audioSize) ? (vqaState.audioSize - i) : negotiatedChunkSize;
          pCharacteristic->setValue(vqaState.audioBuffer + i, len);
          pCharacteristic->notify();
          sentBytes += len;
          chunkCount++;
          vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
          
          // Log progress every 10 chunks or at the end with visual progress bar
          if (chunkCount % 10 == 0 || sentBytes >= vqaState.audioSize) {
            logProgressWithRate("VQA-AUD", sentBytes, vqaState.audioSize, transferStart);
          }
        }

        unsigned long audioTransferTime = millis() - transferStart;
        float audioTransferRate = (vqaState.audioSize / 1024.0) / (audioTransferTime / 1000.0);

        // Audio footer
        String audioFooter = "VQA_AUD_END";
        pCharacteristic->setValue((uint8_t*)audioFooter.c_str(), audioFooter.length());
        pCharacteristic->notify();
        logDebug("VQA", "Audio footer sent: " + audioFooter);

        logInfo("VQA", "Audio transfer complete in " + String(audioTransferTime) + 
                "ms (" + String(audioTransferRate, 1) + " KB/s)");

        // Clean up audio buffer
        delete[] vqaState.audioBuffer;
        vqaState.audioBuffer = nullptr;
        vqaState.audioSize = 0;
      } else {
        logWarn("VQA", "Audio recording failed or no audio data available");
      }

      // Send VQA completion signal
      String vqaComplete = "VQA_COMPLETE";
      pCharacteristic->setValue((uint8_t*)vqaComplete.c_str(), vqaComplete.length());
      pCharacteristic->notify();
      logInfo("VQA", "VQA operation completed successfully");

      logMemory("VQA");

      // Drain extra queued requests
      uint8_t drain;
      int drained = 0;
      while (xQueueReceive(vqaQueue, &drain, 0) == pdTRUE) {
        drained++;
      }
      if (drained > 0) {
        logDebug("VQA", "Drained " + String(drained) + " queued requests");
      }
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
    
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⠿⠿⣷⣦⣀⠀⢀⣀⣀⣤⣤⣤⣤⣤⣄⣀⡀⢀⣤⣶⠿⠿⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⡟⢸⠀⠀⠉⠻⠿⠛⠛⠋⠉⠉⠉⠉⠉⠙⠛⠻⠿⠋⠁⠀⠀⢻⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⡇⠘⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡀⠀⢸⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⠛⠀⠀⠐⢠⣦⣦⣦⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣄⠀⠀⠙⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⡟⠀⠀⠀⠀⠀⠸⣿⣿⣿⣿⣿⣿⣿⣿⠀⠸⣿⣿⣿⣿⣿⣿⣿⡇⡄⠀⠀⠈⢿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠇⠀⠀⠀⠀⠀⠀⢻⣿⣿⣿⣿⣿⡿⠁⣼⣄⠙⠿⣿⣿⣿⡿⠟⠀⠀⠀⠀⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⡆⠀⠀⠀⠀⠀⣴⡿⠛⢿⡏⠉⠈⠰⡿⠛⠻⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣿⡀⢀⠀⠀⣸⣿⠁⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢙⣿⣶⡄⡾⠟⠃⠀⠀⣾⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⠿⣯⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⠃⠀⠀⠀⠀⠀⠀⣴⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⠀⠀⠀⠘⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    Serial.println("⢀⡀⡀⣀⣀⣀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀⠊⠀⠀⠀⠀⠀⢀⣀⠀⠀⠀⠀⠀⠀⣤⣾⠿⠛⠃⠀⠀⠀⣸⣷⠀⠀⠀⠀⣀⣀⡀⡀⠀");
    Serial.println("⠺⠿⠛⠛⠛⠛⠿⠿⠿⠿⠿⠿⠿⠿⠿⠷⣶⢶⡶⡶⠶⠿⠿⠿⠿⠟⠛⠛⠻⠿⠿⠿⢿⣿⣀⣠⣤⣤⣤⣤⣶⠿⠛⠛⠛⠛⠛⠛⠛⠛⠻⠃");
    Serial.println("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀");
    logInfo("SYS", "Compile time: " + String(__DATE__) + " " + String(__TIME__));
    logMemory("SYS");

    // Initialize BLE
    initBLE();

    // Initialize Camera
    initCamera();

    // Initialize Microphone
    initMicrophone();

    // Create Queues
    imageQueue = xQueueCreate(CAPTURE_QUEUE_LEN, sizeof(uint8_t)); 
    if (imageQueue) {
      logInfo("SYS", "Image queue created (size: " + String(CAPTURE_QUEUE_LEN) + ")");
    } else {
      logError("SYS", "Failed to create image queue");
    }
    
    audioQueue = xQueueCreate(CAPTURE_QUEUE_LEN, sizeof(uint8_t)); 
    if (audioQueue) {
      logInfo("SYS", "Audio queue created (size: " + String(CAPTURE_QUEUE_LEN) + ")");
    } else {
      logError("SYS", "Failed to create audio queue");
    }
    
    vqaQueue = xQueueCreate(CAPTURE_QUEUE_LEN, sizeof(uint8_t)); 
    if (vqaQueue) {
      logInfo("SYS", "VQA queue created (size: " + String(CAPTURE_QUEUE_LEN) + ")");
    } else {
      logError("SYS", "Failed to create VQA queue");
    }

    // Create Tasks
    BaseType_t result1 = xTaskCreatePinnedToCore(imageTask, "ImageTask", 16384, nullptr, 1, &imageTaskHandle, 1);
    if (result1 == pdPASS) {
      logInfo("SYS", "Image task created on core 1");
    } else {
      logError("SYS", "Failed to create image task");
    }
    
    BaseType_t result2 = xTaskCreatePinnedToCore(audioTask, "AudioTask", 16384, nullptr, 1, &audioTaskHandle, 1);
    if (result2 == pdPASS) {
      logInfo("SYS", "Audio task created on core 1");
    } else {
      logError("SYS", "Failed to create audio task");
    }
    
    BaseType_t result3 = xTaskCreatePinnedToCore(vqaTask, "VQATask", 20480, nullptr, 1, &vqaTaskHandle, 1);
    if (result3 == pdPASS) {
      logInfo("SYS", "VQA task created on core 1");
    } else {
      logError("SYS", "Failed to create VQA task");
    }
    
    logInfo("SYS", "=== System initialization complete ===");
    logMemory("SYS");
}



// ============================================================================
// Main Loop
// ============================================================================

void loop() {


  // Monitor Temperature
  // float temp = temperatureRead(); // returns °C
  // Serial.print("Internal Temperature: ");
  // Serial.print(temp);
  // Serial.println(" °C");

  // delay(1000); // Update every 1 second

  // !! TEMPORARY !!
  // Serial Commands for testing
  if (deviceConnected && Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "IMAGE") {
      uint8_t token = 1;
      if (xQueueSend(imageQueue, &token, 0) == pdTRUE) {
        logInfo("CMD", "Image capture queued via serial");
      } else {
        logWarn("CMD", "Image queue full - request ignored");
      }
    } 
    else if (cmd == "AUDIO") {
      uint8_t token = 1;
      if (xQueueSend(audioQueue, &token, 0) == pdTRUE) {
        logInfo("CMD", "Audio recording queued via serial");
      } else {
        logWarn("CMD", "Audio queue full - request ignored");
      }
    } 
    else if (cmd == "VQA") {
      uint8_t token = 1;
      if (xQueueSend(vqaQueue, &token, 0) == pdTRUE) {
        logInfo("CMD", "VQA operation queued via serial");
      } else {
        logWarn("CMD", "VQA queue full - request ignored");
      }
    }
    else if (cmd == "STATUS") {
      logInfo("SYS", "=== System Status ===");
      logInfo("SYS", "BLE connected: " + String(deviceConnected ? "Yes" : "No"));
      logInfo("SYS", "Chunk size: " + String(negotiatedChunkSize) + " bytes");
      logInfo("SYS", "Image queue: " + String(uxQueueSpacesAvailable(imageQueue)) + "/" + String(CAPTURE_QUEUE_LEN) + " free");
      logInfo("SYS", "Audio queue: " + String(uxQueueSpacesAvailable(audioQueue)) + "/" + String(CAPTURE_QUEUE_LEN) + " free");
      logInfo("SYS", "VQA queue: " + String(uxQueueSpacesAvailable(vqaQueue)) + "/" + String(CAPTURE_QUEUE_LEN) + " free");
      logMemory("SYS");
    }
    else if (cmd == "DEBUG") {
      currentLogLevel = (currentLogLevel == LOG_DEBUG) ? LOG_INFO : LOG_DEBUG;
      logInfo("SYS", "Debug logging " + String(currentLogLevel == LOG_DEBUG ? "enabled" : "disabled"));
    }
    else if (cmd.length() > 0) {
      logWarn("CMD", "Unknown serial command: '" + cmd + "'");
      logInfo("CMD", "Available commands: IMAGE, AUDIO, VQA, STATUS, DEBUG");
    }
  }
  delay(10);
}
