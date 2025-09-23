#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_camera.h"
#include "ESP_I2S.h"
#include "esp_adc_cal.h"
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define AUDIO_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define IMAGE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define TEMP_CHARACTERISTIC_UUID "00002A6E-0000-1000-8000-00805F9B34FB"

#define CAPTURE_DEBOUNCE_MS 500
#define SEND_DELAY_BETWEEN_CHUNKS_MS 15

#define VQA_STREAM_CHUNK_DURATION_MS 150  // 500ms chunks for VQA streaming
#define VQA_STREAM_BUFFER_COUNT 4         // Number of buffers for smooth streaming

#define BUTTON_PIN D10



// ============================================================================
// Button Globals
// ============================================================================
const int led_pin = LED_BUILTIN; // On-board LED

bool ledState = false;     // indicator LED state (mirrors vqa running)
bool lastButtonState = HIGH;    // Previous button state (HIGH = released, LOW = pressed)
unsigned long lastButtonChange = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 50; // debounce delay



// ============================================================================
// Temperature Globals
// ============================================================================
float tempThreshold = 55.0; // °C limit before shutdown
TaskHandle_t tempTaskHandle;
bool tempMonitoringEnabled = false; // Enable/disable temperature monitoring



// ============================================================================
// BLE Globals
// ============================================================================
bool deviceConnected = false;
bool systemInitialized = false;
BLECharacteristic* pAudioCharacteristic;
BLECharacteristic* pImageCharacteristic;
BLECharacteristic* pTempCharacteristic;
int negotiatedChunkSize = 23;
I2SClass i2s;



// ============================================================================
// VQA Globals
// ============================================================================
struct VQAState {
  bool isRunning = false;
  bool stopRequested = false;
  bool imageTransmissionComplete = false;
  bool audioRecordingInProgress = false;
  bool audioRecordingComplete = false;
  unsigned long audioRecordingStartTime = 0;
  TaskHandle_t vqaTaskHandle = nullptr;
  
  // Streaming audio state
  bool audioStreamingActive = false;
  size_t totalAudioStreamed = 0;
  unsigned long streamStartTime = 0;
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
LogLevel currentLogLevel = LOG_DEBUG;

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

// Enhanced progress logger with transfer rate
void logProgressRate(const String &component, size_t current, size_t total, unsigned long startTime) {
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

// Streaming audio logger
void logStreamingProgress(const String &component, size_t chunkSize, size_t totalStreamed, unsigned long startTime, int chunkNumber) {
  if (currentLogLevel <= LOG_INFO) {
    unsigned long elapsed = millis() - startTime;
    float rate = 0;
    if (elapsed > 0) {
      rate = (totalStreamed / 1024.0) / (elapsed / 1000.0); // KB/s
    }
    
    log(LOG_INFO, component, "Chunk #" + String(chunkNumber) + " (" + String(chunkSize) + " bytes) - " + 
        "Total: " + String(totalStreamed/1024.0, 1) + " KB @ " + String(rate, 1) + " KB/s");
  }
}



// ============================================================================
// BLE Server Callbacks
// ============================================================================

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println();
    logInfo("BLE", "!! Client connected !!");
    deviceConnected = true;
    logMemory("BLE");
    
    // Initialize system components when connected
    if (!systemInitialized) {
      initializeSystem();
    }
  }
  void onDisconnect(BLEServer* pServer) override {
    Serial.println();
    logWarn("BLE", "!! Client disconnected !!");
    deviceConnected = false;
    
    // Clean up system components when disconnected
    if (systemInitialized) {
      cleanupSystem();
    }
    
    BLEDevice::startAdvertising();
    logInfo("BLE", "Advertising restarted");
  }
  void onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
    int mtu = param->mtu.mtu;
    negotiatedChunkSize = max(20, mtu - 3); // 3 bytes for ATT header
    logInfo("BLE", "MTU changed: " + String(mtu) + " bytes, "
                   "chunk size set to: " + String(negotiatedChunkSize) + " bytes");
  }
};



// ============================================================================
// System Initialization and Cleanup
// ============================================================================

// Initialize and setup all components
void initializeSystem() {
  Serial.println();
  logInfo("SYS", "========================== Initializing system components ==========================");
  
  // Initialize Camera
  initCamera();

  // Initialize Microphone
  initMicrophone();

  systemInitialized = true;
  logInfo("SYS", "========================== System initialization complete ==========================");
  Serial.println();
  logMemory("SYS");
}



// Cleanup and free all resources
void cleanupSystem() {
  Serial.println();
  logInfo("SYS", "========================== Cleaning up system components ==========================");
  
  // Clean up VQA background task if running
  if (vqaState.vqaTaskHandle) {
    vqaState.stopRequested = true;
    vTaskDelay(pdMS_TO_TICKS(100)); // Give time for graceful stop
    if (vqaState.vqaTaskHandle) {
      vTaskDelete(vqaState.vqaTaskHandle);
      vqaState.vqaTaskHandle = nullptr;
    }
    vqaState.isRunning = false;
    logInfo("SYS", "VQA streaming task deleted");
  }
  
  // Reset VQA state
  vqaState.isRunning = false;
  vqaState.stopRequested = false;
  vqaState.imageTransmissionComplete = false;
  vqaState.audioRecordingInProgress = false;
  vqaState.audioRecordingComplete = false;
  vqaState.audioStreamingActive = false;
  vqaState.totalAudioStreamed = 0;
  vqaState.audioRecordingStartTime = 0;
  
  // Deinitialize camera
  esp_camera_deinit();
  logInfo("SYS", "Camera deinitialized");
  
  // Stop I2S (microphone)
  i2s.end();
  logInfo("SYS", "Microphone deinitialized");
  
  systemInitialized = false;
  logInfo("SYS", "========================== System cleanup complete ==========================");
  Serial.println();
  logMemory("SYS");
}



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
  // FRAMESIZE_QSXGA - 2592x1944
  config.frame_size = FRAMESIZE_HD; 
  // ------------------------------------------

  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = psramFound() ? 4 : 8;
  config.fb_count = psramFound() ? 2 : 1;

  logDebug("CAM", "PSRAM found: " + String(psramFound() ? "Yes" : "No"));

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
    Serial.println();
    logInfo("SYS", "========================== Initializing Bluetooth Connection ==========================");

    logInfo("BLE", "Initializing Bluetooth LE...");
    
    BLEDevice::init("XIAO_ESP32S3");
    BLEDevice::setMTU(223);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Audio Characteristic (for streaming audio data)
    pAudioCharacteristic = pService->createCharacteristic(
        AUDIO_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pAudioCharacteristic->addDescriptor(new BLE2902());

    // Image Characteristic (for sending image data)
    pImageCharacteristic = pService->createCharacteristic(
        IMAGE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pImageCharacteristic->addDescriptor(new BLE2902());

    // Temperature Characteristic
    pTempCharacteristic = pService->createCharacteristic(
        TEMP_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTempCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    logInfo("BLE", "Ready and advertising as 'XIAO_ESP32S3'");
    logDebug("BLE", "Service UUID: " + String(SERVICE_UUID));
    logDebug("BLE", "Audio UUID: " + String(AUDIO_CHARACTERISTIC_UUID));
    logDebug("BLE", "Image UUID: " + String(IMAGE_CHARACTERISTIC_UUID));
    logMemory("BLE");

    logInfo("SYS", "========================== BLE initialization complete - waiting for connections ==========================");
    Serial.println();
}



// ============================================================================
// FreeRTOS Tasks
// ============================================================================

// VQA Streaming Task - Parallel Audio and Image Transmission
void vqaStreamTask(void *param) {
  logInfo("VQA-STREAM", "VQA streaming task started (parallel audio and image)");
  
  // Ensure LED reflects streaming state
  digitalWrite(led_pin, HIGH);
  ledState = true;

  // Initialize VQA streaming state
  vqaState.isRunning = true;
  vqaState.stopRequested = false;
  vqaState.audioRecordingInProgress = false;
  vqaState.audioRecordingComplete = false;
  vqaState.imageTransmissionComplete = false;
  vqaState.audioStreamingActive = false;
  vqaState.totalAudioStreamed = 0;
  
  // Calculate streaming parameters for continuous audio
  const int sampleRate = 8000;
  const int bytesPerSample = 2; // 16-bit
  const int bytesPerSecond = sampleRate * bytesPerSample;
  const int chunkDuration = VQA_STREAM_CHUNK_DURATION_MS;
  const int chunkSizeBytes = (bytesPerSecond * chunkDuration) / 1000;
  
  logDebug("VQA-STREAM", "Stream config: " + String(chunkSizeBytes) + " bytes/chunk, " + 
           String(chunkDuration) + "ms/chunk, parallel transmission");

  // No need for VQA_START header - characteristics identify data type
  
  // Initialize streaming state
  vqaState.audioStreamingActive = true;
  vqaState.streamStartTime = millis();
  int chunkNumber = 0;
  bool streamingSuccess = true;

  // ============================================================================
  // STEP 1: Start Image Capture Task (runs in parallel)
  // ============================================================================
  TaskHandle_t imageCaptureTaskHandle = nullptr;
  BaseType_t imageTaskResult = xTaskCreatePinnedToCore(
    imageCaptureTask, 
    "ImageCaptureTask", 
    8192, 
    nullptr, 
    1, // Same priority as VQA task
    &imageCaptureTaskHandle, 
    0  // Core 0 (different from main VQA task)
  );
  
  if (imageTaskResult != pdPASS) {
    logError("VQA-STREAM", "Failed to start image capture task");
    streamingSuccess = false;
  } else {
    logInfo("VQA-STREAM", "Image capture task started on Core 0");
  }

  // ============================================================================
  // STEP 2: Stream Audio Continuously (parallel with image)
  // ============================================================================
  logInfo("VQA-STREAM", "Starting continuous audio streaming...");

  while (!vqaState.stopRequested && streamingSuccess && deviceConnected) {
    chunkNumber++;
    
    // Allocate buffer for this chunk
    uint8_t* chunkBuffer = new uint8_t[chunkSizeBytes];
    if (!chunkBuffer) {
      logError("VQA-STREAM", "Failed to allocate chunk buffer");
      streamingSuccess = false;
      break;
    }

    // Record audio chunk
    size_t bytesRead = 0;
    unsigned long recordStart = millis();
    
    while (bytesRead < chunkSizeBytes && (millis() - recordStart) < (chunkDuration + 100)) {
      size_t bytesToRead = min((size_t)negotiatedChunkSize, chunkSizeBytes - bytesRead);
      size_t actuallyRead = i2s.readBytes((char*)(chunkBuffer + bytesRead), bytesToRead);
      bytesRead += actuallyRead;
      
      if (actuallyRead == 0) {
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay if no data available
      }
      
      // Check for stop request during recording
      if (vqaState.stopRequested) {
        break;
      }
    }
    
    if (bytesRead < chunkSizeBytes && !vqaState.stopRequested) {
      logWarn("VQA-STREAM", "Chunk " + String(chunkNumber) + " incomplete: " + 
              String(bytesRead) + "/" + String(chunkSizeBytes) + " bytes");
    }

    // Send audio data directly via audio characteristic (no headers needed)
    if (bytesRead > 0 && !vqaState.stopRequested) {
      for (size_t i = 0; i < bytesRead && !vqaState.stopRequested; i += negotiatedChunkSize) {
        size_t packetSize = min((size_t)negotiatedChunkSize, bytesRead - i);
        pAudioCharacteristic->setValue(chunkBuffer + i, packetSize);
        pAudioCharacteristic->notify();
        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
        
        // Check if client disconnected during streaming
        if (!deviceConnected) {
          logWarn("VQA-STREAM", "Client disconnected during streaming");
          streamingSuccess = false;
          break;
        }
      }

      vqaState.totalAudioStreamed += bytesRead;
      logStreamingProgress("VQA-AUDIO", bytesRead, vqaState.totalAudioStreamed, vqaState.streamStartTime, chunkNumber);
    }

    // Clean up chunk buffer
    delete[] chunkBuffer;

    if (!streamingSuccess) break;
  }

  // Finalize audio streaming
  vqaState.audioStreamingActive = false;
  vqaState.audioRecordingComplete = true;

  unsigned long audioTime = millis() - vqaState.streamStartTime;
  float avgRate = (vqaState.totalAudioStreamed / 1024.0) / (audioTime / 1000.0);
  
  logInfo("VQA-STREAM", "Audio streaming complete: " + String(vqaState.totalAudioStreamed) + " bytes (" + 
          String(vqaState.totalAudioStreamed/1024.0, 1) + " KB) in " + String(audioTime) + 
          "ms (" + String(avgRate, 1) + " KB/s avg)");

  // ============================================================================
  // STEP 3: Wait for Image Task to Complete
  // ============================================================================
  if (imageCaptureTaskHandle && streamingSuccess) {
    logInfo("VQA-STREAM", "Waiting for image capture task to complete...");
    // Wait for image task to finish (with timeout)
    unsigned long waitStart = millis();
    while (eTaskGetState(imageCaptureTaskHandle) != eDeleted && (millis() - waitStart) < 10000) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (eTaskGetState(imageCaptureTaskHandle) != eDeleted) {
      logWarn("VQA-STREAM", "Image task did not complete, force deleting");
      vTaskDelete(imageCaptureTaskHandle);
    }
  }

  // ============================================================================
  // STEP 4: Finalize VQA Operation
  // ============================================================================
  
  // Determine completion status
  String stopReason;
  if (vqaState.stopRequested) {
    stopReason = "User requested stop";
  } else if (!deviceConnected) {
    stopReason = "Client disconnected";
  } else {
    stopReason = "Stream error";
  }

  // Send completion notification via debug log only
  if (streamingSuccess || vqaState.stopRequested) {
    logDebug("VQA-STREAM", "VQA operation completed");

    unsigned long totalTime = millis() - vqaState.streamStartTime;
    
    logInfo("VQA-STREAM", "VQA operation completed: " + stopReason);
    logInfo("VQA-STREAM", "Total duration: " + String(totalTime) + "ms");
    logInfo("VQA-STREAM", "Audio streamed: " + String(vqaState.totalAudioStreamed/1024.0, 1) + " KB");
    logInfo("VQA-STREAM", "Image captured: " + String(vqaState.imageTransmissionComplete ? "Yes" : "No"));
  } else {
    logError("VQA-STREAM", "VQA streaming failed: " + stopReason);
  }

  // Reset VQA state
  vqaState.isRunning = false;
  vqaState.stopRequested = false;
  vqaState.vqaTaskHandle = nullptr;

  // Turn off indicator LED
  digitalWrite(led_pin, LOW);
  ledState = false;
  
  // Task auto-cleanup
  vTaskDelete(NULL);
}

// Image Capture Task - Runs in parallel with audio streaming
void imageCaptureTask(void *param) {
  logInfo("VQA-IMAGE", "Image capture task started");
  
  // Small delay to let audio streaming start first
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Take picture
  camera_fb_t *fb = esp_camera_fb_get();
  
  // Retry once if first capture fails
  if (!fb) {
    logWarn("VQA-IMAGE", "First capture failed, retrying...");
    vTaskDelay(pdMS_TO_TICKS(50));
    fb = esp_camera_fb_get();
    if (!fb) {
      logError("VQA-IMAGE", "Camera capture failed after retry");
      vTaskDelete(NULL);
      return;
    }
  }

  if (fb && deviceConnected) {
    size_t imageSize = fb->len;
    uint8_t *imageBuffer = fb->buf;
    logInfo("VQA-IMAGE", "Captured " + String(imageSize) + " bytes (" + 
            String(imageSize/1024) + " KB)");

    // Send image data via BLE using image characteristic (no headers needed)
    logInfo("VQA-IMAGE", "Starting image transmission...");
    
    // Send image chunks
    size_t sentBytes = 0;
    unsigned long transferStart = millis();
    
    for (size_t i = 0; i < imageSize && deviceConnected; i += negotiatedChunkSize) {
      int len = (i + negotiatedChunkSize > imageSize) ? (imageSize - i) : negotiatedChunkSize;
      pImageCharacteristic->setValue(imageBuffer + i, len);
      pImageCharacteristic->notify();
      sentBytes += len;
      vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_BETWEEN_CHUNKS_MS));
        
      logProgressRate("VQA-IMAGE", sentBytes, imageSize, transferStart);
      
      // Only check for client disconnection, not stop request
      if (!deviceConnected) {
        logWarn("VQA-IMAGE", "Client disconnected during image transfer");
        break;
      }
    }

    if (deviceConnected && sentBytes == imageSize) {
      unsigned long imageTransferTime = millis() - transferStart;
      float imageTransferRate = (imageSize / 1024.0) / (imageTransferTime / 1000.0);
      
      logInfo("VQA-IMAGE", "Image transfer complete in " + String(imageTransferTime) + 
              "ms (" + String(imageTransferRate, 1) + " KB/s)");
      
      vqaState.imageTransmissionComplete = true;
    }

    // Release camera frame buffer
    esp_camera_fb_return(fb);
  }
  
  logInfo("VQA-IMAGE", "Image capture task completed");
  
  // Task auto-cleanup
  vTaskDelete(NULL);
}

// Temperature Task
void temperatureTask(void *param) {
  while (true) {
    float currentTemp = temperatureRead();

    // Safety shutdown
    if (currentTemp > tempThreshold) {
      logError("TEMP", "Overheat detected! Turning off Device");
      esp_deep_sleep_start();
    }

    // Always send to Flutter when connected
    if (deviceConnected) {
      char buffer[16]; // Enough for header + float
      snprintf(buffer, sizeof(buffer), "T:%.2f", currentTemp);
      pTempCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
      pTempCharacteristic->notify();
    }

    if (tempMonitoringEnabled) {
      logDebug("TEMP", "Current temperature: " + String(currentTemp, 1) + " °C");
    }

    vTaskDelay(pdMS_TO_TICKS(10000)); // every 1 sec
  }
}



// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(9600); // Match the simple button example
    delay(1000);
    
    // Welcome
    printWelcomeArt();
    logInfo("SYS", "Compile time: " + String(__DATE__) + " " + String(__TIME__));
    logMemory("SYS");

    // Setup button and LED pins
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Button input with internal pullup

    // Initialize BLE only
    initBLE();

    // Create task to monitor temperature (always active for overheat protection)
    xTaskCreatePinnedToCore(temperatureTask, "temperatureTask", 4096, NULL, 1, &tempTaskHandle, 1);
    logInfo("SYS", "Temperature monitoring task created (overheat protection always active)");

    logInfo("SYS", "System components will be initialized when a client connects");
    logMemory("SYS");
}



// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  // -----------------------------
  // Button handling (press = start VQA, release = stop VQA)
  // -----------------------------
  int buttonState = digitalRead(BUTTON_PIN);
  
  // Debouncing - only process if enough time has passed since last change
  if ((millis() - lastButtonChange) > BUTTON_DEBOUNCE_MS) {
    // Check for button state change
    if (buttonState != lastButtonState) {
      lastButtonChange = millis();
      
      // Button pressed (HIGH to LOW transition)
      if (buttonState == LOW && lastButtonState == HIGH) {
        if (!deviceConnected) {
          logWarn("BUTTON", "Button press ignored - BLE not connected");
        } else if (!systemInitialized) {
          logWarn("BUTTON", "Button press ignored - system not initialized");
        } else if (!vqaState.isRunning) {
          // Start VQA streaming
          vqaState.stopRequested = false;
          BaseType_t result = xTaskCreatePinnedToCore(
            vqaStreamTask, 
            "VQAStreamTask", 
            20480, 
            nullptr, 
            1, // Normal priority
            &vqaState.vqaTaskHandle, 
            1  // Core 1
          );
          if (result == pdPASS) {
            logInfo("BUTTON", "VQA streaming started - button pressed");
            digitalWrite(led_pin, HIGH);
            ledState = true;
          } else {
            logError("BUTTON", "Failed to start VQA streaming task");
          }
        } else {
          logWarn("BUTTON", "VQA streaming already active");
        }
      }
      
      // Button released (LOW to HIGH transition)
      else if (buttonState == HIGH && lastButtonState == LOW) {
        if (vqaState.isRunning) {
          vqaState.stopRequested = true;
          logInfo("BUTTON", "VQA streaming stop requested - button released");
        } else {
          logInfo("BUTTON", "Button released - no active VQA to stop");
        }
      }
      
      lastButtonState = buttonState;
    }
  }

  delay(10);
}



// ============================================================================
// Welcome Art
// ============================================================================
void printWelcomeArt() {
  static const char* art[] = {
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⠿⠿⣷⣦⣀⠀⢀⣀⣀⣤⣤⣤⣤⣤⣄⣀⡀⢀⣤⣶⠿⠿⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⡟⢸⠀⠀⠉⠻⠿⠛⠛⠋⠉⠉⠉⠉⠉⠙⠛⠻⠿⠋⠁⠀⠀⢻⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⡇⠘⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡀⠀⢸⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⠛⠀⠀⠐⢠⣦⣦⣦⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣄⠀⠀⠙⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⡟⠀⠀⠀⠀⠀⠸⣿⣿⣿⣿⣿⣿⣿⣿⠀⠸⣿⣿⣿⣿⣿⣿⣿⡇⡄⠀⠀⠈⢿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠇⠀⠀⠀⠀⠀⠀⢻⣿⣿⣿⣿⣿⡿⠁⣼⣄⠙⠿⣿⣿⣿⡿⠟⠀⠀⠀⠀⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⡆⠀⠀⠀⠀⠀⣴⡿⠛⢿⡏⠉⠈⠰⡿⠛⠻⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣿⡀⢀⠀⠀⣸⣿⠁⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢙⣿⣶⡄⡾⠟⠃⠀⠀⣾⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⠿⣯⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⠃⠀⠀⠀⠀⠀⠀⣴⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⠀⠀⠀⠘⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀",
    "⢀⡀⡀⣀⣀⣀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀⠊⠀⠀⠀⠀⠀⢀⣀⠀⠀⠀⠀⣤⣾⠿⠛⠃⠀⠀⠀⣸⣷⠀⠀⠀⠀⣀⣀⡀⡀⠀",
    "⠺⠿⠛⠛⠛⠛⠿⠿⠿⠿⠿⠿⠿⠿⠿⠷⣶⢶⡶⡶⠶⠿⠿⠿⠿⠟⠛⠛⠻⠿⠿⠿⢿⣿⣀⣠⣤⣤⣤⣤⣶⠿⠛⠛⠛⠛⠛⠛⠛⠛⠻⠃",
    "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  };

  for (const char* line : art) {
    Serial.println(line);
    delay(200);  // shorter, faster but still gives dramatic effect
  }
  
  logInfo("SYS", "Welcome, Cj");
}
