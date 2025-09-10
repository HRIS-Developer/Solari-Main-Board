/*
 * ESP32 Camera BLE Server
 * Captures images and transmits them via Bluetooth Low Energy
 * Optimized for XIAO ESP32S3 with camera module
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_camera.h"
#include "camera_pins.h"

// ============================================================================
// Configuration Constants
// ============================================================================
// BLE Service and Characteristic UUIDs
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Camera and transmission settings
#define CAPTURE_QUEUE_LENGTH     1
#define CAPTURE_DEBOUNCE_MS      500
#define CHUNK_DELAY_MS           10
#define MIN_MTU_SIZE             20
#define MTU_OVERHEAD             6
#define PROGRESS_LOG_INTERVAL    500
#define TASK_STACK_SIZE          16384
#define TASK_PRIORITY            1
#define TASK_CORE                1

// Camera configuration
#define CAMERA_XCLK_FREQ_HZ      20000000
#define CAMERA_JPEG_QUALITY_DEFAULT  12
#define CAMERA_JPEG_QUALITY_PSRAM    10
#define CAMERA_FB_COUNT_DEFAULT  1
#define CAMERA_FB_COUNT_PSRAM    2

// Serial communication
#define SERIAL_BAUD_RATE         115200
#define SERIAL_INIT_DELAY_MS     1000
#define LOOP_DELAY_MS            10

// ============================================================================
// Global Variables and State Management
// ============================================================================
// BLE connection state
volatile bool isDeviceConnected = false;
BLECharacteristic* pCameraCharacteristic = nullptr;
volatile int negotiatedChunkSize = MIN_MTU_SIZE + 3; // Default safe size

// Task management
QueueHandle_t captureRequestQueue = nullptr;
TaskHandle_t imageSenderTaskHandle = nullptr;

// Frame tracking
static uint32_t frameCounter = 0;

// ============================================================================
// Utility Functions
// ============================================================================
/**
 * Log message with timestamp to Serial
 * @param message The message to log
 */
void logWithTimestamp(const String &message) {
  Serial.printf("[%lu ms] %s\n", millis(), message.c_str());
}

/**
 * Log error message with timestamp
 * @param message The error message to log
 */
void logError(const String &message) {
  Serial.printf("[%lu ms] ERROR: %s\n", millis(), message.c_str());
}

/**
 * Handle image capture request from any source
 * @param source The source of the request (BLE/Serial)
 */
void handleCaptureRequest(const String& source) {
  uint8_t captureToken = 1;
  
  if (captureRequestQueue && 
      xQueueSend(captureRequestQueue, &captureToken, 0) == pdTRUE) {
    logWithTimestamp(source + " capture request queued successfully");
  } else {
    logWithTimestamp(source + " capture request ignored (queue full)");
  }
}

// ============================================================================
// BLE Server Event Handlers
// ============================================================================
/**
 * Handles BLE server connection events
 */
class BLEServerEventHandler : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    logWithTimestamp("BLE device connected successfully");
    isDeviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    logWithTimestamp("BLE device disconnected");
    isDeviceConnected = false;
    
    // Restart advertising for new connections
    BLEDevice::startAdvertising();
    logWithTimestamp("BLE advertising restarted");
  }
};

/**
 * Handles BLE characteristic write events and commands
 */
class BLECharacteristicEventHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String receivedValue = String(pCharacteristic->getValue().c_str());
    receivedValue.trim();
    receivedValue.toUpperCase();

    logWithTimestamp("BLE command received: " + receivedValue);

    if (receivedValue.startsWith("MTU:")) {
      handleMTUNegotiation(receivedValue);
    }
    else if (receivedValue == "CAPTURE") {
      handleCaptureRequest("BLE");
    }
    else {
      logWithTimestamp("Unknown BLE command: " + receivedValue);
    }
  }

private:
  /**
   * Handle MTU size negotiation
   * @param mtuCommand The MTU command string (format: "MTU:size")
   */
  void handleMTUNegotiation(const String& mtuCommand) {
    int mtuSize = mtuCommand.substring(4).toInt();
    negotiatedChunkSize = max(MIN_MTU_SIZE, mtuSize - MTU_OVERHEAD);
    
    logWithTimestamp("MTU negotiated: " + String(mtuSize) + 
                    ", chunk size: " + String(negotiatedChunkSize));
  }
};

// ============================================================================
// Camera Configuration and Initialization
// ============================================================================
/**
 * Configure camera with optimal settings based on available memory
 * @return camera_config_t The configured camera settings
 */
camera_config_t createCameraConfig() {
  camera_config_t config;
  
  // LED and timing configuration
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  
  // Data pins (D0-D7)
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  
  // Control pins
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  // Clock and format settings
  config.xclk_freq_hz = CAMERA_XCLK_FREQ_HZ;
  config.frame_size = FRAMESIZE_HD;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = CAMERA_JPEG_QUALITY_DEFAULT;
  config.fb_count = CAMERA_FB_COUNT_DEFAULT;

  return config;
}

/**
 * Initialize camera with PSRAM optimization if available
 */
bool initializeCamera() {
  logWithTimestamp("Initializing camera module...");
  
  camera_config_t config = createCameraConfig();

  // Optimize settings if PSRAM is available
  if (psramFound()) {
    config.jpeg_quality = CAMERA_JPEG_QUALITY_PSRAM;
    config.fb_count = CAMERA_FB_COUNT_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    logWithTimestamp("PSRAM detected - using optimized settings");
  } else {
    logWithTimestamp("PSRAM not found - using standard settings");
  }

  esp_err_t cameraInitResult = esp_camera_init(&config);
  if (cameraInitResult != ESP_OK) {
    logError("Camera initialization failed with error code: " + String(cameraInitResult));
    return false;
  }
  
  logWithTimestamp("Camera initialized successfully");
  return true;
}

// ============================================================================
// Image Transmission Functions
// ============================================================================
/**
 * Send image header with metadata
 * @param imageSize Total size of the image in bytes
 */
void sendImageHeader(size_t imageSize) {
  String header = "IMG_START:" + String(imageSize);
  pCameraCharacteristic->setValue((uint8_t*)header.c_str(), header.length());
  pCameraCharacteristic->notify();
  vTaskDelay(pdMS_TO_TICKS(20));
  logWithTimestamp("Image header sent, size: " + String(imageSize) + " bytes");
}

/**
 * Send image footer to mark transmission end
 */
void sendImageFooter() {
  String footer = "IMG_END";
  pCameraCharacteristic->setValue((uint8_t*)footer.c_str(), footer.length());
  pCameraCharacteristic->notify();
  logWithTimestamp("Image transmission completed");
}

/**
 * Send image data in chunks via BLE
 * @param imageBuffer Pointer to image data
 * @param totalSize Total size of the image
 * @return bool Success status of transmission
 */
bool transmitImageData(uint8_t* imageBuffer, size_t totalSize) {
  if (!imageBuffer || totalSize == 0) {
    logError("Invalid image data for transmission");
    return false;
  }

  logWithTimestamp("Starting image data transmission...");
  
  size_t bytesTransmitted = 0;
  const unsigned long transmissionStartTime = millis();

  // Send image data in chunks
  for (size_t offset = 0; offset < totalSize; offset += negotiatedChunkSize) {
    // Calculate chunk size for this iteration
    size_t currentChunkSize = min((size_t)negotiatedChunkSize, totalSize - offset);
    
    // Send chunk
    pCameraCharacteristic->setValue(imageBuffer + offset, currentChunkSize);
    pCameraCharacteristic->notify();
    bytesTransmitted += currentChunkSize;

    // Small delay to prevent BLE buffer overflow
    vTaskDelay(pdMS_TO_TICKS(CHUNK_DELAY_MS));

    // Log progress periodically
    if (bytesTransmitted % PROGRESS_LOG_INTERVAL <= negotiatedChunkSize) {
      logWithTimestamp("Progress: " + String(bytesTransmitted) + "/" + 
                      String(totalSize) + " bytes transmitted");
    }
  }

  const unsigned long transmissionDuration = millis() - transmissionStartTime;
  logWithTimestamp("Data transmission completed in " + String(transmissionDuration) + " ms");
  
  return true;
}
// ============================================================================
// Image Capture and Transmission Task
// ============================================================================
/**
 * Background task that handles image capture and BLE transmission
 * Processes capture requests from the queue and manages the entire workflow
 */
void imageSenderTask(void *taskParameters) {
  uint8_t captureRequest;
  unsigned long lastCaptureTime = 0;

  logWithTimestamp("Image sender task started successfully");

  // Main task loop
  while (true) {
    // Wait for capture request from queue (blocking)
    if (xQueueReceive(captureRequestQueue, &captureRequest, portMAX_DELAY) == pdTRUE) {
      
      // Implement debouncing to prevent rapid successive captures
      const unsigned long currentTime = millis();
      if (currentTime - lastCaptureTime < CAPTURE_DEBOUNCE_MS) {
        logWithTimestamp("Capture request debounced (too soon after previous)");
        continue;
      }
      lastCaptureTime = currentTime;

      // Check if BLE device is connected
      if (!isDeviceConnected) {
        logWithTimestamp("No BLE device connected - skipping capture");
        continue;
      }

      // Capture image from camera
      camera_fb_t* frameBuffer = esp_camera_fb_get();
      if (!frameBuffer) {
        logError("Failed to capture image from camera");
        continue;
      }

      const size_t imageSize = frameBuffer->len;
      uint8_t* imageData = frameBuffer->buf;
      frameCounter++;
      
      logWithTimestamp("Frame #" + String(frameCounter) + " captured successfully, size: " + 
                      String(imageSize) + " bytes");

      const unsigned long transmissionStartTime = millis();

      // Send complete image via BLE (header + data + footer)
      sendImageHeader(imageSize);
      
      if (transmitImageData(imageData, imageSize)) {
        sendImageFooter();
      } else {
        logError("Image transmission failed");
      }

      // Release camera frame buffer
      esp_camera_fb_return(frameBuffer);
      logWithTimestamp("Frame buffer released");

      const unsigned long totalTransmissionTime = millis() - transmissionStartTime;
      logWithTimestamp("Complete transmission time: " + String(totalTransmissionTime) + " ms");

      // Clear any additional queued requests to prevent backlog
      uint8_t extraRequest;
      while (xQueueReceive(captureRequestQueue, &extraRequest, 0) == pdTRUE) {
        logWithTimestamp("Cleared extra queued capture request");
      }
    }
  }
  
  // This line should never be reached, but included for completeness
  vTaskDelete(NULL);
}

// ============================================================================
// BLE Service Initialization
// ============================================================================
/**
 * Initialize and configure BLE server with camera service
 * @return bool Success status of BLE initialization
 */
bool initializeBLEService() {
  logWithTimestamp("Initializing BLE service...");

  // Initialize BLE device with descriptive name
  BLEDevice::init("XIAO_ESP32S3_Camera");
  
  // Create BLE server and set callbacks
  BLEServer* pBLEServer = BLEDevice::createServer();
  pBLEServer->setCallbacks(new BLEServerEventHandler());

  // Create camera service
  BLEService* pCameraService = pBLEServer->createService(BLE_SERVICE_UUID);
  
  // Create characteristic for camera control and data
  pCameraCharacteristic = pCameraService->createCharacteristic(
    BLE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );

  // Set initial value and callbacks
  pCameraCharacteristic->setValue("ESP32 Camera Ready");
  pCameraCharacteristic->setCallbacks(new BLECharacteristicEventHandler());

  // Start the service
  pCameraService->start();

  // Configure and start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  logWithTimestamp("BLE service initialized and advertising started");
  return true;
}

/**
 * Initialize task queue and background processing
 * @return bool Success status of task initialization
 */
bool initializeTaskSystem() {
  logWithTimestamp("Setting up task system...");

  // Create capture request queue
  captureRequestQueue = xQueueCreate(CAPTURE_QUEUE_LENGTH, sizeof(uint8_t));
  if (captureRequestQueue == nullptr) {
    logError("Failed to create capture request queue");
    return false;
  }

  // Create background task for image processing
  BaseType_t taskCreationResult = xTaskCreatePinnedToCore(
    imageSenderTask,              // Task function
    "ImageSenderTask",            // Task name
    TASK_STACK_SIZE,             // Stack size
    nullptr,                     // Task parameters
    TASK_PRIORITY,               // Task priority
    &imageSenderTaskHandle,      // Task handle
    TASK_CORE                    // CPU core
  );

  if (taskCreationResult != pdPASS) {
    logError("Failed to create image sender task");
    return false;
  }

  logWithTimestamp("Task system initialized successfully");
  return true;
}

// ============================================================================
// Serial Command Processing
// ============================================================================
/**
 * Process commands received via Serial interface
 * @param command The command string to process
 */
void processSerialCommand(const String& command) {
  String processedCommand = command;
  processedCommand.trim();
  processedCommand.toUpperCase();

  if (processedCommand == "CAPTURE") {
    if (!isDeviceConnected) {
      logWithTimestamp("Serial capture request ignored - no BLE device connected");
      return;
    }

    uint8_t captureToken = 1;
    if (captureRequestQueue && 
        xQueueSend(captureRequestQueue, &captureToken, 0) == pdTRUE) {
      logWithTimestamp("Serial capture request queued successfully");
    } else {
      logWithTimestamp("Serial capture request ignored (queue full)");
    }
  } 
  else if (processedCommand == "STATUS") {
    logWithTimestamp("System Status:");
    logWithTimestamp("  BLE Connected: " + String(isDeviceConnected ? "Yes" : "No"));
    logWithTimestamp("  Frames Captured: " + String(frameCounter));
    logWithTimestamp("  Chunk Size: " + String(negotiatedChunkSize));
  }
  else if (processedCommand == "RESET") {
    logWithTimestamp("System reset requested");
    ESP.restart();
  }
  else {
    logWithTimestamp("Unknown serial command: " + processedCommand);
    logWithTimestamp("Available commands: CAPTURE, STATUS, RESET");
  }
}

// ============================================================================
// Arduino Setup Function
// ============================================================================
void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  delay(SERIAL_INIT_DELAY_MS);
  
  logWithTimestamp("=== ESP32S3 BLE Camera Server Starting ===");
  logWithTimestamp("Build: " + String(__DATE__) + " " + String(__TIME__));

  // Initialize camera module
  if (!initializeCamera()) {
    logError("Camera initialization failed - system halted");
    while (true) {
      delay(1000);
    }
  }

  // Initialize BLE service
  if (!initializeBLEService()) {
    logError("BLE service initialization failed - system halted");
    while (true) {
      delay(1000);
    }
  }

  // Initialize task system
  if (!initializeTaskSystem()) {
    logError("Task system initialization failed - system halted");
    while (true) {
      delay(1000);
    }
  }

  logWithTimestamp("=== System initialization completed successfully ===");
  logWithTimestamp("Ready to receive capture commands via BLE or Serial");
}

// ============================================================================
// Arduino Main Loop
// ============================================================================
void loop() {
  // Check for serial commands when device is ready
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processSerialCommand(command);
  }

  // Small delay to prevent excessive CPU usage
  delay(LOOP_DELAY_MS);
}
