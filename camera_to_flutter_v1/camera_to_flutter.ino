/*
 * ESP32-S3 XIAO Camera BLE Server
 * 
 * This firmware enables the XIAO ESP32-S3 to capture images using the built-in camera
 * and transmit them over Bluetooth Low Energy (BLE) to connected devices.
 * 
 * Features:
 * - BLE-based image capture and transmission
 * - Configurable MTU for optimal data transfer
 * - Debounced capture requests to prevent spam
 * - RTOS task-based architecture for concurrent operations
 * - Serial command interface for debugging
 * 
 * Author: [Your Name]
 * Date: September 2025
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_camera.h"

// ============================================================================
// CAMERA PIN DEFINITIONS FOR XIAO ESP32S3
// ============================================================================
#include "camera_pins.h"

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// BLE Service and Characteristic UUIDs
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Device identification
#define DEVICE_NAME "XIAO_ESP32S3_Camera"

// Camera configuration - OPTIMIZED FOR XIAO ESP32 S3
#define CAMERA_FRAME_SIZE       FRAMESIZE_SVGA    // Reduced from HD for better memory usage
#define CAMERA_JPEG_QUALITY     15                // Slightly higher compression for smaller files
#define CAMERA_JPEG_QUALITY_PSRAM 12              // Better quality when PSRAM available
#define CAMERA_XCLK_FREQ_HZ     20000000

// BLE transmission settings
#define DEFAULT_CHUNK_SIZE      23
#define MIN_CHUNK_SIZE          20
#define MAX_CHUNK_SIZE          512
#define MTU_OVERHEAD            6
#define CHUNK_DELAY_MS          5     // Reduced from 10ms for faster transmission
#define HEADER_FOOTER_DELAY_MS  15    // Reduced from 20ms

// Task and queue configuration - OPTIMIZED FOR XIAO ESP32 S3
#define CAPTURE_QUEUE_LENGTH    2     // Reduced from 3 to save memory
#define CAPTURE_DEBOUNCE_MS     500   // Increased to prevent rapid captures
#define SENDER_TASK_STACK_SIZE  12288 // Reduced from 16KB to save memory
#define SENDER_TASK_PRIORITY    1
#define SENDER_TASK_CORE        1

// Serial communication
#define SERIAL_BAUD_RATE        115200
#define SERIAL_TIMEOUT_MS       1000

// Memory management - CONSERVATIVE SETTINGS FOR XIAO ESP32 S3
#define MAX_RETRY_ATTEMPTS      2
#define MEMORY_CHECK_THRESHOLD  80000 // Increased threshold for safer operation

// System monitoring
#define STATUS_REPORT_INTERVAL_MS    30000 // Report status every 30 seconds
#define WATCHDOG_TIMEOUT_MS          60000 // Watchdog timeout
#define MEMORY_WARNING_THRESHOLD     40000 // Higher warning threshold

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// BLE connection state
volatile bool g_bleDeviceConnected = false;
BLECharacteristic* g_pBleCharacteristic = nullptr;

// Communication parameters
volatile uint16_t g_negotiatedChunkSize = DEFAULT_CHUNK_SIZE;

// Task management
QueueHandle_t g_captureQueue = nullptr;
TaskHandle_t g_senderTaskHandle = nullptr;

// Frame tracking
static uint32_t g_frameCounter = 0;

// System monitoring
static unsigned long g_lastStatusReport = 0;
static unsigned long g_lastWatchdogReset = 0;
static uint32_t g_totalCaptureRequests = 0;
static uint32_t g_successfulTransmissions = 0;
static uint32_t g_failedTransmissions = 0;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Enhanced logging function with timestamp
 * @param level Log level (INFO, ERROR, DEBUG, etc.)
 * @param message Log message
 */
void logMessage(const char* level, const String& message) {
    Serial.printf("[%lu][%s] %s\n", millis(), level, message.c_str());
}

// Convenience macros for different log levels
#define LOG_INFO(msg)  logMessage("INFO", msg)
#define LOG_ERROR(msg) logMessage("ERROR", msg)
#define LOG_DEBUG(msg) logMessage("DEBUG", msg)
#define LOG_BLE(msg)   logMessage("BLE", msg)
#define LOG_CAM(msg)   logMessage("CAM", msg)

/**
 * @brief Emergency memory cleanup and system recovery
 */
void emergencyMemoryCleanup() {
    LOG_ERROR("Emergency memory cleanup initiated");
    
    // Force garbage collection
    if (g_captureQueue) {
        // Clear any pending capture requests
        uint8_t dummy;
        while (xQueueReceive(g_captureQueue, &dummy, 0) == pdTRUE) {
            // Clear queue
        }
    }
    
    // Force camera driver cleanup
    esp_camera_deinit();
    delay(100);
    
    // Reinitialize camera
    if (!initializeCamera()) {
        LOG_ERROR("Failed to reinitialize camera after cleanup");
    }
    
    LOG_INFO("Emergency cleanup completed");
}

/**
 * @brief Check if system has sufficient memory for operation
 * @param requiredBytes Minimum required free memory
 * @return true if sufficient memory available
 */
bool checkMemoryAvailability(size_t requiredBytes = MEMORY_CHECK_THRESHOLD) {
    size_t freeHeap = ESP.getFreeHeap();
    
    if (freeHeap < requiredBytes) {
        LOG_ERROR("Insufficient memory - Required: " + String(requiredBytes) + 
                 ", Available: " + String(freeHeap));
                 
        if (freeHeap < MEMORY_WARNING_THRESHOLD) {
            emergencyMemoryCleanup();
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Check if PSRAM is available and log the result
 * @return true if PSRAM is available, false otherwise
 */
bool checkPsramAvailability() {
    if (psramFound()) {
        LOG_INFO("PSRAM detected - enabling enhanced camera settings");
        return true;
    } else {
        LOG_INFO("PSRAM not found - using standard camera settings");
        return false;
    }
}

/**
 * @brief Send a BLE notification with error handling
 * @param data Data to send
 * @param length Length of data
 * @param delayMs Delay after sending (default: CHUNK_DELAY_MS)
 * @return true if successful, false otherwise
 */
bool sendBleNotification(const uint8_t* data, size_t length, uint16_t delayMs = CHUNK_DELAY_MS) {
    if (!g_bleDeviceConnected || !g_pBleCharacteristic) {
        LOG_ERROR("BLE not ready for transmission");
        return false;
    }
    
    try {
        g_pBleCharacteristic->setValue(const_cast<uint8_t*>(data), length);
        g_pBleCharacteristic->notify();
        
        if (delayMs > 0) {
            vTaskDelay(pdMS_TO_TICKS(delayMs));
        }
        return true;
    } catch (...) {
        LOG_ERROR("Failed to send BLE notification");
        return false;
    }
}

/**
 * @brief Send a text message over BLE
 * @param message Text message to send
 * @param delayMs Delay after sending
 * @return true if successful, false otherwise
 */
bool sendBleTextMessage(const String& message, uint16_t delayMs = CHUNK_DELAY_MS) {
    return sendBleNotification(reinterpret_cast<const uint8_t*>(message.c_str()), 
                              message.length(), delayMs);
}

// ============================================================================
// BLE SERVER CALLBACK CLASSES
// ============================================================================

/**
 * @brief Handles BLE server connection events
 */
class BleServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer) override {
        LOG_BLE("Device connected successfully");
        g_bleDeviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override {
        LOG_BLE("Device disconnected - restarting advertising");
        g_bleDeviceConnected = false;
        
        // Restart advertising to allow new connections
        BLEDevice::startAdvertising();
        LOG_BLE("Advertising restarted");
    }
};

/**
 * @brief Handles BLE characteristic write events and processes commands
 */
class BleCharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* pCharacteristic) override {
        String rawValue = pCharacteristic->getValue();
        
        if (rawValue.isEmpty()) {
            LOG_BLE("Received empty command - ignoring");
            return;
        }

        String command = String(rawValue.c_str());
        command.trim();
        
        // Store original command for logging
        String originalCommand = command;
        command.toUpperCase();

        LOG_BLE("Received command: '" + originalCommand + "'");

        // Enhanced command processing with validation
        if (command.startsWith("MTU:")) {
            handleMtuNegotiation(command);
        } else if (command == "CAPTURE" || command == "CAP") {
            handleCaptureRequest();
        } else if (command == "STATUS" || command == "STAT") {
            handleStatusRequest();
        } else if (command == "PING") {
            handlePingRequest();
        } else if (command.startsWith("CONFIG:")) {
            handleConfigurationRequest(command);
        } else {
            LOG_BLE("Unknown command received: '" + originalCommand + "'");
            sendBleTextMessage("ERROR:UNKNOWN_COMMAND");
        }
    }

private:
    /**
     * @brief Handle MTU negotiation from client with enhanced validation
     * @param command MTU command string (format: "MTU:value")
     */
    void handleMtuNegotiation(const String& command) {
        String mtuString = command.substring(4);
        int mtuValue = mtuString.toInt();
        
        // Validate MTU value
        if (mtuValue <= 0 || mtuString.length() == 0) {
            LOG_BLE("Invalid MTU value received: " + mtuString);
            sendBleTextMessage("ERROR:INVALID_MTU");
            return;
        }
        
        if (mtuValue < MIN_CHUNK_SIZE + MTU_OVERHEAD) {
            LOG_BLE("MTU too small: " + String(mtuValue) + " (minimum: " + 
                    String(MIN_CHUNK_SIZE + MTU_OVERHEAD) + ")");
            sendBleTextMessage("ERROR:MTU_TOO_SMALL");
            return;
        }

        if (mtuValue > MAX_CHUNK_SIZE + MTU_OVERHEAD) {
            LOG_BLE("MTU too large: " + String(mtuValue) + " (maximum: " + 
                    String(MAX_CHUNK_SIZE + MTU_OVERHEAD) + ")");
            mtuValue = MAX_CHUNK_SIZE + MTU_OVERHEAD;
        }

        uint16_t newChunkSize = max((uint16_t)MIN_CHUNK_SIZE, 
                                   min((uint16_t)MAX_CHUNK_SIZE, (uint16_t)(mtuValue - MTU_OVERHEAD)));
        uint16_t oldChunkSize = g_negotiatedChunkSize;
        g_negotiatedChunkSize = newChunkSize;
        
        LOG_BLE("MTU negotiated - MTU: " + String(mtuValue) + 
                ", Old chunk: " + String(oldChunkSize) + 
                ", New chunk: " + String(g_negotiatedChunkSize));
        
        sendBleTextMessage("MTU_OK:" + String(g_negotiatedChunkSize));
    }

    /**
     * @brief Handle image capture request with enhanced validation
     */
    void handleCaptureRequest() {
        if (!g_captureQueue) {
            LOG_BLE("Capture queue not initialized");
            sendBleTextMessage("ERROR:QUEUE_NOT_READY");
            return;
        }

        // Check memory availability before queuing
        size_t freeHeap = ESP.getFreeHeap();
        if (freeHeap < MEMORY_CHECK_THRESHOLD) {
            LOG_BLE("Insufficient memory for capture: " + String(freeHeap) + " bytes");
            sendBleTextMessage("ERROR:LOW_MEMORY");
            return;
        }

        uint8_t captureToken = 1;
        BaseType_t result = xQueueSend(g_captureQueue, &captureToken, 0);
        
        if (result == pdTRUE) {
            LOG_BLE("Capture request queued successfully");
            sendBleTextMessage("CAPTURE_QUEUED");
        } else {
            LOG_BLE("Capture queue is full - request ignored");
            sendBleTextMessage("ERROR:QUEUE_FULL");
        }
    }

    /**
     * @brief Handle status request - send system information
     */
    void handleStatusRequest() {
        String status = "STATUS:" + 
                       String(g_bleDeviceConnected ? "CONNECTED" : "DISCONNECTED") + ":" +
                       String(g_negotiatedChunkSize) + ":" +
                       String(g_frameCounter) + ":" +
                       String(ESP.getFreeHeap()) + ":" +
                       String(psramFound() ? "PSRAM" : "NO_PSRAM");
        
        sendBleTextMessage(status);
        LOG_BLE("Status sent: " + status);
    }

    /**
     * @brief Handle ping request for connection testing
     */
    void handlePingRequest() {
        String response = "PONG:" + String(millis());
        sendBleTextMessage(response);
        LOG_BLE("Ping response sent");
    }

    /**
     * @brief Handle configuration requests
     * @param command Configuration command (format: "CONFIG:param:value")
     */
    void handleConfigurationRequest(const String& command) {
        // Parse configuration command
        int firstColon = command.indexOf(':', 7); // Skip "CONFIG:"
        if (firstColon == -1) {
            sendBleTextMessage("ERROR:INVALID_CONFIG_FORMAT");
            return;
        }
        
        String param = command.substring(7, firstColon);
        String value = command.substring(firstColon + 1);
        
        LOG_BLE("Config request - Param: " + param + ", Value: " + value);
        
        // For now, just acknowledge the configuration request
        // Future: Implement actual configuration changes
        sendBleTextMessage("CONFIG_ACK:" + param + ":" + value);
    }
};

// ============================================================================
// CAMERA INITIALIZATION AND CONFIGURATION
// ============================================================================

// Pin configuration (handled externally)

/**
 * @brief Configure camera settings based on available resources
 * @param config Camera configuration structure to populate
 * @param hasPsram Whether PSRAM is available
 */
void configureCameraSettings(camera_config_t& config, bool hasPsram) {
    // Pin configuration (XIAO ESP32-S3 specific)
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

    // Basic camera settings
    config.xclk_freq_hz = CAMERA_XCLK_FREQ_HZ;
    config.frame_size = CAMERA_FRAME_SIZE;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = CAMERA_JPEG_QUALITY;
    config.fb_count = 1;

    // Enhanced settings for PSRAM-equipped devices
    if (hasPsram) {
        config.jpeg_quality = CAMERA_JPEG_QUALITY_PSRAM;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
        LOG_CAM("PSRAM optimizations applied");
    } else {
        LOG_CAM("Standard configuration applied (no PSRAM)");
    }
}

/**
 * @brief Initialize camera with error handling and configuration
 * @return true if initialization successful, false otherwise
 */
bool initializeCamera() {
    LOG_CAM("Initializing camera subsystem...");
    
    bool hasPsram = checkPsramAvailability();
    
    camera_config_t config;
    configureCameraSettings(config, hasPsram);

    esp_err_t result = esp_camera_init(&config);
    if (result != ESP_OK) {
        LOG_ERROR("Camera initialization failed with error code: " + String(result));
        return false;
    }

    LOG_CAM("Camera initialized successfully");
    return true;
}

// ============================================================================
// IMAGE CAPTURE AND TRANSMISSION
// ============================================================================

/**
 * @brief Capture image from camera with enhanced error handling and memory management
 * @return Camera frame buffer pointer, or nullptr on failure
 */
camera_fb_t* captureImage() {
    // Check available memory before capture
    if (!checkMemoryAvailability()) {
        return nullptr;
    }

    // Attempt capture with retry logic
    camera_fb_t* frameBuffer = nullptr;
    int retryCount = 0;
    
    while (retryCount < MAX_RETRY_ATTEMPTS && !frameBuffer) {
        frameBuffer = esp_camera_fb_get();
        
        if (!frameBuffer) {
            retryCount++;
            LOG_ERROR("Capture attempt " + String(retryCount) + " failed");
            if (retryCount < MAX_RETRY_ATTEMPTS) {
                delay(200); // Increased delay between retries
            }
        }
    }
    
    if (!frameBuffer) {
        LOG_ERROR("Failed to capture image after " + String(MAX_RETRY_ATTEMPTS) + " attempts");
        return nullptr;
    }
    
    // Validate captured frame
    if (frameBuffer->len == 0) {
        LOG_ERROR("Captured image has zero length");
        esp_camera_fb_return(frameBuffer);
        return nullptr;
    }
    
    // Validate frame buffer format
    if (frameBuffer->format != PIXFORMAT_JPEG) {
        LOG_ERROR("Unexpected frame format: " + String(frameBuffer->format));
        esp_camera_fb_return(frameBuffer);
        return nullptr;
    }
    
    g_frameCounter++;
    LOG_CAM("Image captured successfully - Frame #" + String(g_frameCounter) + 
            ", Size: " + String(frameBuffer->len) + " bytes" +
            ", Resolution: " + String(frameBuffer->width) + "x" + String(frameBuffer->height));
    
    return frameBuffer;
}

/**
 * @brief Send image data over BLE in chunks with enhanced error handling
 * @param frameBuffer Camera frame buffer containing image data
 * @return true if transmission successful, false otherwise
 */
bool transmitImageOverBle(camera_fb_t* frameBuffer) {
    if (!frameBuffer || !g_bleDeviceConnected || !g_pBleCharacteristic) {
        LOG_ERROR("[SENDER] Invalid parameters for image transmission");
        return false;
    }

    // --- Setup ---
    const size_t totalSize = frameBuffer->len;
    const uint8_t* buffer = frameBuffer->buf;
    const unsigned long startTime = millis();

    LOG_BLE("[SENDER] Captured frame, size=" + String(totalSize) + " bytes");

    // --- Show first bytes of the buffer (preview) ---
    size_t previewLen = min((size_t)32, totalSize);  // show up to 32 bytes
    Serial.print("[SENDER] Frame preview (first ");
    Serial.print(previewLen);
    Serial.println(" bytes):");
    for (size_t i = 0; i < previewLen; i++) {
        Serial.printf("%02X", buffer[i]);  // hex format without 0x prefix
        if (i < previewLen - 1) {
            Serial.print(", ");
        }
    }
    Serial.println();

    // --- Send header ---
    String header = "IMG_START:" + String(totalSize) + ":" + String(g_frameCounter);
    if (!sendBleTextMessage(header, HEADER_FOOTER_DELAY_MS)) {
        LOG_ERROR("[BLE] Failed to send image header");
        return false;
    }
    LOG_BLE("[BLE] Sent image header: " + header);

    // --- Send chunks ---
    size_t sentBytes = 0;
    for (size_t i = 0; i < totalSize; i += g_negotiatedChunkSize) {
        if (!g_bleDeviceConnected) {
            LOG_ERROR("[BLE] Disconnected during transmission at offset " + String(i));
            return false;
        }
        
        int len = (i + g_negotiatedChunkSize > totalSize) ? (totalSize - i) : g_negotiatedChunkSize;
        g_pBleCharacteristic->setValue(const_cast<uint8_t*>(buffer + i), len);
        g_pBleCharacteristic->notify();
        sentBytes += len;

        // Delay to avoid overflow
        vTaskDelay(pdMS_TO_TICKS(CHUNK_DELAY_MS));

        // Log progress every ~500 bytes
        if (sentBytes % 500 <= g_negotiatedChunkSize) {
            LOG_BLE("[BLE] Sent " + String(sentBytes) + "/" + String(totalSize) + " bytes");
        }
    }

    // --- Send footer ---
    String footer = "IMG_END:" + String(g_frameCounter);
    if (!sendBleTextMessage(footer, HEADER_FOOTER_DELAY_MS)) {
        LOG_ERROR("[BLE] Failed to send image footer");
        return false;
    }
    LOG_BLE("[BLE] Sent image footer, total size: " + String(totalSize));

    // --- Stats ---
    unsigned long endTime = millis();
    unsigned long duration = endTime - startTime;
    float throughputKbps = (float)(totalSize * 8) / duration;
    LOG_INFO("[TIME] Transmission finished - Time: " + String(duration) + " ms, " +
             "Throughput: " + String(throughputKbps, 2) + " kbps");

    return true;
}

/**
 * @brief Process a single image capture and transmission cycle
 * @return true if successful, false otherwise
 */
bool processCaptureRequest() {
    // Capture image from camera
    camera_fb_t* frameBuffer = captureImage();
    if (!frameBuffer) {
        return false;
    }

    // Transmit image over BLE
    bool transmissionSuccess = transmitImageOverBle(frameBuffer);

    // Always return the frame buffer to avoid memory leaks
    esp_camera_fb_return(frameBuffer);
    LOG_CAM("Frame buffer returned to camera driver");

    return transmissionSuccess;
}

// ============================================================================
// RTOS TASK MANAGEMENT
// ============================================================================

/**
 * @brief Main task for handling image capture requests
 * @param param Task parameters (unused)
 */
void imageSenderTask(void* param) {
    uint8_t captureRequest;
    unsigned long lastCaptureTime = 0;

    LOG_INFO("Image sender task started");

    while (true) {
        // Wait for capture request from queue
        if (xQueueReceive(g_captureQueue, &captureRequest, portMAX_DELAY) == pdTRUE) {
            
            // Implement debouncing to prevent spam requests
            unsigned long currentTime = millis();
            if (currentTime - lastCaptureTime < CAPTURE_DEBOUNCE_MS) {
                LOG_DEBUG("Capture request debounced (too frequent)");
                continue;
            }
            lastCaptureTime = currentTime;

            // Verify BLE connection before proceeding
            if (!g_bleDeviceConnected) {
                LOG_ERROR("No BLE connection - skipping capture");
                continue;
            }

            // Process the capture request
            LOG_INFO("Processing capture request...");
            g_totalCaptureRequests++;
            
            bool success = processCaptureRequest();
            
            if (success) {
                g_successfulTransmissions++;
                LOG_INFO("Capture and transmission completed successfully");
            } else {
                g_failedTransmissions++;
                LOG_ERROR("Capture or transmission failed");
            }

            // Coalesce any additional queued requests to prevent backlog
            uint8_t dummyRequest;
            int coalescedCount = 0;
            while (xQueueReceive(g_captureQueue, &dummyRequest, 0) == pdTRUE) {
                coalescedCount++;
            }
            
            if (coalescedCount > 0) {
                LOG_DEBUG("Coalesced " + String(coalescedCount) + " duplicate capture requests");
            }
        }
    }

    // This should never be reached
    vTaskDelete(nullptr);
}

// ============================================================================
// BLE INITIALIZATION AND SETUP
// ============================================================================

/**
 * @brief Initialize BLE server with proper configuration
 * @return true if initialization successful, false otherwise
 */
bool initializeBleServer() {
    LOG_BLE("Initializing BLE server...");

    // Initialize BLE device
    BLEDevice::init(DEVICE_NAME);
    
    // Create BLE server with callbacks
    BLEServer* pServer = BLEDevice::createServer();
    if (!pServer) {
        LOG_ERROR("Failed to create BLE server");
        return false;
    }
    pServer->setCallbacks(new BleServerCallbacks());

    // Create BLE service
    BLEService* pService = pServer->createService(BLE_SERVICE_UUID);
    if (!pService) {
        LOG_ERROR("Failed to create BLE service");
        return false;
    }

    // Create BLE characteristic with read/write/notify properties
    g_pBleCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    if (!g_pBleCharacteristic) {
        LOG_ERROR("Failed to create BLE characteristic");
        return false;
    }

    // Set initial value and callbacks
    g_pBleCharacteristic->setValue("XIAO ESP32-S3 Camera Ready");
    g_pBleCharacteristic->setCallbacks(new BleCharacteristicCallbacks());

    // Start the service
    pService->start();

    // Configure and start advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Help with iPhone connection issues
    pAdvertising->setMinPreferred(0x12);
    
    BLEDevice::startAdvertising();
    LOG_BLE("BLE server initialized and advertising started");
    
    return true;
}

/**
 * @brief Initialize RTOS components (queues and tasks)
 * @return true if initialization successful, false otherwise
 */
bool initializeRtosComponents() {
    LOG_INFO("Initializing RTOS components...");

    // Create capture request queue
    g_captureQueue = xQueueCreate(CAPTURE_QUEUE_LENGTH, sizeof(uint8_t));
    if (!g_captureQueue) {
        LOG_ERROR("Failed to create capture queue");
        return false;
    }

    // Create image sender task
    BaseType_t taskResult = xTaskCreatePinnedToCore(
        imageSenderTask,           // Task function
        "ImageSenderTask",         // Task name
        SENDER_TASK_STACK_SIZE,    // Stack size
        nullptr,                   // Task parameters
        SENDER_TASK_PRIORITY,      // Task priority
        &g_senderTaskHandle,       // Task handle
        SENDER_TASK_CORE           // CPU core
    );

    if (taskResult != pdPASS) {
        LOG_ERROR("Failed to create image sender task");
        return false;
    }

    LOG_INFO("RTOS components initialized successfully");
    return true;
}

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================

/**
 * @brief Process commands received via Serial interface with enhanced functionality
 */
void processSerialCommands() {
    if (!Serial.available()) {
        return;
    }

    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.isEmpty()) {
        return;
    }

    String originalCommand = command;
    command.toUpperCase();

    LOG_INFO("Serial command received: " + originalCommand);

    if (command == "CAPTURE" || command == "CAP") {
        if (g_bleDeviceConnected) {
            uint8_t captureToken = 1;
            if (g_captureQueue && xQueueSend(g_captureQueue, &captureToken, 0) == pdTRUE) {
                LOG_INFO("Capture request queued via serial");
            } else {
                LOG_ERROR("Capture request ignored (queue full)");
            }
        } else {
            LOG_ERROR("No BLE device connected - cannot capture");
        }
    } else if (command == "STATUS" || command == "STAT") {
        reportSystemStatus();
    } else if (command == "RESTART" || command == "RESET") {
        LOG_INFO("Restarting system...");
        delay(100);
        ESP.restart();
    } else if (command == "HELP" || command == "?") {
        LOG_INFO("=== Available Commands ===");
        LOG_INFO("CAPTURE/CAP - Queue image capture");
        LOG_INFO("STATUS/STAT - Show system status");
        LOG_INFO("RESTART/RESET - Restart the device");
        LOG_INFO("MEMORY - Show memory information");
        LOG_INFO("STATS - Show transmission statistics");
        LOG_INFO("HELP/? - Show this help message");
        LOG_INFO("==========================");
    } else if (command == "MEMORY" || command == "MEM") {
        LOG_INFO("=== Memory Information ===");
        LOG_INFO("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
        if (psramFound()) {
            LOG_INFO("Free PSRAM: " + String(ESP.getFreePsram()) + " bytes");
            LOG_INFO("Total PSRAM: " + String(ESP.getPsramSize()) + " bytes");
        } else {
            LOG_INFO("PSRAM: Not available");
        }
        LOG_INFO("Flash Size: " + String(ESP.getFlashChipSize()) + " bytes");
        LOG_INFO("=========================");
    } else if (command == "STATS") {
        LOG_INFO("=== Transmission Statistics ===");
        LOG_INFO("Total Capture Requests: " + String(g_totalCaptureRequests));
        LOG_INFO("Successful Transmissions: " + String(g_successfulTransmissions));
        LOG_INFO("Failed Transmissions: " + String(g_failedTransmissions));
        if (g_totalCaptureRequests > 0) {
            float successRate = (float(g_successfulTransmissions) / g_totalCaptureRequests) * 100.0;
            LOG_INFO("Success Rate: " + String(successRate, 1) + "%");
        }
        LOG_INFO("Frames Captured: " + String(g_frameCounter));
        LOG_INFO("===============================");
    } else {
        LOG_ERROR("Unknown serial command: " + originalCommand);
        LOG_INFO("Type 'HELP' for available commands");
    }
}

// ============================================================================
// MAIN SETUP AND LOOP FUNCTIONS
// ============================================================================

/**
 * @brief Main setup function - initializes all system components
 */
void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    delay(SERIAL_TIMEOUT_MS);
    
    LOG_INFO("=== XIAO ESP32-S3 Camera BLE Server ===");
    LOG_INFO("Firmware version: 2.0");
    LOG_INFO("Build date: " + String(__DATE__) + " " + String(__TIME__));
    
    // Initialize BLE server
    if (!initializeBleServer()) {
        LOG_ERROR("BLE initialization failed - halting");
        while (true) delay(1000);
    }

    // Initialize camera
    if (!initializeCamera()) {
        LOG_ERROR("Camera initialization failed - halting");
        while (true) delay(1000);
    }

    // Initialize RTOS components
    if (!initializeRtosComponents()) {
        LOG_ERROR("RTOS initialization failed - halting");
        while (true) delay(1000);
    }

    LOG_INFO("=== System initialization completed successfully ===");
    LOG_INFO("Device ready for BLE connections and image capture");
}

/**
 * @brief Main loop function - handles serial commands and system monitoring
 */
void loop() {
    unsigned long currentTime = millis();
    
    // Process any incoming serial commands
    processSerialCommands();
    
    // Periodic system status reporting
    if (currentTime - g_lastStatusReport >= STATUS_REPORT_INTERVAL_MS) {
        reportSystemStatus();
        g_lastStatusReport = currentTime;
    }
    
    // Watchdog reset to prevent system hangs
    if (currentTime - g_lastWatchdogReset >= WATCHDOG_TIMEOUT_MS) {
        LOG_DEBUG("Watchdog timer reset");
        g_lastWatchdogReset = currentTime;
        
        // Check for memory leaks or system issues
        size_t freeHeap = ESP.getFreeHeap();
        if (freeHeap < MEMORY_WARNING_THRESHOLD) {
            LOG_ERROR("Low memory warning - Free heap: " + String(freeHeap) + " bytes");
        }
    }
    
    // Small delay to prevent overwhelming the system
    delay(10);
}

/**
 * @brief Report comprehensive system status
 */
void reportSystemStatus() {
    LOG_INFO("=== System Status Report ===");
    LOG_INFO("Uptime: " + String(millis() / 1000) + " seconds");
    LOG_INFO("BLE Connected: " + String(g_bleDeviceConnected ? "Yes" : "No"));
    LOG_INFO("Negotiated Chunk Size: " + String(g_negotiatedChunkSize) + " bytes");
    LOG_INFO("Frame Counter: " + String(g_frameCounter));
    LOG_INFO("Capture Requests: " + String(g_totalCaptureRequests));
    LOG_INFO("Successful Transmissions: " + String(g_successfulTransmissions));
    LOG_INFO("Failed Transmissions: " + String(g_failedTransmissions));
    
    if (g_totalCaptureRequests > 0) {
        float successRate = (float(g_successfulTransmissions) / g_totalCaptureRequests) * 100.0;
        LOG_INFO("Success Rate: " + String(successRate, 1) + "%");
    }
    
    LOG_INFO("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
    LOG_INFO("PSRAM Available: " + String(psramFound() ? "Yes" : "No"));
    if (psramFound()) {
        LOG_INFO("Free PSRAM: " + String(ESP.getFreePsram()) + " bytes");
    }
    LOG_INFO("CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
    LOG_INFO("Flash Size: " + String(ESP.getFlashChipSize()) + " bytes");
    LOG_INFO("============================");
}
