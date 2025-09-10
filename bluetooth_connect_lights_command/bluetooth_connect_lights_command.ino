#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

volatile bool deviceConnected = false;
BLECharacteristic *pCharacteristic;  // make it global so we can update it

// ==========================
// BLE Server Callbacks
// ==========================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("[BLE] Device connected!");
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("[BLE] Device disconnected!");
    deviceConnected = false;
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Restarting advertising...");
  }
};

// ==========================
// BLE Characteristic Callbacks
// ==========================
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* pCharacteristic) override {
    Serial.print("[BLE] Characteristic read: ");
    Serial.println(pCharacteristic->getValue().c_str());
  }

  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue().c_str();
    Serial.print("[BLE] Message from app: ");
    Serial.println(value);

    // Handle LED commands
    if (value.equalsIgnoreCase("lights on")) {
      digitalWrite(LED_BUILTIN, LOW);   // ON (XIAO ESP32: LOW = ON)
      pCharacteristic->setValue("LED is ON");
      Serial.println("[BLE] -> LED turned ON, characteristic set to: LED is ON");
    } 
    else if (value.equalsIgnoreCase("lights off")) {
      digitalWrite(LED_BUILTIN, HIGH);  // OFF
      pCharacteristic->setValue("LED is OFF");
      Serial.println("[BLE] -> LED turned OFF, characteristic set to: LED is OFF");
    } 
    else {
      String response = value;
      pCharacteristic->setValue(response.c_str());
      Serial.println("[BLE] -> " + response);
    }
  }
};


// ==========================
// Setup
// ==========================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // works with XIAO ESP32S3
  digitalWrite(LED_BUILTIN, HIGH); // LED OFF initially

  Serial.begin(115200);
  Serial.println("[BLE] Starting BLE setup...");

  BLEDevice::init("XIAO_ESP32S3");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setValue("Hello World!");
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started...");
}

// ==========================
// Main Loop
// ==========================
void loop() {
  // LED status is handled in onWrite
  delay(10);
}
