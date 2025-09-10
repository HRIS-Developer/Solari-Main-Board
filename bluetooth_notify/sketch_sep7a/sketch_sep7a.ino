#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

volatile bool deviceConnected = false;
BLECharacteristic* pCharacteristic;

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
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue().c_str();
    Serial.print("[BLE] Message from app: ");
    Serial.println(value);
  }
};

// ==========================
// Setup
// ==========================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("[BLE] Starting BLE setup...");

  BLEDevice::init("XIAO_ESP32S3");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

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
  if (deviceConnected) {
    digitalWrite(LED_BUILTIN, LOW); // LED ON when connected  

    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.length() > 0) {
        if (input == "SEND50K") {
          send50KData(); // send dummy 50KB in 517-byte chunks
        } else {
          pCharacteristic->setValue(input.c_str());
          pCharacteristic->notify();
          Serial.print("[BLE] Notified: ");
          Serial.println(input);
        }
      }
    }

  } else {
    digitalWrite(LED_BUILTIN, HIGH);  // LED OFF when disconnected
  }

  delay(10);
}

// ==========================
// Send payload in 517-byte chunks
// ==========================
void send50KData() {
  while (Serial.available() > 0) Serial.read(); // flush leftover input

  const int totalSize = 50000;   
  const int chunkSize = 517;     // fixed chunk
  static char buffer[517];

  Serial.print("[BLE] Sending 50KB using chunk size: ");
  Serial.println(chunkSize);

  unsigned long startTime = millis();

  for (int i = 0; i < totalSize; i += chunkSize) {
    int size = (i + chunkSize > totalSize) ? (totalSize - i) : chunkSize;

    memset(buffer, 'A', size);
    pCharacteristic->setValue((uint8_t*)buffer, size);
    pCharacteristic->notify();
    delay(5); // small delay to avoid congestion
  }

  unsigned long endTime = millis();
  Serial.print("[BLE] 50KB data sent in ");
  Serial.print(endTime - startTime);
  Serial.println(" ms");
}
