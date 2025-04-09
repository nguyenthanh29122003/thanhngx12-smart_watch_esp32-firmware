#include "BluetoothManager.h"
#include "Config.h"

class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        Serial.println("Device connected");
    }
    void onDisconnect(NimBLEServer* pServer) override {
        Serial.println("Device disconnected");
        NimBLEDevice::startAdvertising();
    }
};

void WifiConfigCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    Serial.println("Received WiFi config: " + value);
    
    StaticJsonDocument<128> doc;
    deserializeJson(doc, value);
    manager->wifiSSID = doc["ssid"].as<String>();
    manager->wifiPassword = doc["password"].as<String>();
    manager->connectWiFi();
}

BluetoothManager::BluetoothManager() 
    : pServer(nullptr), pDataCharacteristic(nullptr), pWifiConfigCharacteristic(nullptr), taskHandle(NULL),
      wifiSSID(""), wifiPassword("") {
}

void BluetoothManager::begin() {
    NimBLEDevice::init("ESP32_SmartWatch");
    setupBLE();
}

void BluetoothManager::setupBLE() {
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pDataCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pWifiConfigCharacteristic = pService->createCharacteristic(
        WIFI_CONFIG_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pWifiConfigCharacteristic->setCallbacks(new WifiConfigCallbacks(this));

    pService->start();
    NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    Serial.println("BLE advertising started");
}

void BluetoothManager::startTask() {
    xTaskCreate(
        taskFunction, "BluetoothTask", 4096, this, 1, &taskHandle
    );
}

void BluetoothManager::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        Serial.println("Bluetooth task stopped");
    }
}

void BluetoothManager::taskFunction(void* pvParameters) {
    BluetoothManager* instance = static_cast<BluetoothManager*>(pvParameters);
    while (true) {
        instance->sendWifiStatus(); // Chỉ gửi trạng thái WiFi nếu cần
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Giảm tần suất, ví dụ 5s
    }
}

void BluetoothManager::sendHealthData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, 
                                      long irValue, long redValue, bool wifiConnected, float gx, float gy, float gz, const char* timestamp) {
    StaticJsonDocument<256> doc;
    doc["ax"] = ax;
    doc["ay"] = ay;
    doc["az"] = az;
    doc["gx"] = gx;
    doc["gy"] = gy;
    doc["gz"] = gz;
    doc["steps"] = stepCount;
    doc["hr"] = heartRate;
    doc["spo2"] = spo2;
    doc["ir"] = irValue;
    doc["red"] = redValue;
    doc["wifi"] = wifiConnected;
    doc["timestamp"] = timestamp;

    String jsonStr;
    serializeJson(doc, jsonStr);
    if (pDataCharacteristic) {
        pDataCharacteristic->setValue(jsonStr.c_str());
        pDataCharacteristic->notify();
        Serial.println("Health data sent: " + jsonStr);
    }
}

bool BluetoothManager::isWifiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void BluetoothManager::connectWiFi() {
    if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        Serial.print("Connecting to WiFi: ");
        Serial.println(wifiSSID);
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi connected");
        } else {
            Serial.println("\nWiFi connection failed");
        }
    }
}

void BluetoothManager::sendWifiStatus() {
    StaticJsonDocument<64> doc;
    doc["wifi"] = isWifiConnected();
    String jsonStr;
    serializeJson(doc, jsonStr);
    if (pDataCharacteristic) {
        pDataCharacteristic->setValue(jsonStr.c_str());
        pDataCharacteristic->notify();
        Serial.println("WiFi status sent: " + jsonStr);
    }
}