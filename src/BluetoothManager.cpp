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
      axLocal(0), ayLocal(0), azLocal(0), gxLocal(0), gyLocal(0), gzLocal(0),
      stepCountLocal(0), heartRateLocal(0), spo2Local(0), 
      irValueLocal(0), redValueLocal(0), wifiConnectedLocal(false), bleConnected(false),
      wifiSSID(""), wifiPassword("") {
    dataMutex = xSemaphoreCreateMutex();
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
    ); // Xóa createDescriptor("2902") vì NimBLE tự động thêm

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
        instance->processData();
        instance->sendWifiStatus();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void BluetoothManager::processData() {
    StaticJsonDocument<256> doc;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        doc["ax"] = axLocal;
        doc["ay"] = ayLocal;
        doc["az"] = azLocal;
        doc["gx"] = gxLocal;
        doc["gy"] = gyLocal;
        doc["gz"] = gzLocal;
        doc["steps"] = stepCountLocal;
        doc["hr"] = heartRateLocal;
        doc["spo2"] = spo2Local;
        doc["ir"] = irValueLocal;
        doc["red"] = redValueLocal;
        doc["wifi"] = wifiConnectedLocal;
        xSemaphoreGive(dataMutex);
    }

    String jsonStr;
    serializeJson(doc, jsonStr);
    if (pDataCharacteristic) {
        pDataCharacteristic->setValue(jsonStr.c_str());
        pDataCharacteristic->notify();
        Serial.println("Sent: " + jsonStr);
    }
}

void BluetoothManager::updateData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, 
                                 long irValue, long redValue, bool wifiConnected, float gx, float gy, float gz) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        axLocal = ax;
        ayLocal = ay;
        azLocal = az;
        gxLocal = gx;
        gyLocal = gy;
        gzLocal = gz;
        stepCountLocal = stepCount;
        heartRateLocal = heartRate;
        spo2Local = spo2;
        irValueLocal = irValue;
        redValueLocal = redValue;
        wifiConnectedLocal = wifiConnected;
        xSemaphoreGive(dataMutex);
    }
}

void BluetoothManager::sendHealthData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, 
                                      long irValue, long redValue, bool wifiConnected) {
    StaticJsonDocument<256> doc;
    doc["ax"] = ax;
    doc["ay"] = ay;
    doc["az"] = az;
    doc["steps"] = stepCount;
    doc["hr"] = heartRate;
    doc["spo2"] = spo2;
    doc["ir"] = irValue;
    doc["red"] = redValue;
    doc["wifi"] = wifiConnected;

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
            wifiConnectedLocal = true;
        } else {
            Serial.println("\nWiFi connection failed");
            wifiConnectedLocal = false;
        }
        sendWifiStatus();
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