#include "BluetoothManager.h"
#include "Config.h"

BluetoothManager::BluetoothManager() 
    : pCharacteristic(nullptr), taskHandle(NULL), wifiConnected(false), bleActive(false),
      axLocal(0), ayLocal(0), azLocal(0), stepCountLocal(0), heartRateLocal(0), 
      spo2Local(-999), irValueLocal(0), redValueLocal(0), wifiConnectedLocal(false) {
    dataMutex = xSemaphoreCreateMutex();
}

void BluetoothManager::begin() {
    NimBLEDevice::init("ESP32");
    NimBLEServer* pServer = NimBLEDevice::createServer();
    NimBLEService* pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );
    pCharacteristic->setCallbacks(new MyCallbacks(this)); // Truyền con trỏ this vào MyCallbacks
    pService->start();
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->start();
    bleActive = true;
    Serial.println("BLE started");
}

void BluetoothManager::startTask() {
    xTaskCreate(
        taskFunction,       // Hàm task
        "BluetoothTask",    // Tên task
        4096,               // Stack size
        this,               // Tham số
        1,                  // Độ ưu tiên
        &taskHandle         // Handle
    );
}

void BluetoothManager::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        NimBLEDevice::deinit(true);
        WiFi.mode(WIFI_OFF);
        bleActive = false;
        wifiConnected = false;
        Serial.println("Bluetooth task stopped");
    }
}

void BluetoothManager::taskFunction(void* pvParameters) {
    BluetoothManager* instance = static_cast<BluetoothManager*>(pvParameters);
    while (true) {
        instance->processData();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void BluetoothManager::updateData(float ax, float ay, float az, int stepCount, int heartRate, 
                                 int spo2, long irValue, long redValue, bool wifiConnected) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        axLocal = ax;
        ayLocal = ay;
        azLocal = az;
        stepCountLocal = stepCount;
        heartRateLocal = heartRate;
        spo2Local = spo2;
        irValueLocal = irValue;
        redValueLocal = redValue;
        wifiConnectedLocal = wifiConnected;
        xSemaphoreGive(dataMutex);
    }
}

void BluetoothManager::sendHealthData(float ax, float ay, float az, int stepCount, int heartRate, 
                                     int spo2, long irValue, long redValue, bool wifiConnected) {
    updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, wifiConnected);
    processData();
}

void BluetoothManager::processData() {
    if (!bleActive || !pCharacteristic->getSubscribedCount()) {
        if (bleActive) {
            NimBLEDevice::deinit(true);
            bleActive = false;
            Serial.println("BLE stopped: No subscribers");
        }
        return;
    }

    if (!hasDataChanged()) {
        return;
    }

    StaticJsonDocument<128> doc;
    doc["x"] = axLocal;
    doc["y"] = ayLocal;
    doc["z"] = azLocal;
    doc["s"] = stepCountLocal;
    doc["h"] = heartRateLocal;
    doc["o"] = spo2Local;
    doc["i"] = irValueLocal;
    doc["r"] = redValueLocal;
    doc["w"] = wifiConnectedLocal ? 1 : 0;

    String jsonData;
    serializeJson(doc, jsonData);

    pCharacteristic->setValue(jsonData);
    pCharacteristic->notify();
    Serial.println("Sent: " + jsonData);
}

bool BluetoothManager::hasDataChanged() {
    static float lastAx = 0, lastAy = 0, lastAz = 0;
    static int lastStepCount = 0, lastHeartRate = 0, lastSpo2 = -999;
    static long lastIrValue = 0, lastRedValue = 0;
    static bool lastWifiConnected = false;

    bool changed = false;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        changed = (abs(axLocal - lastAx) > 0.1 || abs(ayLocal - lastAy) > 0.1 || 
                   abs(azLocal - lastAz) > 0.1 || stepCountLocal != lastStepCount || 
                   heartRateLocal != lastHeartRate || spo2Local != lastSpo2 || 
                   irValueLocal != lastIrValue || redValueLocal != lastRedValue || 
                   wifiConnectedLocal != lastWifiConnected);

        lastAx = axLocal;
        lastAy = ayLocal;
        lastAz = azLocal;
        lastStepCount = stepCountLocal;
        lastHeartRate = heartRateLocal;
        lastSpo2 = spo2Local;
        lastIrValue = irValueLocal;
        lastRedValue = redValueLocal;
        lastWifiConnected = wifiConnectedLocal;
        xSemaphoreGive(dataMutex);
    }
    return changed;
}

bool BluetoothManager::isWifiConnected() {
    return wifiConnected;
}

void BluetoothManager::connectToWiFi(std::string ssid, std::string password) {
    WiFi.begin(ssid.c_str(), password.c_str());
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 5) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        retries++;
    }
    wifiConnected = (WiFi.status() == WL_CONNECTED);
    pCharacteristic->setValue(wifiConnected ? "success" : "fail: connection error");
    pCharacteristic->notify();

    if (!wifiConnected) {
        WiFi.mode(WIFI_OFF);
    }
}

void BluetoothManager::MyCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string data = pCharacteristic->getValue();
    int separator = data.find(",");
    if (separator != std::string::npos) {
        std::string ssid = data.substr(0, separator);
        std::string password = data.substr(separator + 1);
        manager->connectToWiFi(ssid, password);
    } else {
        pCharacteristic->setValue("fail: invalid format");
        pCharacteristic->notify();
    }
}