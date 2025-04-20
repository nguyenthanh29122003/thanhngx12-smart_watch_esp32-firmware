#include "BluetoothManager.h"
#include "Config.h"
#include <string.h>

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
    Serial.println("Received data: " + value);
    
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, value);
    if (error) {
        Serial.println("JSON parse failed: " + String(error.c_str()));
        return;
    }

    // Xử lý WiFi config
    if (doc.containsKey("ssid") && doc.containsKey("password")) {
        manager->wifiSSID = doc["ssid"].as<String>();
        manager->wifiPassword = doc["password"].as<String>();
        manager->connectWiFi();
    }
    // Xử lý thời gian
    else if (doc.containsKey("time")) {
        struct tm timeinfo = {};
        timeinfo.tm_year = doc["time"]["year"].as<int>() - 1900; // struct tm: năm từ 1900
        timeinfo.tm_mon = doc["time"]["month"].as<int>() - 1;    // 0-11
        timeinfo.tm_mday = doc["time"]["day"].as<int>();
        timeinfo.tm_hour = doc["time"]["hour"].as<int>();
        timeinfo.tm_min = doc["time"]["minute"].as<int>();
        timeinfo.tm_sec = doc["time"]["second"].as<int>();
        // Bỏ qua timezone (giả định đã được xử lý ở client, +7)
        manager->timeManager->setTimeFromBLE(timeinfo);
    }
    else {
        Serial.println("Unknown JSON format");
    }
}

BluetoothManager::BluetoothManager(TimeManager* timeManager) 
    : pServer(nullptr), pDataCharacteristic(nullptr), pWifiConfigCharacteristic(nullptr), taskHandle(NULL),
      axLocal(0), ayLocal(0), azLocal(0), gxLocal(0), gyLocal(0), gzLocal(0),
      stepCountLocal(0), heartRateLocal(0), spo2Local(0), 
      irValueLocal(0), redValueLocal(0), wifiConnectedLocal(false),
      timestampLocal("Not initialized"), wifiSSID(""), wifiPassword(""), timeManager(timeManager) {
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
        instance->sendData();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void BluetoothManager::updateData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, 
                                 long irValue, long redValue, bool wifiConnected, float gx, float gy, float gz, const char* timestamp) {
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
        timestampLocal = (timestamp != nullptr) ? String(timestamp) : "Not initialized";
        xSemaphoreGive(dataMutex);
    }
}

void BluetoothManager::sendData() {
    String jsonStr = "{";
    float _ax, _ay, _az, _gx, _gy, _gz;
    int _steps, _hr, _spo2;
    long _ir, _red;
    bool _wifi;
    String _timestamp;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _ax = axLocal; _ay = ayLocal; _az = azLocal;
        _gx = gxLocal; _gy = gyLocal; _gz = gzLocal;
        _steps = stepCountLocal; _hr = heartRateLocal; _spo2 = spo2Local;
        _ir = irValueLocal; _red = redValueLocal;
        _wifi = wifiConnectedLocal; _timestamp = timestampLocal;
        xSemaphoreGive(dataMutex);

        jsonStr += "\"ax\":" + String(_ax, 2);
        jsonStr += ", \"ay\":" + String(_ay, 2);
        jsonStr += ", \"az\":" + String(_az, 2);
        jsonStr += ", \"gx\":" + String(_gx, 2);
        jsonStr += ", \"gy\":" + String(_gy, 2);
        jsonStr += ", \"gz\":" + String(_gz, 2);
        jsonStr += ", \"steps\":" + String(_steps);
        jsonStr += ", \"hr\":" + String(_hr);
        jsonStr += ", \"spo2\":" + String(_spo2);
        jsonStr += ", \"ir\":" + String(_ir);
        jsonStr += ", \"red\":" + String(_red);
        jsonStr += ", \"wifi\":" + String(_wifi ? "true" : "false");
        jsonStr += ", \"timestamp\":\"" + _timestamp + "\"";
        jsonStr += "}";

        if (pDataCharacteristic != nullptr && pServer->getConnectedCount() > 0) {
            try {
                Serial.println("Sending JSON (len " + String(jsonStr.length()) + "): " + jsonStr);
                pDataCharacteristic->setValue(jsonStr);
                pDataCharacteristic->notify();
            } catch (const std::exception& e) {
                Serial.printf("BLE Send Exception: %s\n", e.what());
            } catch (...) {
                Serial.println("Unknown BLE Send Exception");
            }
        }
    } else {
        Serial.println("Mutex timeout in sendData! Skipping send.");
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
    }
}