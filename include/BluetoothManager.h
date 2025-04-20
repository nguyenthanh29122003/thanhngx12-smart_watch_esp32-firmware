#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "TimeManager.h" // Thêm include

class BluetoothManager;

class WifiConfigCallbacks : public NimBLECharacteristicCallbacks {
public:
    WifiConfigCallbacks(BluetoothManager* manager) : manager(manager) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override;

private:
    BluetoothManager* manager;
};

class BluetoothManager {
public:
    BluetoothManager(TimeManager* timeManager); // Thêm con trỏ TimeManager
    void begin();
    void startTask();
    void stopTask();
    void updateData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, 
                   long irValue, long redValue, bool wifiConnected, float gx, float gy, float gz, const char* timestamp);
    void sendData();
    bool isWifiConnected();

private:
    NimBLEServer* pServer;
    NimBLECharacteristic* pDataCharacteristic;
    NimBLECharacteristic* pWifiConfigCharacteristic;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex;
    float axLocal, ayLocal, azLocal;
    float gxLocal, gyLocal, gzLocal;
    int stepCountLocal;
    int heartRateLocal;
    int spo2Local;
    long irValueLocal, redValueLocal;
    bool wifiConnectedLocal;
    String timestampLocal;
    String wifiSSID;
    String wifiPassword;
    TimeManager* timeManager; // Con trỏ đến TimeManager
    static void taskFunction(void* pvParameters);
    void setupBLE();
    void connectWiFi();
    friend class WifiConfigCallbacks;
};

#endif