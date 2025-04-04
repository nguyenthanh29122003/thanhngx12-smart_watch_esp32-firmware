#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <NimBLEDevice.h> // Thay BLEDevice.h bằng NimBLEDevice.h
#include <ArduinoJson.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"

class BluetoothManager;

class WifiConfigCallbacks : public NimBLECharacteristicCallbacks { // Thay BLECharacteristicCallbacks
public:
    WifiConfigCallbacks(BluetoothManager* manager) : manager(manager) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override;

private:
    BluetoothManager* manager;
};

class BluetoothManager {
public:
    BluetoothManager();
    void begin();
    void startTask();
    void stopTask();
    void updateData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, 
                    long irValue, long redValue, bool wifiConnected, float gx, float gy, float gz);
    void sendHealthData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, 
                        long irValue, long redValue, bool wifiConnected);
    bool isWifiConnected();

private:
    NimBLEServer* pServer; // Thay BLEServer
    NimBLECharacteristic* pDataCharacteristic; // Thay BLECharacteristic
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
    bool bleConnected;
    String wifiSSID;
    String wifiPassword;
    static void taskFunction(void* pvParameters);
    void processData();
    void setupBLE();
    void connectWiFi();
    void sendWifiStatus();
    friend class WifiConfigCallbacks;
};

#endif