#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class BluetoothManager {
public:
    BluetoothManager();
    void begin();
    void startTask();
    void stopTask();
    void sendHealthData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, long irValue, long redValue, bool wifiConnected);
    bool isWifiConnected();
    void updateData(float ax, float ay, float az, int stepCount, int heartRate, int spo2, long irValue, long redValue, bool wifiConnected);

private:
    NimBLECharacteristic* pCharacteristic;
    TaskHandle_t taskHandle;
    bool wifiConnected;
    bool bleActive;
    static void taskFunction(void* pvParameters);
    void connectToWiFi(std::string ssid, std::string password);
    void processData();
    bool hasDataChanged();
    float axLocal, ayLocal, azLocal;
    int stepCountLocal;
    int heartRateLocal;
    int spo2Local;
    long irValueLocal, redValueLocal;
    bool wifiConnectedLocal;
    SemaphoreHandle_t dataMutex;

    class MyCallbacks : public NimBLECharacteristicCallbacks {
    public:
        MyCallbacks(BluetoothManager* mgr) : manager(mgr) {} // Truyền con trỏ BluetoothManager
        void onWrite(NimBLECharacteristic* pCharacteristic) override;
    private:
        BluetoothManager* manager; // Lưu con trỏ đến BluetoothManager
    };
};

#endif