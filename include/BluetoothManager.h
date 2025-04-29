// include/BluetoothManager.h
#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <NimBLEDevice.h>
// Bỏ #include <ArduinoJson.h> nếu không dùng
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "TimeManager.h"

class BluetoothManager; // Forward declaration

// --- WifiConfigCallbacks (Giữ nguyên) ---
class WifiConfigCallbacks : public NimBLECharacteristicCallbacks {
public:
    WifiConfigCallbacks(BluetoothManager* manager) : manager(manager) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override;
private:
    BluetoothManager* manager;
};

// --- Server Callbacks (Giữ nguyên) ---
class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override;
    void onDisconnect(NimBLEServer* pServer) override;
};


class BluetoothManager {
public:
    BluetoothManager(TimeManager* timeManager);
    void begin();
    void startTask(UBaseType_t priority = 1);
    void stopTask();
    // ===== SỬA KHAI BÁO updateData =====
    void updateData(float ax, float ay, float az, int stepCount, int heartRate, int spo2,
                   long irValue, long redValue, bool wifiConnected, float gx, float gy, float gz,
                   float temperature, float pressure, // <-- THÊM THAM SỐ MỚI
                   const char* timestamp);
    bool isWifiConnected(); // Giữ lại hàm này

    // Hàm cập nhật trạng thái (cho Status Characteristic)
    void setStatus(const String &status);

private:
    NimBLEServer* pServer;
    NimBLECharacteristic* pDataCharacteristic;
    NimBLECharacteristic* pWifiConfigCharacteristic;
    NimBLECharacteristic* pStatusCharacteristic; // Khai báo Status Char
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex; // Mutex bảo vệ dữ liệu local

    // --- Dữ liệu cục bộ ---
    float axLocal, ayLocal, azLocal;
    float gxLocal, gyLocal, gzLocal;
    int stepCountLocal;
    int heartRateLocal;
    int spo2Local;
    long irValueLocal, redValueLocal;
    bool wifiConnectedLocal;
    String timestampLocal;
    float temperatureLocal; // <-- THÊM
    float pressureLocal;    // <-- THÊM
    String statusLocal;     // <-- THÊM

    // --- Cấu hình WiFi ---
    String wifiSSID;
    String wifiPassword;

    // --- Con trỏ phụ thuộc ---
    TimeManager* timeManager;

    // --- Hàm private ---
    static void taskFunction(void* pvParameters);
    void setupBLE();                       // Thiết lập BLE
    void sendData();                       // Gửi dữ liệu (BẮT BUỘC DÙNG STRING)
    // void connectWiFi();                 // <-- HÀM CŨ, SẼ THAY BẰNG initWiFi + event handler
    static void wifiEventHandler(WiFiEvent_t event, WiFiEventInfo_t info); // Static handler
    void initWiFi(); // Hàm đăng ký event và bắt đầu kết nối non-blocking

    // Friend class để callback truy cập được
    friend class WifiConfigCallbacks;
    friend class MyServerCallbacks; // Cho phép callback server truy cập (nếu cần)
};

#endif // BLUETOOTH_MANAGER_H