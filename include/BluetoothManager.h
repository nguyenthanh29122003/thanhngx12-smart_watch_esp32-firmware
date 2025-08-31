// include/BluetoothManager.h
#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "TimeManager.h"

class BluetoothManager; // Forward declaration

class NavigationCallbacks : public NimBLECharacteristicCallbacks {
public:
    NavigationCallbacks(BluetoothManager* manager) : p_manager(manager) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override;
private:
    BluetoothManager* p_manager;
};

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

struct NavigationInfo {
    String nextTurnDirection;      // "Rẽ trái"
    String nextTurnDistance;       // "200 m"
    String streetName;             // "Đ. Trần Hưng Đạo"
    String totalRemainingDistance; // "5.4 km"
    String eta;                    // "15 phút"
    // Hàm để reset dữ liệu
    void clear() {
        nextTurnDirection = "";
        nextTurnDistance = "";
        streetName = "";
        totalRemainingDistance = "";
        eta = "";
    }
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
    NavigationInfo getNavigationInfo();

private:
    NimBLEServer* pServer;
    NimBLECharacteristic* pDataCharacteristic;
    NimBLECharacteristic* pWifiConfigCharacteristic;
    NimBLECharacteristic* pStatusCharacteristic; // Khai báo Status Char
    NimBLECharacteristic* pNavigationCharacteristic;
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
    
    NavigationInfo navInfoLocal;

    // --- Hàm private ---
    static void taskFunction(void* pvParameters);
    void setupBLE();                       // Thiết lập BLE
    void sendData();                       // Gửi dữ liệu (BẮT BUỘC DÙNG STRING)
    // void connectWiFi();                 // <-- HÀM CŨ, SẼ THAY BẰNG initWiFi + event handler
    static void wifiEventHandler(WiFiEvent_t event, WiFiEventInfo_t info); // Static handler
    void initWiFi(); // Hàm đăng ký event và bắt đầu kết nối non-blocking
    void processNavigationData(const char* jsonData);

    // Friend class để callback truy cập được
    friend class WifiConfigCallbacks;
    friend class MyServerCallbacks;
    friend class NavigationCallbacks;
};

#endif // BLUETOOTH_MANAGER_H