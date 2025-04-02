// #ifndef DISPLAY_MANAGER_H
// #define DISPLAY_MANAGER_H

// #include <TFT_eSPI.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <freertos/semphr.h> // Thêm include

// class DisplayManager {
// public:
//     DisplayManager();
//     void begin();
//     void startTask();
//     void stopTask();
//     void updateData(bool wifiConnected, int stepCount, float distance, int heartRate, int spo2, struct tm* timeinfo, bool timeInitialized);
//     void toggleScreen(bool screenOn);

// private:
//     TFT_eSPI tft;
//     TaskHandle_t taskHandle;
//     bool screenOn;
//     bool displayActive;
//     static void taskFunction(void* pvParameters);
//     void updateDisplay();
//     void drawIcons(bool wifiConnected);
//     void partialUpdate(int x, int y, int w, int h, const char* text, uint16_t color);
//     bool wifiConnectedLocal;
//     int stepCountLocal;
//     float distanceLocal;
//     int heartRateLocal;
//     int spo2Local;
//     struct tm timeinfoLocal;
//     bool timeInitializedLocal;
//     SemaphoreHandle_t dataMutex;
// };

// #endif

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <TFT_eSPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class DisplayManager {
public:
    DisplayManager();
    void begin();
    void startTask();
    void stopTask();
    void updateData(bool wifiConnected, int stepCount, float distance, int heartRate, int spo2, struct tm* timeinfo, bool timeInitialized);
    void toggleScreen(bool screenOn);
    void switchDisplayMode(); // Hàm mới để chuyển chế độ hiển thị

private:
    TFT_eSPI tft;
    TaskHandle_t taskHandle;
    bool screenOn;
    bool displayActive;
    int displayMode; // 0: Steps, 1: HeartRate, 2: Time
    static void taskFunction(void* pvParameters);
    void updateDisplay();
    void drawIcons(bool wifiConnected);
    void partialUpdate(int x, int y, int w, int h, const char* text, uint16_t color);
    bool wifiConnectedLocal;
    int stepCountLocal;
    float distanceLocal;
    int heartRateLocal;
    int spo2Local;
    struct tm timeinfoLocal;
    bool timeInitializedLocal;
    SemaphoreHandle_t dataMutex;
};

#endif