// // #include <Arduino.h>
// // #include <Wire.h> // Thêm Wire để khởi tạo I2C
// // #include "Config.h"
// // #include "DisplayManager.h"
// // #include "StepCounter.h"
// // #include "HeartRateSpO2.h"
// // #include "BluetoothManager.h"
// // #include "TimeManager.h"

// // DisplayManager display;
// // StepCounter stepCounter;
// // HeartRateSpO2 heartRateSpO2;
// // BluetoothManager ble;
// // TimeManager timeManager;

// // int stepCount = 0;
// // float distance = 0;
// // int heartRate = 0;
// // int spo2 = 0;
// // float ax = 0, ay = 0, az = 0;
// // long irValue = 0, redValue = 0;
// // struct tm currentTime;
// // bool timeInitialized = false;
// // bool screenOn = true;
// // unsigned long lastSendTime = 0;
// // const unsigned long sendInterval = 60000;

// // void setup() {
// //     Serial.begin(115200);
// //     pinMode(BUTTON_DISPLAY, INPUT_PULLUP);

// //     Wire.begin(); // Khởi tạo I2C với chân mặc định (SDA: 21, SCL: 22)

// //     display.begin();
// //     stepCounter.begin();
// //     heartRateSpO2.begin();
// //     ble.begin();
// //     timeManager.begin();

// //     stepCounter.startTask();
// //     heartRateSpO2.startTask();
// //     ble.startTask();
// //     timeManager.startTask();
// //     display.startTask();
// // }

// // void loop() {
// //     if (digitalRead(BUTTON_DISPLAY) == LOW) {
// //         screenOn = !screenOn;
// //         display.toggleScreen(screenOn);
// //         vTaskDelay(200 / portTICK_PERIOD_MS);
// //     }

// //     stepCounter.getData(stepCount, distance, ax, ay, az);
// //     heartRateSpO2.getData(heartRate, spo2, irValue, redValue);
// //     timeManager.getTime(currentTime, timeInitialized);
// //     ble.updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected());
// //     display.updateData(ble.isWifiConnected(), stepCount, distance, heartRate, spo2, &currentTime, timeInitialized);

// //     unsigned long currentMillis = millis();
// //     if (currentMillis - lastSendTime >= sendInterval) {
// //         ble.sendHealthData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected());
// //         lastSendTime = currentMillis;
// //     }

// //     vTaskDelay(100 / portTICK_PERIOD_MS);
// // }

// #include <Arduino.h>
// #include <Wire.h>
// #include "Config.h"
// #include "DisplayManager.h"
// #include "StepCounter.h"
// #include "HeartRateSpO2.h"
// #include "BluetoothManager.h"
// #include "TimeManager.h"

// DisplayManager display;
// StepCounter stepCounter;
// HeartRateSpO2 heartRateSpO2;
// BluetoothManager ble;
// TimeManager timeManager;

// int stepCount = 0;
// float distance = 0;
// int heartRate = 0;
// int spo2 = 0;
// float ax = 0, ay = 0, az = 0;
// long irValue = 0, redValue = 0;
// struct tm currentTime;
// bool timeInitialized = false;
// bool screenOn = true;
// unsigned long lastSendTime = 0;
// const unsigned long sendInterval = 60000;

// TaskHandle_t buttonTaskHandle = NULL;
// SemaphoreHandle_t buttonSemaphore;
// volatile unsigned long lastPressTime = 0;
// volatile int pressCount = 0;

// void IRAM_ATTR buttonISR() {
//     unsigned long currentTime = millis();
//     if (currentTime - lastPressTime > 50) { // Debounce cơ bản 50ms
//         pressCount++;
//         lastPressTime = currentTime;
//         xSemaphoreGiveFromISR(buttonSemaphore, NULL);
//     }
// }

// void buttonTask(void* pvParameters) {
//     unsigned long pressStart = 0;
//     int localPressCount = 0;

//     while (true) {
//         if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
//             pressStart = lastPressTime;
//             localPressCount = pressCount;

//             // Kiểm tra giữ lâu
//             while (digitalRead(BUTTON_DISPLAY) == LOW) {
//                 if (millis() - pressStart > 2000) { // Nhấn lâu > 2 giây
//                     Serial.println("Long press - Resetting system...");
//                     ESP.restart();
//                     break;
//                 }
//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }

//             // Chờ 500ms để xác định nhấn đơn hay nhấn đôi
//             vTaskDelay(500 / portTICK_PERIOD_MS);
//             if (pressCount == localPressCount) { // Không có lần nhấn mới
//                 if (localPressCount == 1) { // Nhấn đơn
//                     screenOn = !screenOn;
//                     display.toggleScreen(screenOn);
//                     Serial.println("Single press - Screen toggled");
//                 } else if (localPressCount == 2) { // Nhấn đôi
//                     display.switchDisplayMode();
//                     Serial.println("Double press - Switched display mode");
//                 }
//                 pressCount = 0; // Reset sau khi xử lý
//             }
//         }
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     Serial.printf("Free heap at start: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
//     pinMode(BUTTON_DISPLAY, INPUT_PULLUP);

//     buttonSemaphore = xSemaphoreCreateBinary();
//     attachInterrupt(digitalPinToInterrupt(BUTTON_DISPLAY), buttonISR, FALLING);

//     Wire.begin();

//     display.begin();
//     stepCounter.begin();
//     heartRateSpO2.begin();
//     ble.begin();
//     timeManager.begin();

//     stepCounter.startTask();
//     heartRateSpO2.startTask();
//     ble.startTask();
//     timeManager.startTask();
//     display.startTask();

//     xTaskCreate(
//         buttonTask,         // Hàm task
//         "ButtonTask",       // Tên task
//         2048,               // Stack size
//         NULL,               // Tham số
//         3,                  // Độ ưu tiên cao hơn các task khác
//         &buttonTaskHandle   // Handle
//     );
// }

// void loop() {
//     stepCounter.getData(stepCount, distance, ax, ay, az);
//     heartRateSpO2.getData(heartRate, spo2, irValue, redValue);
//     timeManager.getTime(currentTime, timeInitialized);
//     ble.updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected());
//     display.updateData(ble.isWifiConnected(), stepCount, distance, heartRate, spo2, &currentTime, timeInitialized);

//     unsigned long currentMillis = millis();
//     if (currentMillis - lastSendTime >= sendInterval) {
//         ble.sendHealthData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected());
//         lastSendTime = currentMillis;
//     }

//     Serial.printf("Free heap: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
//     vTaskDelay(100 / portTICK_PERIOD_MS);
// }

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "DisplayManager.h"
#include "StepCounter.h"
#include "HeartRateSpO2.h"
#include "BluetoothManager.h"
#include "TimeManager.h"

DisplayManager display;
StepCounter stepCounter;
HeartRateSpO2 heartRateSpO2;
BluetoothManager ble;
TimeManager timeManager;

int stepCount = 0;
float distance = 0;
int heartRate = 0;
int spo2 = 0;
float ax = 0, ay = 0, az = 0;
long irValue = 0, redValue = 0;
struct tm currentTime;
bool timeInitialized = false;
bool screenOn = true;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 60000;

TaskHandle_t buttonTaskHandle = NULL;
SemaphoreHandle_t buttonSemaphore;
volatile unsigned long lastPressTime = 0;
volatile int pressCount = 0;

void IRAM_ATTR buttonISR() {
    unsigned long currentTime = millis();
    if (currentTime - lastPressTime > 50) {
        pressCount++;
        lastPressTime = currentTime;
        xSemaphoreGiveFromISR(buttonSemaphore, NULL);
    }
}

void buttonTask(void* pvParameters) {
    unsigned long pressStart = 0;
    int localPressCount = 0;

    while (true) {
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            pressStart = lastPressTime;
            localPressCount = pressCount;

            while (digitalRead(BUTTON_DISPLAY) == LOW) {
                if (millis() - pressStart > 2000) {
                    Serial.println("Long press - Resetting system...");
                    ESP.restart();
                    break;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            vTaskDelay(500 / portTICK_PERIOD_MS);
            if (pressCount == localPressCount) {
                if (localPressCount == 1) {
                    screenOn = !screenOn;
                    display.toggleScreen(screenOn);
                    Serial.println("Single press - Screen toggled");
                } else if (localPressCount == 2) {
                    display.switchDisplayMode();
                    Serial.println("Double press - Switched display mode");
                }
                pressCount = 0;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_DISPLAY, INPUT_PULLUP);

    buttonSemaphore = xSemaphoreCreateBinary();
    attachInterrupt(digitalPinToInterrupt(BUTTON_DISPLAY), buttonISR, FALLING);

    Wire.begin();

    display.begin();
    stepCounter.begin();
    heartRateSpO2.begin();
    ble.begin();
    timeManager.begin();

    stepCounter.startTask();
    heartRateSpO2.startTask();
    ble.startTask();
    timeManager.startTask();
    display.startTask();

    xTaskCreate(
        buttonTask, "ButtonTask", 2048, NULL, 3, &buttonTaskHandle
    );

    Serial.printf("Free heap at start: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
}

void loop() {
    stepCounter.getData(stepCount, distance, ax, ay, az);
    heartRateSpO2.getData(heartRate, spo2, irValue, redValue);
    timeManager.getTime(currentTime, timeInitialized);
    ble.updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected());
    display.updateData(ble.isWifiConnected(), stepCount, distance, heartRate, spo2, &currentTime, timeInitialized);

    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= sendInterval) {
        ble.sendHealthData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected());
        lastSendTime = currentMillis;
    }

    Serial.printf("Free heap: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    vTaskDelay(100 / portTICK_PERIOD_MS);
}