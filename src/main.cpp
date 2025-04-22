// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Config.h"
#include "DisplayManager.h"
#include "StepCounter.h"
#include "HeartRateSpO2.h"
#include "BluetoothManager.h"
#include "TimeManager.h"

#define STEP_COUNT_ADDR 0

SemaphoreHandle_t i2cMutex;

DisplayManager display;
StepCounter stepCounter;
HeartRateSpO2 heartRateSpO2;
TimeManager timeManager;
BluetoothManager ble(&timeManager);

int stepCount = 0;
float distance = 0;
int heartRate = 0;
int spo2 = 0;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
long irValue = 0, redValue = 0;
struct tm currentTime;
bool timeInitialized = false;
bool screenOn = true;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000;
int lastDay = -1;

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
                    display.toggleScreen();
                    Serial.println("Single press - Screen toggled");
                } else if (localPressCount == 2) {
                    display.toggleTheme();
                    Serial.println("Double press - Switched display mode");
                }
                pressCount = 0;
            }
        }
    }
}

void setup() {
    pinMode(BUTTON_DISPLAY, INPUT_PULLUP);
    buttonSemaphore = xSemaphoreCreateBinary();
    attachInterrupt(digitalPinToInterrupt(BUTTON_DISPLAY), buttonISR, FALLING);

    Serial.begin(921600); // Tăng baud rate
    while (!Serial) delay(10);
    delay(1000); // Đợi Serial Monitor sẵn sàng
    Serial.println("System starting...");

    Wire.begin();
    EEPROM.begin(512);
    Wire.setClock(50000); // I2C 50kHz

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("Failed to create I2C mutex!");
        while (1);
    }

    Serial.println("Scanning I2C bus...");
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found device at 0x%02X\n", addr);
        }
    }

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

    xTaskCreate(buttonTask, "ButtonTask", 2048, NULL, 3, &buttonTaskHandle);

    stepCount = EEPROM.readInt(STEP_COUNT_ADDR);
    Serial.printf("Restored step count: %d\n", stepCount);
    Serial.printf("Free heap at start: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
}

void loop() {
// 1. Lấy dữ liệu từ các module cảm biến và thời gian
stepCounter.getData(stepCount, distance, ax, ay, az, gx, gy, gz);
heartRateSpO2.getData(heartRate, spo2, irValue, redValue);
timeManager.getTime(currentTime, timeInitialized);
// 2. Lấy trạng thái kết nối WiFi
bool wifiConnected = ble.isWifiConnected(); // Hoặc từ TimeManager nếu nó quản lý WiFi

// 3. Cập nhật dữ liệu cho DisplayManager
// Truyền tất cả dữ liệu cần thiết cho các chế độ hiển thị
// display.updateData(wifiConnected, stepCount, distance, heartRate, spo2, &currentTime, timeInitialized);
display.updateData(wifiConnected, &currentTime, timeInitialized);

// 4. Chuẩn bị và gửi dữ liệu qua BLE
char timeStr[40];
memset(timeStr, 0, sizeof(timeStr));
if (timeInitialized) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%S+07:00", &currentTime);
} else {
    strcpy(timeStr, "Not initialized");
}
// Gửi tất cả dữ liệu thu thập được
ble.updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, wifiConnected, gx, gy, gz, timeStr);

// 5. Logic reset bước chân hàng ngày (Giữ nguyên)
if (timeInitialized) {
    int currentDay = currentTime.tm_mday;
    if (lastDay != -1 && currentDay != lastDay && currentTime.tm_hour == 0 && currentTime.tm_min <= 1) {
         Serial.println("New day detected, attempting to reset step count...");
         // Nên có cơ chế reset trong StepCounter thay vì ghi trực tiếp ở đây
         // Tạm thời giữ lại:
         EEPROM.writeInt(STEP_COUNT_ADDR, 0);
         if (EEPROM.commit()) {
              Serial.println("Step count reset to 0 in EEPROM.");
              // Cập nhật biến local để hiển thị đúng ngay lập tức
              stepCount = 0;
              distance = 0;
         } else {
              Serial.println("Failed to commit EEPROM for step count reset!");
         }
         // TODO: Thông báo cho StepCounter để reset stepCountLocal của nó
    }
    lastDay = currentDay;
}

    vTaskDelay(100 / portTICK_PERIOD_MS);
}