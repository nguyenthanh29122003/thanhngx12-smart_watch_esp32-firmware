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
    if (currentTime - lastPressTime > 50) { // Debounce 50ms
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
    pinMode(BUTTON_DISPLAY, INPUT_PULLUP);
    buttonSemaphore = xSemaphoreCreateBinary();
    attachInterrupt(digitalPinToInterrupt(BUTTON_DISPLAY), buttonISR, FALLING);

    Serial.begin(115200);
    Wire.begin();
    EEPROM.begin(512); // Khởi tạo EEPROM với kích thước tối thiểu

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

    // Khôi phục stepCount từ EEPROM
    stepCount = EEPROM.readInt(STEP_COUNT_ADDR);
    Serial.printf("Restored step count: %d\n", stepCount);
    Serial.printf("Free heap at start: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
}

void loop() {
    stepCounter.getData(stepCount, distance, ax, ay, az, gx, gy, gz);
    heartRateSpO2.getData(heartRate, spo2, irValue, redValue);
    timeManager.getTime(currentTime, timeInitialized);
    ble.updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected(), gx, gy, gz);
    display.updateData(ble.isWifiConnected(), stepCount, distance, heartRate, spo2, &currentTime, timeInitialized); // Sửa lỗi cú pháp &currentTime

    // Kiểm tra ngày mới và reset bước chân
    if (timeInitialized) {
        int currentDay = currentTime.tm_mday;
        if (lastDay != -1 && currentDay != lastDay && currentTime.tm_hour == 0 && currentTime.tm_min == 0) {
            stepCount = 0;
            distance = 0;
            EEPROM.writeInt(STEP_COUNT_ADDR, 0);
            EEPROM.commit();
            Serial.println("New day detected, step count reset to 0");
        }
        lastDay = currentDay;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= sendInterval) {
        ble.sendHealthData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected());
        lastSendTime = currentMillis;
    }

    Serial.printf("Free heap: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    vTaskDelay(100 / portTICK_PERIOD_MS);
}