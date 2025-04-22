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
#include <OneButton.h>

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
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000;
int lastDay = -1;

SemaphoreHandle_t buttonSemaphore;
volatile unsigned long lastPressTime = 0;
volatile int pressCount = 0;
OneButton buttonMode(MODE_BUTTON_PIN, true, true);  // Nút Mode/Theme (Chân 12)
OneButton powerButton(POWER_BUTTON_PIN, true, true); // Nút Nguồn (Chân 14)

// Biến lưu trạng thái nhấn giữ để tránh reboot nhiều lần
volatile bool isPowerLongPressHandled = false;

// ===== CÁC HÀM CALLBACK CHO NÚT MODE/THEME (Chân 12) =====

// Nhấn đơn nút Mode -> Switch Display Mode
void handleModeClick() {
    Serial.println("Mode Button Click() - Switching Display Mode");
    display.switchDisplayMode();
}

// Nhấn giữ nút Mode -> Toggle Theme
void handleModeLongPressStart() {
    Serial.println("Mode Button LongPressStart() - Toggling Theme");
    display.toggleTheme();
    // Có thể thêm phản hồi rung hoặc sáng đèn nhẹ khi bắt đầu giữ
}

// ===== CÁC HÀM CALLBACK CHO NÚT POWER (Chân 14) =====

// Nhấn đơn nút Power -> Toggle Screen
void handlePowerClick() {
    Serial.println("Power Button Click() - Toggling Screen");
    display.toggleScreen();
}

// Bắt đầu nhấn giữ nút Power
void handlePowerLongPressStart() {
    Serial.println("Power Button LongPressStart()");
    isPowerLongPressHandled = false; // Reset cờ xử lý reboot
}

// Khi đang nhấn giữ nút Power
void handlePowerDuringLongPress() {
    // Chỉ reboot MỘT LẦN khi giữ đủ 2 giây
    if (!isPowerLongPressHandled && powerButton.getPressedMs() >= 3000) {
        Serial.println("Power Button DuringLongPress() >= 3s - Rebooting...");
        isPowerLongPressHandled = true; // Đánh dấu đã xử lý
        ESP.restart();
    }
}

void setup() {
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

    // --- GẮN CALLBACK CHO NÚT MODE (Chân 12) ---
    Serial.println("Attaching Mode Button callbacks...");
    buttonMode.attachClick(handleModeClick);             // Nhấn đơn -> Switch Mode
    buttonMode.attachLongPressStart(handleModeLongPressStart); // Nhấn giữ -> Toggle Theme
    buttonMode.setPressMs(1000); // Thời gian giữ để kích hoạt LongPressStart (1 giây)

    // --- GẮN CALLBACK CHO NÚT POWER (Chân 14) ---
    Serial.println("Attaching Power Button callbacks...");
    powerButton.attachClick(handlePowerClick);             // Nhấn đơn -> Toggle Screen
    powerButton.attachLongPressStart(handlePowerLongPressStart); // Bắt đầu giữ
    powerButton.attachDuringLongPress(handlePowerDuringLongPress); // Đang giữ (để reboot)

    stepCount = EEPROM.readInt(STEP_COUNT_ADDR);
    Serial.printf("Restored step count: %d\n", stepCount);
    Serial.printf("Free heap at start: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
}

void loop() {
    stepCounter.getData(stepCount, distance, ax, ay, az, gx, gy, gz);
    heartRateSpO2.getData(heartRate, spo2, irValue, redValue);
    timeManager.getTime(currentTime, timeInitialized);
    bool wifiConnected = ble.isWifiConnected();
    
    buttonMode.tick();
    powerButton.tick();

    char timeStr[40];
    memset(timeStr, 0, sizeof(timeStr));
    if (timeInitialized) {
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%S+07:00", &currentTime);
    } else {
        strcpy(timeStr, "Not initialized");
    }
    
    ble.updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, ble.isWifiConnected(), gx, gy, gz, timeStr);
    display.updateData(wifiConnected, stepCount, distance, heartRate, spo2, &currentTime, timeInitialized);

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

    vTaskDelay(100 / portTICK_PERIOD_MS);
}