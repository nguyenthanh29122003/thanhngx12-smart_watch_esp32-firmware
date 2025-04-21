// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Config.h"       // Đảm bảo Config.h có I2C_MUTEX_TIMEOUT_MS
#include "DisplayManager.h"
#include "StepCounter.h"    // Đảm bảo StepCounter.cpp đã dùng i2cMutex
#include "HeartRateSpO2.h"  // Đảm bảo HeartRateSpO2.cpp đã dùng i2cMutex
#include "BluetoothManager.h"
#include "TimeManager.h"
#define STEP_COUNT_ADDR 0
// Định nghĩa Mutex I2C toàn cục
SemaphoreHandle_t i2cMutex = NULL; // Khởi tạo là NULL
// Khởi tạo các đối tượng quản lý
DisplayManager display;
StepCounter stepCounter;
HeartRateSpO2 heartRateSpO2;
TimeManager timeManager;
BluetoothManager ble(&timeManager); // Truyền TimeManager vào BLE
// Biến global để lưu dữ liệu tạm thời
int stepCount = 0;
float distance = 0;
int heartRate = 0;
int spo2 = 0;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
long irValue = 0, redValue = 0;
struct tm currentTime;
bool timeInitialized = false;
// bool screenOn = true; // <-- Biến này không cần ở đây nữa, DisplayManager tự quản lý
int lastDay = -1;
// unsigned long lastSendTime = 0; // Không thấy dùng
// const unsigned long sendInterval = 1000; // Không thấy dùng
// Phần xử lý nút bấm
TaskHandle_t buttonTaskHandle = NULL;
SemaphoreHandle_t buttonSemaphore;
volatile unsigned long lastPressTime = 0;
volatile int pressCount = 0; // Biến đếm số lần nhấn từ ISR

// ISR Nút bấm
void IRAM_ATTR buttonISR() {
    // Chỉ cần đánh thức task xử lý
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void buttonTask(void* pvParameters) {
    unsigned long pressStartTime = 0;
    unsigned long releaseTime = 0;
    unsigned long lastReleaseTime = 0; // Lưu thời điểm nhả nút của lần nhấn TRƯỚC ĐÓ
    const int debounceTime = 50;      // Thời gian debounce (ms)
    const int doublePressMaxInterval = 400; // Khoảng thời gian tối đa giữa 2 lần NHẤN để tính là double press
    const int shortPressMaxDuration = 800; // Thời gian nhấn tối đa cho single/double press
    const int screenToggleHoldDuration = 1000; // Thời gian giữ để bật/tắt màn hình
    const int rebootHoldDuration = 2000;    // Thời gian giữ để reboot

    while (true) {
        // Chờ tín hiệu nhấn nút từ ISR
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            // --- Debounce khi nhấn xuống ---
            vTaskDelay(pdMS_TO_TICKS(debounceTime));
            // Kiểm tra lại xem nút có còn nhấn không (tránh nhiễu)
            if (digitalRead(BUTTON_DISPLAY) == HIGH) {
                Serial.println("Button Bounce Ignored (Down)");
                continue; // Bỏ qua nếu là nhiễu
            }

            pressStartTime = millis(); // Ghi thời điểm sau debounce
            Serial.println("Button Pressed");

            // --- Chờ nhả nút ---
            while (digitalRead(BUTTON_DISPLAY) == LOW) {
                vTaskDelay(pdMS_TO_TICKS(20)); // Chờ nút nhả ra
            }
            releaseTime = millis(); // Ghi thời điểm nhả nút
             // --- Debounce khi nhả lên ---
             vTaskDelay(pdMS_TO_TICKS(debounceTime));
             // Kiểm tra lại xem nút có bị nhấn lại ngay không (tránh nhiễu)
             if(digitalRead(BUTTON_DISPLAY) == LOW){
                  Serial.println("Button Bounce Ignored (Up)");
                  continue; // Bỏ qua nếu là nhiễu
             }

            Serial.println("Button Released");

            // --- Phân tích hành động ---
            unsigned long pressDuration = releaseTime - pressStartTime;
            unsigned long intervalSinceLastRelease = pressStartTime - lastReleaseTime; // Thời gian từ lần nhả trước đến lần nhấn này

            Serial.printf("Press Duration: %lu ms, Interval: %lu ms\n", pressDuration, intervalSinceLastRelease);

            // 1. Ưu tiên kiểm tra Reboot (Nhấn giữ lâu nhất)
            if (pressDuration >= rebootHoldDuration) {
                Serial.println("Action: Long Press (>2s) - Rebooting");
                ESP.restart();
            }
            // 2. Kiểm tra Screen Toggle (Nhấn giữ vừa)
            else if (pressDuration >= screenToggleHoldDuration) {
                Serial.println("Action: Hold (>=1s) - Toggle Screen");
                display.toggleScreen();
            }
            // 3. Kiểm tra Double Press
            // Điều kiện: lần nhấn này xảy ra NHANH sau lần nhả trước ĐÓ (< doublePressMaxInterval)
            // VÀ thời gian nhấn của lần này là NGẮN (< shortPressMaxDuration)
            else if (intervalSinceLastRelease <= doublePressMaxInterval && pressDuration < shortPressMaxDuration) {
                 Serial.println("Action: Double Press - Toggle Theme");
                 display.toggleTheme();
                 // Reset lastReleaseTime để tránh lần nhấn tiếp theo bị coi là double press
                 lastReleaseTime = 0; // Quan trọng
            }
            // 4. Kiểm tra Single Press
            // Điều kiện: KHÔNG phải double press (tức là interval lớn)
            // VÀ thời gian nhấn là NGẮN (< shortPressMaxDuration)
            else if (intervalSinceLastRelease > doublePressMaxInterval && pressDuration < shortPressMaxDuration) {
                 Serial.println("Action: Single Press - Toggle Screen");
                 // Yêu cầu mới: Nhấn 1 lần -> Toggle Screen
                 display.toggleScreen();
            }
            // Các trường hợp khác (nhấn quá ngắn, hoặc nhấn giữ không đủ lâu) -> Bỏ qua
            else {
                 Serial.println("Action: Ignored (press too short or invalid timing)");
            }

            // Lưu lại thời điểm nhả nút cho lần xử lý tiếp theo
            // Chỉ lưu nếu không phải là double press (vì double press đã reset nó)
            if (lastReleaseTime != 0 || intervalSinceLastRelease > doublePressMaxInterval) {
                 lastReleaseTime = releaseTime;
            }

        } // Kết thúc if xSemaphoreTake
    } // Kết thúc while(true)
}

// ===== HÀM SETUP ĐÃ SỬA =====

void setup() {
pinMode(BUTTON_DISPLAY, INPUT_PULLUP);
buttonSemaphore = xSemaphoreCreateBinary();
if (buttonSemaphore == NULL) {
Serial.println("CRITICAL: Failed to create button semaphore!"); while(1);
}
attachInterrupt(digitalPinToInterrupt(BUTTON_DISPLAY), buttonISR, FALLING);
Serial.begin(921600); // Giữ tốc độ cao nếu ổn định
while (!Serial) delay(10);
delay(1000); // Đợi Serial
Serial.println("\n\n--- System Starting (Multi-Screen Version) ---");

// ----> THÊM DELAY ỔN ĐỊNH BAN ĐẦU <----
Serial.println("Initial stabilization delay (500ms)...");
delay(500);

// ----> KHỞI TẠO I2C VÀ EEPROM <----
Wire.begin();
Wire.setClock(50000); // Giữ tốc độ I2C 50kHz
Serial.println("I2C Clock set to 50kHz.");
if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialise EEPROM!");
} else {
    Serial.println("EEPROM initialized.");
}

// ----> THÊM DELAY TRƯỚC KHI INIT I2C DEVICES <----
Serial.println("Delay before I2C init (250ms)...");
delay(250);

// ----> TẠO I2C MUTEX <----
i2cMutex = xSemaphoreCreateMutex();
if (i2cMutex == NULL) {
    Serial.println("CRITICAL: Failed to create I2C Mutex!");
    while(1);
} else {
     Serial.println("I2C Mutex created.");
}

Serial.println("Scanning I2C bus...");
byte foundCount = 0;
for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
        Serial.printf("Found device at 0x%02X\n", addr);
        foundCount++;
    }
}
Serial.printf("I2C Scan complete. Found %d devices.\n", foundCount);


// ----> KHỞI TẠO CÁC MODULE <----
Serial.println("Initializing Display...");
display.begin(); // DisplayManager Init

// Init cảm biến với mutex
if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) { // Chờ vô hạn khi khởi tạo
     Serial.println("Initializing StepCounter (MPU6050)...");
     stepCounter.begin(); // Đảm bảo hàm này dùng mutex bên trong nếu cần
     Serial.println("Initializing HeartRateSpO2 (MAX3010x)...");
     heartRateSpO2.begin(); // Đảm bảo hàm này dùng mutex bên trong nếu cần
     xSemaphoreGive(i2cMutex);
     Serial.println("Sensor initialization complete.");
} else {
      Serial.println("CRITICAL: Failed to take I2C Mutex during sensor init!");
      // Nên dừng lại ở đây nếu không khởi tạo được cảm biến
      while(1);
}

Serial.println("Initializing Bluetooth...");
ble.begin();

Serial.println("Initializing TimeManager...");
timeManager.begin();


// ----> KHỞI ĐỘNG CÁC TASK <----
Serial.println("Starting Tasks...");
stepCounter.startTask();
heartRateSpO2.startTask(); // Đảm bảo task này được bật
ble.startTask();
timeManager.startTask();
display.startTask(); // Task cho DisplayManager

// Tạo Button Task với độ ưu tiên cao
xTaskCreate(buttonTask, "ButtonTask", 2048, NULL, 3, &buttonTaskHandle);
if (buttonTaskHandle == NULL) {
    Serial.println("Error creating Button Task!");
}

// Khôi phục bước chân (StepCounter::begin đã làm, nhưng đọc lại ở đây cũng không sao)
stepCount = EEPROM.readInt(STEP_COUNT_ADDR);
Serial.printf("Restored step count (in setup): %d\n", stepCount);


Serial.printf("Free heap at start: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
Serial.println("--- Setup Complete ---");

}
// ===== HÀM LOOP ĐÃ SỬA =====
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

// 6. Delay cho vòng lặp chính
vTaskDelay(100 / portTICK_PERIOD_MS); // Tần suất cập nhật chung khoảng 10Hz

}