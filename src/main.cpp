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
// ISR Nút bấm (Giữ nguyên logic cũ)
void IRAM_ATTR buttonISR() {
unsigned long currentMillis = millis(); // Đổi tên biến để tránh trùng
if (currentMillis - lastPressTime > 50) { // Debounce 50ms
pressCount++;
lastPressTime = currentMillis;
// Đánh thức ButtonTask
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
if (xHigherPriorityTaskWoken) {
portYIELD_FROM_ISR();
}
}
}
void buttonTask(void* pvParameters) {
unsigned long pressStartTime = 0;
bool heldActionDone = false; // Cờ đánh dấu đã xử lý nhấn giữ (1s hoặc 2s)
while (true) {
    // Chờ tín hiệu nhấn nút từ ISR
    if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
        pressStartTime = millis(); // Ghi lại thời điểm bắt đầu nhấn
        heldActionDone = false;    // Reset cờ xử lý nhấn giữ
        Serial.println("Button Pressed Down");

        // Vòng lặp kiểm tra khi nút còn được giữ
        while (digitalRead(BUTTON_DISPLAY) == LOW) {
            unsigned long heldDuration  = millis() - pressStartTime; // Thời gian đã giữ

            // Ưu tiên kiểm tra nhấn giữ lâu (> 2 giây) để reset
            if (!heldActionDone && heldDuration  >= 2000) {
                Serial.println("Button Held (>2s) - Resetting system...");
                heldActionDone = true; // Đánh dấu đã xử lý
                ESP.restart();
                // Không cần break vì restart sẽ không quay lại
            }
            // Kiểm tra nhấn giữ vừa (1 giây) để bật/tắt màn hình
            else if (!heldActionDone && heldDuration  >= 1000) {
                Serial.println("Button Held (>=1s) - Toggling Screen");
                display.toggleScreen(); // Gọi hàm toggle của DisplayManager
                heldActionDone = true; // Đánh dấu đã xử lý
                // Tiếp tục giữ vòng lặp để kiểm tra xem có thành nhấn giữ 2s không
                // Hoặc có thể break nếu muốn hành động ngay khi đủ 1s
                // break;
            }

            vTaskDelay(50 / portTICK_PERIOD_MS); // Kiểm tra lại sau 50ms
        } // Kết thúc while giữ nút

        // Nút đã được thả ra
        Serial.println("Button Released");

        // Nếu chưa thực hiện hành động nhấn giữ nào (nghĩa là nhấn ngắn)
        if (!heldActionDone) {
            // Kiểm tra thời gian nhấn thực tế để tránh lỗi do debounce ISR
            if (millis() - pressStartTime < 800) { // Coi là nhấn ngắn nếu dưới 800ms
                 Serial.println("Short Press detected - Toggling Theme");
                 display.toggleTheme(); // Gọi hàm đổi theme
             } else {
                 // Nếu thời gian > 800ms nhưng < 1000ms, có thể không làm gì
                 Serial.println("Press duration between short and hold, ignoring.");
             }
        }
        // Reset trạng thái sau khi xử lý xong
        heldActionDone = false;
        // Không cần reset pressCount nữa vì không dùng
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