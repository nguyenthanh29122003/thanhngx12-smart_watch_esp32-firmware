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
#include "BarometerManager.h" // <<< THÊM INCLUDE BMP280
#include "BatteryManager.h"
#include <OneButton.h>      // <<< Đã có
#include <EEPROM.h>

#define STEP_COUNT_ADDR 0

// --- Mutex ---
SemaphoreHandle_t i2cMutex = NULL; // Khởi tạo là NULL

// --- Khởi tạo Đối tượng Quản lý ---
DisplayManager display;
StepCounter stepCounter;
HeartRateSpO2 heartRateSpO2;
BarometerManager barometer; // <<< THÊM ĐỐI TƯỢNG BMP280
TimeManager timeManager;
BluetoothManager ble(&timeManager);

// --- Biến Global Lưu Dữ liệu Tạm thời ---
int stepCount = 0;
float distance = 0.0f;
int heartRate = 0;
int spo2 = -999; // Khởi tạo giá trị không hợp lệ
float temperature = NAN; // Khởi tạo là Not-a-Number
float pressure = NAN;    // Khởi tạo là Not-a-Number
float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;
long irValue = 0, redValue = 0;
struct tm currentTime;
bool timeInitialized = false;
int lastDay = -1; // Theo dõi ngày để reset bước chân

// --- Khởi tạo OneButton ---
// !!! QUAN TRỌNG: Sửa lại chân GPIO cho đúng với board ESP32-S3 của bạn !!!
// Nếu chỉ có nút BOOT ở GPIO 0:
OneButton button(BUTTON_PIN, true, true); // Giả sử BUTTON_PIN = 0 trong Config.h
BatteryManager batteryManager((gpio_num_t)BATT_ADC_PIN, BATT_VOLTAGE_DIVIDER_RATIO);
// Biến trạng thái cho nhấn giữ reboot
volatile bool isRebootHandled = false;

// --- Hàm Callback cho Nút Bấm (Ví dụ cho 1 nút) ---
void handleClick() {
    Serial.println("Button Click() - Toggling Screen");
    display.toggleScreen();
}

void handleDoubleClick() {
    Serial.println("Button DoubleClick() - Switching Display Mode");
    display.switchDisplayMode();
}

void handleLongPressStart() {
     Serial.println("Button LongPressStart()");
     isRebootHandled = false; // Reset cờ khi bắt đầu nhấn giữ mới
     // Có thể thêm hành động khác ở đây nếu muốn (ví dụ: toggle theme)
     display.toggleTheme();
}

void handleDuringLongPress() {
    // Dùng button.getPressedMs() thay vì powerButton
    if (!isRebootHandled && button.getPressedMs() >= 2000) { // Giữ 2 giây
        Serial.println("Button DuringLongPress() >= 2s - Rebooting...");
        isRebootHandled = true;
        ESP.restart();
    }
}
void handleLongPressStop() {
     Serial.println("Button LongPressStop()");
     isRebootHandled = false; // Reset cờ khi nhả nút
}


// ===================== SETUP =====================
void setup() {
    Serial.begin(921600);
    delay(1000);
    Serial.println("\n\n--- ESP32-S3 Smartwatch Firmware Starting ---");

    // ----> 1. BẬT NGUỒN NGOẠI VI (Quan trọng!) <----
    Serial.println("Enabling external power...");
    #if defined(TFT_I2C_POWER) // Kiểm tra macro từ board definition
        pinMode(TFT_I2C_POWER, OUTPUT);
        digitalWrite(TFT_I2C_POWER, HIGH);
        Serial.printf("Set TFT_I2C_POWER (Pin %d) HIGH\n", TFT_I2C_POWER);
    #else
        Serial.println("Warning: TFT_I2C_POWER pin not defined for this board! Check connections.");
        // Nếu không có, bạn cần tìm chân GPIO đúng và define trong Config.h
        // pinMode(TFT_I2C_POWER_PIN, OUTPUT);
        // digitalWrite(TFT_I2C_POWER_PIN, HIGH);
    #endif
    delay(50); // Chờ nguồn ổn định

    Serial.println("Initializing Battery Manager...");
    if (!batteryManager.begin()) {
        Serial.println("!!! WARNING: Battery Manager initialization failed!");
    }

    // ----> 2. KHỞI TẠO I2C, EEPROM, MUTEX <----
    Wire.begin(); // Dùng chân I2C mặc định của board S3 (Cần xác nhận nếu cần)
    Wire.setClock(50000); // Giữ 50kHz cho ổn định
    Serial.println("I2C Clock set to 50kHz.");
    if (!EEPROM.begin(EEPROM_SIZE)) Serial.println("Failed to initialise EEPROM!");
    else Serial.println("EEPROM initialized.");

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) { Serial.println("CRITICAL: Failed to create I2C Mutex!"); while(1); }
    else Serial.println("I2C Mutex created.");

    // ----> 3. DELAY VÀ QUÉT I2C <----
    Serial.println("Delay before I2C scan (250ms)...");
    delay(250);
    Serial.println("Scanning I2C bus...");
    byte foundCount = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("I2C Device Found at address 0x%02X\n", addr);
            foundCount++;
        }
    }
    Serial.printf("I2C Scan complete. Found %d devices.\n", foundCount);
    // ---> !! SO SÁNH KẾT QUẢ QUÉT VỚI ĐỊA CHỈ TRONG CONFIG.H !! <---

    // ----> 4. KHỞI TẠO CÁC MODULE <----
    Serial.println("Initializing Display...");
    display.begin();

    Serial.println("Initializing Core Modules...");
    bool stepSensorOk = stepCounter.begin();      // Lưu kết quả init
    bool hrSensorOk = heartRateSpO2.begin();    // Lưu kết quả init
    bool baroSensorOk = barometer.begin();        // Lưu kết quả init
    ble.begin();
    timeManager.begin();

    // In trạng thái khởi tạo cảm biến
    Serial.printf("Sensor Init Status: QMI8658C=%s, MAX3010x=%s, BMP280=%s\n",
                  stepSensorOk ? "OK" : "FAIL",
                  hrSensorOk ? "OK" : "FAIL",
                  baroSensorOk ? "OK" : "FAIL");

    // ----> 5. GẮN CALLBACK CHO NÚT BẤM <----
    Serial.println("Attaching Button callbacks (using OneButton)...");
    // Gắn cho 1 nút (button)
    button.attachClick(handleClick);                 // Nhấn đơn -> Toggle Screen
    button.attachDoubleClick(handleDoubleClick);       // Nhấn đúp -> Switch Mode
    button.attachLongPressStart(handleLongPressStart); // Bắt đầu nhấn giữ
    button.attachDuringLongPress(handleDuringLongPress); // Đang nhấn giữ -> Reboot
    button.attachLongPressStop(handleLongPressStop);     // Nhả nút sau khi giữ
    button.setDebounceMs(50);       // Thời gian debounce
    button.setClickMs(400);         // Thời gian tối đa cho 1 click (trước khi thành giữ)
    button.setPressMs(1000);        // Thời gian giữ để kích hoạt LongPressStart/DuringLongPress
    // Nếu dùng 2 nút thì gắn callback tương ứng cho buttonMode và powerButton

    // ----> 6. KHỞI ĐỘNG CÁC TASK <----
    Serial.println("Starting Tasks...");
    if (hrSensorOk) heartRateSpO2.startTask(TASK_PRIORITY_HEART_RATE); // Chỉ start nếu init OK
    if (stepSensorOk) stepCounter.startTask(TASK_PRIORITY_STEP); // Chỉ start nếu init OK
    if (baroSensorOk) barometer.startTask(1); // Ưu tiên thấp cho barometer
    ble.startTask(1);             // Ưu tiên thấp/trung bình cho BLE
    timeManager.startTask();
    display.startTask(1);           // Ưu tiên thấp/trung bình cho display

    batteryManager.startTask(1);

    Serial.printf("Free heap at start: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.println("--- Setup Complete ---");
}

// ===================== LOOP =====================
void loop() {
    // 1. Luôn gọi tick() cho các nút bấm
    button.tick();
    // buttonMode.tick(); // Nếu có nút thứ 2
    // powerButton.tick(); // Nếu có nút thứ 2

    // 2. Lấy dữ liệu từ các module
    stepCounter.getData(stepCount, distance, ax, ay, az, gx, gy, gz);
    heartRateSpO2.getData(heartRate, spo2, irValue, redValue);
    barometer.getData(temperature, pressure); // <<< LẤY DỮ LIỆU BMP280
    timeManager.getTime(currentTime, timeInitialized);

    // 3. Lấy trạng thái WiFi
    bool wifiConnected = ble.isWifiConnected(); // Hoặc WiFi.status() == WL_CONNECTED;

    // 4. Cập nhật DisplayManager với TẤT CẢ dữ liệu
    display.updateData(wifiConnected, stepCount, distance, heartRate, spo2,
                       temperature, pressure, // <<< TRUYỀN DỮ LIỆU BMP
                       ax, ay, az, gx, gy, gz, // <<< TRUYỀN DỮ LIỆU IMU
                       &currentTime, timeInitialized);

    // 5. Chuẩn bị và gửi dữ liệu qua BLE
    char timeStr[40];
    memset(timeStr, 0, sizeof(timeStr));
    if (timeInitialized) strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%S+07:00", &currentTime);
    else strcpy(timeStr, "Not initialized");
    // Gọi hàm updateData của BLE với đủ tham số
    ble.updateData(ax, ay, az, stepCount, heartRate, spo2, irValue, redValue, wifiConnected, gx, gy, gz,
                   temperature, pressure, // <<< TRUYỀN DỮ LIỆU BMP
                   timeStr);

    // 6. Logic reset bước chân hàng ngày (SỬA LẠI)
    if (timeInitialized) {
        int currentDay = currentTime.tm_mday;
        if (lastDay != -1 && currentDay != lastDay) { // Chỉ cần kiểm tra ngày thay đổi
            // Kiểm tra thêm giờ để tránh reset nhiều lần nếu bị treo lúc nửa đêm
            if (currentTime.tm_hour == 0 && currentTime.tm_min <= 5) { // Reset trong 5 phút đầu ngày mới
                 Serial.println("New day detected, calling stepCounter.resetSteps()...");
                 stepCounter.resetSteps(); // <<< GỌI HÀM RESET CỦA MODULE
                 // Không cần reset biến stepCount/distance ở đây nữa
                 lastDay = currentDay; // Cập nhật ngày ngay sau khi reset
            }
        } else if (lastDay == -1) {
            lastDay = currentDay; // Khởi tạo lastDay lần đầu
        }
    }

    // 7. Delay cho vòng lặp chính
    vTaskDelay(pdMS_TO_TICKS(20)); // Giữ delay ngắn để OneButton nhạy
}  

// #include <Wire.h>

// #include "MAX30105.h"  //Get it here: http://librarymanager/All#SparkFun_MAX30105
// MAX30105 particleSensor;

// void setup()
// {
//   Serial.begin(9600);
//   Serial.println("Initializing...");

//   // Initialize sensor
//   if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
//   {
//     Serial.println("MAX30105 was not found. Please check wiring/power. ");
//     while (1);
//   }

//   //The LEDs are very low power and won't affect the temp reading much but
//   //you may want to turn off the LEDs to avoid any local heating
//   particleSensor.setup(0); //Configure sensor. Turn off LEDs
//   //particleSensor.setup(); //Configure sensor. Use 25mA for LED drive

//   particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
// }

// void loop()
// {
//   float temperature = particleSensor.readTemperature();

//   Serial.print("temperatureC=");
//   Serial.print(temperature, 4);

//   float temperatureF = particleSensor.readTemperatureF(); //Because I am a bad global citizen

//   Serial.print(" temperatureF=");
//   Serial.print(temperatureF, 4);

//   Serial.println();
// }