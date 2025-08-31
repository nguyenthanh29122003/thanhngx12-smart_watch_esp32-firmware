// src/BarometerManager.cpp
#include "BarometerManager.h"
#include "Config.h"   // Cần cho địa chỉ I2C và timeout mutex
#include <Arduino.h>
#include <Wire.h>     // Cần cho TwoWire

// Khai báo Mutex I2C toàn cục
extern SemaphoreHandle_t i2cMutex;

// Constructor
BarometerManager::BarometerManager()
    : bmp(), _wire(nullptr), // Khởi tạo con trỏ Wire là nullptr
      taskHandle(NULL), dataMutex(NULL),
      temperatureLocal(NAN), pressureLocal(NAN), // Khởi tạo là Not-a-Number
      sensorReady(false)
{
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("CRITICAL: Failed to create Barometer data mutex!");
    }
}

// Hàm begin() - Khởi tạo cảm biến BMP280
bool BarometerManager::begin(TwoWire &wireInstance) {
    sensorReady = false;
    _wire = &wireInstance; // Lưu lại đối tượng Wire được sử dụng

    // Lấy Mutex I2C
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS * 2)) == pdTRUE) {
        Serial.println("Initializing BMP280...");

        // Khởi tạo BMP280 với địa chỉ và đối tượng Wire cụ thể
        // Thư viện Adafruit cần begin với địa chỉ
        if (!bmp.begin(BMP280_ADDRESS)) {
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.printf("Could not find a valid BMP280 sensor at 0x%02X!\n", BMP280_ADDRESS);
            Serial.println("Check wiring, I2C address.");
            Serial.println("Barometer features will be disabled.");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        } else {
            Serial.println("BMP280 Found and Initialized!");
            sensorReady = true;

            // --- Cấu hình các tham số đo (Tùy chọn nhưng khuyến nghị) ---
            /* Chế độ hoạt động:
             * - MODE_SLEEP
             * - MODE_FORCED (Đo một lần rồi sleep)
             * - MODE_NORMAL (Đo liên tục)
             * Oversampling (Độ phân giải/Nhiễu vs Thời gian đo):
             * - SAMPLING_NONE
             * - SAMPLING_X1, SAMPLING_X2, SAMPLING_X4, SAMPLING_X8, SAMPLING_X16
             * Filter (Bộ lọc IIR để giảm nhiễu ngắn hạn):
             * - FILTER_OFF
             * - FILTER_X2, FILTER_X4, FILTER_X8, FILTER_X16
             * Standby Time (Thời gian nghỉ giữa các lần đo ở MODE_NORMAL):
             * - STANDBY_MS_0_5, STANDBY_MS_62_5, ..., STANDBY_MS_4000
             */
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Chế độ hoạt động */
                            Adafruit_BMP280::SAMPLING_X4,     /* Oversampling nhiệt độ */
                            Adafruit_BMP280::SAMPLING_X16,    /* Oversampling áp suất */
                            Adafruit_BMP280::FILTER_X4,       /* Bộ lọc IIR */
                            Adafruit_BMP280::STANDBY_MS_500); /* Thời gian nghỉ (ms) */
             Serial.println("BMP280 configured.");
        }
        xSemaphoreGive(i2cMutex); // Trả mutex
    } else {
        Serial.println("CRITICAL: Timeout waiting for I2C mutex during BMP280 init!");
    }
    return sensorReady;
}

// Hàm startTask()
void BarometerManager::startTask(UBaseType_t priority) {
    if (!sensorReady) {
        Serial.println("BMP280 not ready, skipping Barometer Task creation.");
        return;
    }
    xTaskCreate(
        taskFunction,
        "BarometerTask", // Tên task
        2560,            // Stack size (2.5K có thể đủ)
        this,            // Tham số
        priority,        // Ưu tiên (nên thấp, ví dụ 1)
        &taskHandle);
    if (taskHandle == NULL) {
        Serial.println("CRITICAL: Error creating Barometer Task!");
    } else {
        Serial.println("Barometer Task started.");
    }
}

// Hàm stopTask()
void BarometerManager::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle;
        taskHandle = NULL;
        vTaskDelete(tempHandle);
        Serial.println("Barometer task stopped.");
        // Đưa BMP280 về chế độ sleep khi dừng task
        if (sensorReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
             bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
             xSemaphoreGive(i2cMutex);
             Serial.println("BMP280 set to sleep mode.");
        }
    }
}

// Hàm taskFunction()
void BarometerManager::taskFunction(void* pvParameters) {
    BarometerManager* instance = static_cast<BarometerManager*>(pvParameters);
     if (instance == nullptr) { vTaskDelete(NULL); return; }

    Serial.println("Barometer Task running...");
    while (true) {
        instance->updateSensor();
        // Tần suất cập nhật áp suất/nhiệt độ không cần quá cao
        vTaskDelay(pdMS_TO_TICKS(5000)); // Ví dụ: 5 giây một lần
    }
}

// Hàm updateSensor() - Đọc dữ liệu
void BarometerManager::updateSensor() {
    if (!sensorReady) return;

    float temp = NAN;
    float pres = NAN; // Dùng float cho áp suất (thư viện trả về float)
    bool readSuccess = false;

    // Lấy Mutex I2C
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Đọc nhiệt độ và áp suất từ thư viện Adafruit
        temp = bmp.readTemperature();
        pres = bmp.readPressure(); // Trả về Pascals (Pa)

        // Kiểm tra giá trị NAN
        if (!isnan(temp) && !isnan(pres)) {
            readSuccess = true;
        } else {
            Serial.println("Warning: BMP280 read returned NAN.");
            // Thử khởi tạo lại nếu lỗi liên tục?
             // static int failCount = 0; if (++failCount > 5) sensorReady = false;
        }
        xSemaphoreGive(i2cMutex); // Trả mutex
    } else {
        Serial.println("Timeout waiting for I2C mutex in Barometer updateSensor!");
        return;
    }

    // Cập nhật biến cục bộ nếu đọc thành công
    if (readSuccess) {
        // failCount = 0; // Reset bộ đếm lỗi
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            temperatureLocal = temp;
            pressureLocal = pres;
            xSemaphoreGive(dataMutex);
            // Debug định kỳ
            static unsigned long lastBaroDebugTime = 0;
            if (millis() - lastBaroDebugTime > 10000) { // In mỗi 10 giây
                 Serial.printf("BMP280 - Temp: %.2f C, Pres: %.0f Pa\n", temperatureLocal, pressureLocal);
                 lastBaroDebugTime = millis();
            }
        } else {
            Serial.println("Timeout taking Barometer data mutex!");
        }
    }
}

// Hàm getData()
void BarometerManager::getData(float& temperature, float& pressure) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        temperature = temperatureLocal;
        pressure = pressureLocal; // Trả về float
        xSemaphoreGive(dataMutex);
    } else {
        temperature = NAN; pressure = NAN;
        Serial.println("CRITICAL: Failed to take Barometer data mutex in getData!");
    }
}