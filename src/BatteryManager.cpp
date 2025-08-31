// src/BatteryManager.cpp
#include "BatteryManager.h"
#include "Config.h"   // Có thể cần cho các cấu hình khác
#include <Arduino.h>  // Cho delay, millis, vTaskDelay, Serial
#include <cmath>      // Cho isnan, constrain

// Constructor
BatteryManager::BatteryManager(gpio_num_t adcPin, float dividerRatio, adc1_channel_t adcChannel)
    : _adcPin(adcPin), _adcChannel(adcChannel), _dividerRatio(dividerRatio),
      _adcInitialized(false), taskHandle(NULL), dataMutex(NULL),
      voltageLocal(NAN), percentageLocal(-1)
{
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("CRITICAL: Failed to create BatteryManager data mutex!");
    }

    // Tự động xác định kênh ADC nếu không được cung cấp (chỉ hoạt động với ADC1)
    if (_adcChannel == ADC1_CHANNEL_MAX) {
        for (int i = 0; i < ADC1_CHANNEL_MAX; ++i) {
            gpio_num_t pin;
            if (adc1_pad_get_io_num((adc1_channel_t)i, &pin) == ESP_OK) {
                if (pin == _adcPin) {
                    _adcChannel = (adc1_channel_t)i;
                    Serial.printf("BatteryManager: ADC Pin %d mapped to ADC1 Channel %d\n", _adcPin, _adcChannel);
                    break;
                }
            }
        }
        if (_adcChannel == ADC1_CHANNEL_MAX) {
             Serial.printf("ERROR: Could not map GPIO %d to an ADC1 channel!\n", _adcPin);
             // Có thể cần xử lý lỗi này, ví dụ không khởi tạo ADC
        }
    }
     // ESP32-S3: GPIO 1-10 là ADC1, 11-20 là ADC2. Cần xử lý ADC2 nếu dùng các chân đó.
     // Code này hiện chỉ hỗ trợ ADC1.
}

// Hàm cấu hình ADC
bool BatteryManager::configureADC() {
    if (_adcChannel == ADC1_CHANNEL_MAX) return false; // Không cấu hình nếu không tìm thấy kênh

    esp_err_t config_result = adc1_config_width(ADC_WIDTH_BIT_12); // Độ phân giải 12 bit
    if(config_result != ESP_OK){
        Serial.printf("ERROR: Failed to configure ADC1 width! (%s)\n", esp_err_to_name(config_result));
        return false;
    }

    // Chọn Attenuation 11dB để đo được dải điện áp 1.5V - 2.1V (tại chân ADC)
    config_result = adc1_config_channel_atten(_adcChannel, ADC_ATTEN_DB_12);
     if(config_result != ESP_OK){
        Serial.printf("ERROR: Failed to configure ADC1 attenuation for channel %d! (%s)\n", _adcChannel, esp_err_to_name(config_result));
        return false;
    }

    // --- Tùy chọn: Sử dụng hiệu chuẩn ADC (Khuyến nghị) ---
    // Cần #include "esp_adc_cal.h"
    // static esp_adc_cal_characteristics_t adc_chars;
    // esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    // // Kiểm tra kiểu hiệu chuẩn
    // if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    //     Serial.printf("ADC Calibration: eFuse Vref used: %d mV\n", adc_chars.vref);
    // } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    //     Serial.println("ADC Calibration: Two Point values used");
    // } else {
    //     Serial.println("ADC Calibration: Default Vref used");
    // }
    // ------------------------------------------------------

    _adcInitialized = true;
    Serial.println("ADC1 configured for battery measurement.");
    return true;
}


// Hàm begin()
bool BatteryManager::begin() {
    Serial.println("Initializing Battery Manager...");
    return configureADC(); // Thực hiện cấu hình ADC
}

// --- Quản lý Task ---
void BatteryManager::startTask(UBaseType_t priority) {
    if (!_adcInitialized) {
        Serial.println("ADC not initialized, cannot start Battery Task.");
        return;
    }
    xTaskCreate(
        taskFunction,
        "BatteryTask",   // Tên task
        2048,            // Stack size
        this,            // Tham số
        priority,        // Ưu tiên
        &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating Battery Task!");
    else Serial.println("Battery Task started.");
}

void BatteryManager::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle;
        taskHandle = NULL;
        vTaskDelete(tempHandle);
        Serial.println("Battery task stopped.");
    }
}

// --- Task Function ---
void BatteryManager::taskFunction(void* pvParameters) {
    BatteryManager* instance = static_cast<BatteryManager*>(pvParameters);
    if (instance == nullptr) { vTaskDelete(NULL); return; }

    Serial.println("Battery Task running...");
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(15000); // Đọc mỗi 15 giây

    while (true) {
        instance->readAndUpdate();
        // Delay chính xác sử dụng vTaskDelayUntil
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

// --- Đọc và Cập nhật ---
void BatteryManager::readAndUpdate() {
    if (!_adcInitialized) return;

    float voltage = readRawVoltage(); // Đọc điện áp tại chân ADC
    if (isnan(voltage)) {
         Serial.println("Warning: Failed to read ADC voltage.");
         // Không cập nhật nếu đọc lỗi
         return;
    }

    // Tính điện áp pin thực tế
    float batteryVoltage = voltage * _dividerRatio;
    // Tính phần trăm pin
    int percentage = calculatePercentage(batteryVoltage);

    // Cập nhật giá trị local an toàn
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        voltageLocal = batteryVoltage;
        percentageLocal = percentage;
        xSemaphoreGive(dataMutex);
        // Debug
        // Serial.printf("Battery Update: %.2f V, %d %%\n", voltageLocal, percentageLocal);
    } else {
        Serial.println("Timeout taking Battery data mutex in readAndUpdate!");
    }
}

// --- Đọc Điện áp Thô từ ADC ---
float BatteryManager::readRawVoltage() {
    if (!_adcInitialized) return NAN;

    uint32_t adc_reading = 0;
    for (int i = 0; i < NUM_ADC_SAMPLES; i++) {
        int raw = adc1_get_raw(_adcChannel);
        if (raw == -1) { // Lỗi đọc ADC1
             Serial.printf("ERROR: ADC1 read failed on channel %d\n", _adcChannel);
             return NAN;
        }
        adc_reading += raw;
        vTaskDelay(pdMS_TO_TICKS(1)); // Delay nhỏ
    }
    adc_reading /= NUM_ADC_SAMPLES;

    // --- Chuyển đổi Raw sang Voltage (mV) - Ước tính, CẦN HIỆU CHUẨN ---
    // Sử dụng hàm hiệu chuẩn nếu có (xem comment trong configureADC)
    // uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);

    // Ước tính tuyến tính tạm thời (giả sử Vref ~ 3.1V cho Atten 11dB)
    // Giá trị này RẤT không chính xác, đặc biệt khi có WiFi
    float voltage_mv = (float)adc_reading / 4095.0 * 3100.0; // Giả sử Full Scale là 3100mV

    // Có thể cần hiệu chỉnh tuyến tính đơn giản nếu bạn đo được giá trị thực tế
    // voltage_mv = voltage_mv * CALIBRATION_FACTOR + OFFSET;

    return voltage_mv / 1000.0; // Trả về Volt
}

// --- Tính Phần trăm Pin ---
int BatteryManager::calculatePercentage(float voltage) {
    if (isnan(voltage)) return -1;
    // Sử dụng ngưỡng min/max đã định nghĩa
    if (voltage >= BATT_MAX_V) return 100;
    if (voltage <= BATT_MIN_V) return 0;
    // Ánh xạ tuyến tính
    int percentage = (int)(((voltage - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V)) * 100.0);
    return constrain(percentage, 0, 100);
}

// --- Hàm getData() ---
float BatteryManager::getVoltage() {
    float v = NAN;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        v = voltageLocal;
        xSemaphoreGive(dataMutex);
    }
    return v;
}

int BatteryManager::getPercentage() {
    int p = -1;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        p = percentageLocal;
        xSemaphoreGive(dataMutex);
    }
    return p;
}

void BatteryManager::getData(float& voltage, int& percentage) {
     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        voltage = voltageLocal;
        percentage = percentageLocal;
        xSemaphoreGive(dataMutex);
    } else {
        voltage = NAN; percentage = -1;
        Serial.println("CRITICAL: Failed to take Battery data mutex in getData!");
    }
}