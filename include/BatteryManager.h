// include/BatteryManager.h
#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/adc.h> // Cần cho các kiểu ADC

class BatteryManager {
public:
    BatteryManager(gpio_num_t adcPin, float dividerRatio = 1.0, adc1_channel_t adcChannel = ADC1_CHANNEL_MAX);
    bool begin(); // Cấu hình ADC
    void startTask(UBaseType_t priority = 1); // Task đọc nền
    void stopTask();

    // Lấy dữ liệu pin
    float getVoltage();      // Trả về điện áp pin (Volt)
    int getPercentage();     // Trả về phần trăm pin (0-100), -1 nếu lỗi
    void getData(float& voltage, int& percentage); // Lấy cả hai

private:
    gpio_num_t _adcPin;             // Chân GPIO nối với điểm giữa mạch chia áp
    adc1_channel_t _adcChannel;     // Kênh ADC1 tương ứng với chân GPIO
    float _dividerRatio;          // Tỷ lệ chia áp (Vpin / Vadc)
    bool _adcInitialized;         // ADC đã được cấu hình chưa?

    TaskHandle_t taskHandle;      // Handle cho task đọc pin
    SemaphoreHandle_t dataMutex;  // Mutex bảo vệ dữ liệu pin

    // Dữ liệu cục bộ
    float voltageLocal;           // Điện áp pin cuối cùng đọc được (Volt)
    int percentageLocal;          // Phần trăm pin cuối cùng tính được

    // Hàm private
    static void taskFunction(void* pvParameters); // Task chạy nền
    void readAndUpdate();        // Hàm đọc ADC và cập nhật giá trị local
    float readRawVoltage();      // Đọc và tính toán điện áp tại chân ADC
    int calculatePercentage(float voltage); // Tính phần trăm từ điện áp

    // Cấu hình ADC (có thể di chuyển vào begin)
    bool configureADC();

    // Hằng số tính toán (có thể đưa ra Config.h)
    static const int NUM_ADC_SAMPLES = 64; // Số mẫu ADC lấy trung bình
    // Ngưỡng điện áp cho phần trăm pin (cần tinh chỉnh)
    static constexpr float BATT_MAX_V = 4.18; // Điện áp đầy thực tế
    static constexpr float BATT_MIN_V = 3.20; // Điện áp gần cạn an toàn
};

#endif // BATTERY_MANAGER_H