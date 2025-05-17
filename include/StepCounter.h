// include/StepCounter.h
#ifndef STEP_COUNTER_H
#define STEP_COUNTER_H

#include <SensorQMI8658.hpp> // Sử dụng thư viện cho QMI8658C

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"          // Để lấy TASK_PRIORITY_STEP và các hằng số khác

class StepCounter {
public:
    StepCounter();
    bool begin(); // Trả về true nếu khởi tạo thành công
    void startTask(UBaseType_t priority = TASK_PRIORITY_STEP); // Nhận ưu tiên task
    void stopTask();
    // Hàm getData giữ nguyên để cung cấp dữ liệu ra ngoài
    void getData(int& stepCount, float& distance, float& ax, float& ay, float& az,
                 float& gx, float& gy, float& gz);
    void resetSteps(); // Hàm reset số bước về 0

private:
    SensorQMI8658 qmi; // Đối tượng cảm biến QMI8658C

    TaskHandle_t taskHandle;      // Handle cho Task FreeRTOS
    SemaphoreHandle_t dataMutex;  // Mutex bảo vệ dữ liệu cục bộ của StepCounter

    // --- Trạng thái Nội bộ của Module ---
    bool sensorReady;           // Cờ báo cảm biến đã được khởi tạo và sẵn sàng
    bool stepDetectedThisCycle; // Cờ phát hiện bước đi trong chu kỳ updateSensor hiện tại
    unsigned long lastStepTimeMs; // Thời điểm (ms) của bước đi cuối cùng được phát hiện
    int stepCountLocal;         // Số bước chân đếm được (lưu trữ nội bộ)
    float distanceLocal;        // Quãng đường ước tính (mét)
    int lastSavedStepCountEEPROM; // Số bước chân cuối cùng đã được lưu vào EEPROM

    // Dữ liệu cảm biến cục bộ (đã được chuyển đổi sang đơn vị chuẩn: g và deg/s)
    float axLocal, ayLocal, azLocal; // Gia tốc (g)
    float gxLocal, gyLocal, gzLocal; // Vận tốc góc (deg/s)

    // --- Biến cho bộ lọc EMA (Nếu bạn quyết định sử dụng lại) ---
    // float accMagnitudeFiltered;
    // float gyroMagnitudeFiltered;

    // --- Hàm Private ---
    static void taskFunction(void* pvParameters); // Hàm thực thi của Task
    void updateSensorData();                      // Hàm chính đọc và xử lý dữ liệu cảm biến
    float lowPassFilterEMA(float input, float previousValue, float alpha); // Hàm lọc EMA
    bool checkForStep(float currentAccMagnitude, float currentGyroMagnitude); // Thuật toán phát hiện bước
};

#endif // STEP_COUNTER_H