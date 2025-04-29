// include/StepCounter.h
#ifndef STEP_COUNTER_H
#define STEP_COUNTER_H

// #include <MPU6050.h> // <-- XÓA
#include <SensorQMI8658.hpp> // <-- THÊM THƯ VIỆN QMI8658

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class StepCounter {
public:
    StepCounter();
    bool begin(); // <-- Đổi kiểu trả về
    void startTask(UBaseType_t priority = 1); // <-- Nhận priority
    void stopTask();
    void updateSensor();
    // Giữ nguyên hàm getData
    void getData(int& stepCount, float& distance, float& ax, float& ay, float& az,
                 float& gx, float& gy, float& gz);
    void resetSteps(); // <-- Thêm hàm reset

private:
    // MPU6050 mpu; // <-- XÓA
    SensorQMI8658 qmi; // <-- THAY BẰNG ĐỐI TƯỢNG QMI8658

    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex; // Mutex bảo vệ dữ liệu nội bộ

    // Trạng thái nội bộ
    bool sensorReady;       // Cảm biến đã sẵn sàng chưa?
    bool stepDetected;      // Cờ phát hiện bước đi trong chu kỳ hiện tại
    unsigned long lastStepTime; // Thời gian của bước đi cuối
    int stepCountLocal;     // Số bước đếm được
    float distanceLocal;    // Quãng đường ước tính
    int lastSavedStepCount; // Số bước lúc lưu EEPROM lần cuối

    // Dữ liệu cảm biến cục bộ (đã chuyển đổi đơn vị)
    float axLocal, ayLocal, azLocal; // gia tốc (g)
    float gxLocal, gyLocal, gzLocal; // vận tốc góc (deg/s)

    // Bộ lọc (tùy chọn, có thể không cần)
    // float accFiltered, gyroFiltered;

    // Hàm private
    static void taskFunction(void* pvParameters);
    float lowPassFilter(float input, float previous, float alpha); // Giữ lại nếu dùng
    bool detectStep(float accMagnitude, float gyroMagnitude);
};

#endif // STEP_COUNTER_H