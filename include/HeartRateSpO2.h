// include/HeartRateSpO2.h
#ifndef HEART_RATE_SPO2_H
#define HEART_RATE_SPO2_H

#include "Max30102_Custom.h" // Sử dụng driver MAX30102 tùy chỉnh
#include "Pulse_Custom.h"    // Sử dụng lớp xử lý xung tùy chỉnh
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"          // Cho các hằng số cấu hình và ưu tiên task

class HeartRateSpO2 {
public:
    HeartRateSpO2();
    bool begin(TwoWire &wireInstance = Wire); // Khởi tạo cảm biến và module
    void startTask(UBaseType_t priority = TASK_PRIORITY_HEART_RATE); // Bắt đầu task xử lý
    void stopTask(); // Dừng task và tắt cảm biến
    // Cung cấp giá trị HR, SpO2 (đã ổn định nếu có logic), và giá trị IR/Red thô cuối cùng
    void getData(int& heartRate, int& spo2, long& irValue, long& redValue);

private:
    MAX30102_Custom sensor; // Đối tượng driver cấp thấp cho MAX30102
    Pulse_Custom pulseIR;   // Đối tượng xử lý xung cho kênh IR (chủ yếu cho HR)
    Pulse_Custom pulseRed;  // Đối tượng xử lý xung cho kênh Red (cần cho SpO2)
    MAFilter_Custom bpmSmoother; // Bộ lọc làm mịn giá trị BPM tính được

    TwoWire* _wire;          // Con trỏ tới đối tượng I2C bus đang sử dụng
    TaskHandle_t taskHandle; // Handle cho Task FreeRTOS
    SemaphoreHandle_t dataMutex; // Mutex bảo vệ dữ liệu cục bộ được chia sẻ

    bool sensorReady;       // Cờ báo cảm biến đã được khởi tạo và sẵn sàng

    // --- Dữ liệu Cuối cùng để Cung cấp ra ngoài ---
    int heartRateFinal;     // Giá trị nhịp tim cuối cùng (BPM)
    int spo2Final;          // Giá trị SpO2 cuối cùng (%)
    long irValueRawLast;    // Giá trị IR thô cuối cùng đọc được từ cảm biến
    long redValueRawLast;   // Giá trị Red thô cuối cùng đọc được từ cảm biến

    // --- Biến trạng thái cho thuật toán HR ---
    unsigned long lastBeatTimestampMs; // Thời điểm (ms) của nhịp đập cuối cùng được phát hiện

    // --- Logic Ổn định hóa (Tùy chọn - có thể thêm sau nếu cần) ---
    // (Bỏ qua các biến buffer ổn định ở bước này để tập trung vào thuật toán chính)
    // bool hrIsStable;
    // bool spo2IsStable;

    // --- Hàm Private ---
    static void taskFunction(void* pvParameters); // Hàm thực thi của Task
    void updateSensorData();                     // Hàm chính đọc và xử lý dữ liệu cảm biến
};

#endif // HEART_RATE_SPO2_H