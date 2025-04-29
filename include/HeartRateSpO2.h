// include/HeartRateSpO2.h
#ifndef HEART_RATE_SPO2_H
#define HEART_RATE_SPO2_H

#include <MAX30105.h> // Đảm bảo đúng tên chip (MAX30102?) nếu thư viện hỗ trợ
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h" // Cần để lấy TASK_PRIORITY_HEART_RATE

class HeartRateSpO2 {
public:
    HeartRateSpO2();
    // Sửa: Trả về bool để báo thành công/thất bại, nhận đối tượng Wire
    bool begin(TwoWire &wireInstance = Wire);
    // Sửa: Nhận tham số priority, có giá trị mặc định từ Config.h
    void startTask(UBaseType_t priority = TASK_PRIORITY_HEART_RATE);
    void stopTask();
    // Cung cấp giá trị ĐÃ ĐƯỢC ỔN ĐỊNH HÓA và giá trị thô cuối cùng
    void getData(int& stableHeartRate, int& stableSpo2, long& lastIrValue, long& lastRedValue);

private:
    MAX30105 particleSensor; // Hoặc MAX30102 nếu dùng chip đó
    TwoWire* _wire;          // Con trỏ tới đối tượng I2C bus đang sử dụng
    TaskHandle_t taskHandle; // Handle cho Task FreeRTOS
    SemaphoreHandle_t dataMutex; // Mutex bảo vệ dữ liệu cục bộ

    // --- Trạng thái Cảm biến và Task ---
    bool sensorReady;       // Cảm biến đã được khởi tạo thành công chưa?

    // --- Dữ liệu Thô và Trạng thái Tính toán HR ---
    byte rates[4];          // Buffer lưu 4 giá trị BPM gần nhất (từ ví dụ)
    byte rateSpot;          // Index hiện tại trong buffer rates
    unsigned long lastBeat; // Thời điểm (ms) của nhịp đập cuối cùng được phát hiện

    // --- Dữ liệu Thô và Trạng thái Tính toán SpO2 ---
    float spo2_redDC;       // Thành phần DC ước tính của tín hiệu Red
    float spo2_irDC;        // Thành phần DC ước tính của tín hiệu IR

    // --- Dữ liệu Cuối cùng (Ổn định & Thô) ---
    int heartRateStable;    // Giá trị nhịp tim ổn định cuối cùng (BPM)
    int spo2Stable;         // Giá trị SpO2 ổn định cuối cùng (%)
    long irValueLocal;      // Giá trị IR thô cuối cùng đọc được
    long redValueLocal;     // Giá trị Red thô cuối cùng đọc được

    // --- Bộ đệm và Trạng thái Ổn định hóa ---
    static const int STABLE_BUFFER_SIZE = 5; // Kích thước bộ đệm ổn định (có thể điều chỉnh)
    int hrBuffer[STABLE_BUFFER_SIZE];        // Buffer cho HR
    int spo2Buffer[STABLE_BUFFER_SIZE];      // Buffer cho SpO2
    byte hrBufferIndex;                      // Index ghi tiếp theo cho buffer HR
    byte spo2BufferIndex;                     // Index ghi tiếp theo cho buffer SpO2
    byte hrValidCount;                       // Số lượng mẫu HR hợp lệ trong buffer
    byte spo2ValidCount;                     // Số lượng mẫu SpO2 hợp lệ trong buffer
    bool hrIsStable;                         // Cờ báo HR có ổn định không
    bool spo2IsStable;                       // Cờ báo SpO2 có ổn định không

    // --- Ngưỡng Ổn định (Có thể định nghĩa trong Config.h nếu muốn) ---
    // Ngưỡng này có thể cần tinh chỉnh dựa trên thực tế
    static const int HR_STABILITY_THRESHOLD = 3;  // Chênh lệch tối đa cho phép trong buffer HR
    static const int SPO2_STABILITY_THRESHOLD = 2; // Chênh lệch tối đa cho phép trong buffer SpO2
    static const byte MIN_VALID_FOR_STABLE = 3; // Số mẫu hợp lệ tối thiểu để đánh giá ổn định

    // --- Hàm Private ---
    static void taskFunction(void* pvParameters); // Hàm thực thi của Task
    void updateSensor();                          // Hàm chính đọc và xử lý dữ liệu cảm biến
    float lowPassFilter(float input, float previous, float alpha); // Hàm lọc thông thấp
    float calculateSpO2(long redValue, long irValue); // Hàm tính SpO2 thô (dùng biến thành viên)
    void checkHrStability(int currentHr);           // Hàm kiểm tra và cập nhật ổn định HR
    void checkSpo2Stability(int currentSpo2);        // Hàm kiểm tra và cập nhật ổn định SpO2
};

#endif // HEART_RATE_SPO2_H