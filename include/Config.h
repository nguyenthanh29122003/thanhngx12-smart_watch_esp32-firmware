#ifndef CONFIG_H
#define CONFIG_H

// Button
#define BUTTON_DISPLAY 12

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789013"
#define WIFI_CONFIG_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9" // Thêm dòng này

// StepCounter constants
#define STEP_LENGTH 0.75    // Chiều dài bước chân trung bình (mét)
#define THRESHOLD 1.2       // Ngưỡng phát hiện bước chân (g)
#define STEP_DELAY 300      // Thời gian trễ giữa các bước (ms)
#define ACC_THRESHOLD 0.5   // Ngưỡng gia tốc tối thiểu để bật cảm biến (g)
#define GYRO_THRESHOLD 20   // Ngưỡng con quay tối thiểu để bật cảm biến (°/s)
#define ACC_FILTER_ALPHA 0.9 // Hằng số bộ lọc Low-pass cho gia tốc
#define GYRO_FILTER_ALPHA 0.9 // Hằng số bộ lọc Low-pass cho con quay

// HeartRateSpO2 constants
#define IR_THRESHOLD 50000  // Ngưỡng IR để phát hiện ngón tay
#define MOTION_THRESHOLD 10000 // Ngưỡng phát hiện chuyển động dựa trên delta IR
#define BPM_MIN 20          // Nhịp tim tối thiểu (BPM)
#define BPM_MAX 255         // Nhịp tim tối đa (BPM)
#define SPO2_MIN 90         // SpO2 tối thiểu (%)
#define SPO2_MAX 100        // SpO2 tối đa (%)
#define FILTER_ALPHA 0.95   // Hằng số bộ lọc Low-pass cho tín hiệu IR/Red
#define SAMPLE_RATE_ACTIVE 50  // Tần suất lấy mẫu khi đo (Hz)
#define SAMPLE_RATE_IDLE 10    // Tần suất lấy mẫu khi chờ (Hz)

// Time configuration
#define NTP_SERVER "time.google.com"
#define GMT_OFFSET_SEC (7 * 3600) // GMT+7 (Việt Nam)
#define DAYLIGHT_OFFSET_SEC 0

// Power management
#define SENSOR_SHUTDOWN_DELAY 5000 // Thời gian chờ trước khi tắt cảm biến (ms, nếu cần)

// Task priorities
#define TASK_PRIORITY_HEART_RATE 2  // Ưu tiên cao cho HeartRateSpO2
#define TASK_PRIORITY_STEP 1        // Ưu tiên trung bình cho StepCounter
#define TASK_PRIORITY_TIME 1        // Ưu tiên trung bình cho TimeManager

#endif