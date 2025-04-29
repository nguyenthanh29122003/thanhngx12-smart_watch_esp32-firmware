// include/Config.h
#ifndef CONFIG_H
#define CONFIG_H

// --- Button Pins ---
// #define MODE_BUTTON_PIN 12 // <<< Vô hiệu hóa hoặc xóa nếu không có nút này
#define BUTTON_PIN 0 // <<< SỬ DỤNG NÚT BOOT (GPIO 0) LÀM NÚT CHÍNH
// Nếu bạn có 2 nút thật, hãy xác định chân và đặt tên lại (ví dụ BUTTON_LEFT, BUTTON_RIGHT)

// --- I2C Pins (QMI8658 specific) ---
// Thư viện QMI của Lewis He có thể nhận chân qua hàm begin, không cần define ở đây
// #define QMI8658_SDA_PIN 42 // Chỉ để tham khảo nếu cần
// #define QMI8658_SCL_PIN 41 // Chỉ để tham khảo nếu cần

// --- I2C Addresses (QUAN TRỌNG: Xác nhận bằng I2C Scanner!) ---
#define QMI8658_ADDRESS   0x6B // <<< SỬA THÀNH ĐỊA CHỈ QUÉT ĐƯỢC (VÍ DỤ: 0x7E ?)
#define MAX30102_ADDRESS  0x57 // <<< XÁC NHẬN ĐỊA CHỈ NÀY BẰNG SCANNER!

// --- Power Control Pin (Nếu PlatformIO không tự định nghĩa) ---
// Thử biên dịch trước, nếu lỗi thiếu TFT_I2C_POWER thì mới cần dòng này
// #define TFT_I2C_POWER_PIN 21 // <<< THAY BẰNG CHÂN GPIO ĐÚNG CỦA BOARD BẠN

// --- BLE UUIDs (Giữ nguyên) ---
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789013"
#define WIFI_CONFIG_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define STATUS_UUID         "abce0001-ef00-1234-5678-90abcdef1234"

// --- StepCounter constants (Giữ nguyên, tinh chỉnh sau) ---
#define STEP_LENGTH 0.75
#define THRESHOLD 1.0
#define STEP_DELAY 300
#define ACC_THRESHOLD 0.5
#define GYRO_THRESHOLD 10
#define ACC_FILTER_ALPHA 0.9 // Có thể không cần nếu dùng dữ liệu trực tiếp từ QMI
#define GYRO_FILTER_ALPHA 0.9 // Có thể không cần

// --- HeartRateSpO2 constants (Giữ nguyên, tinh chỉnh sau) ---
#define IR_THRESHOLD 500
#define MOTION_THRESHOLD 10000
#define BPM_MIN 20
#define BPM_MAX 255
#define SPO2_MIN 90
#define SPO2_MAX 100
#define FILTER_ALPHA 0.95
#define SAMPLE_RATE_ACTIVE 50
#define SAMPLE_RATE_IDLE 10

// --- Time configuration (Giữ nguyên) ---
#define NTP_SERVER "time.google.com"
#define GMT_OFFSET_SEC (7 * 3600)
#define DAYLIGHT_OFFSET_SEC 0

// --- Power management (Giữ nguyên) ---
#define SENSOR_SHUTDOWN_DELAY 5000

// --- Task priorities (Giữ nguyên) ---
#define TASK_PRIORITY_HEART_RATE 2
#define TASK_PRIORITY_STEP 1
#define TASK_PRIORITY_TIME 1
// Có thể thêm priority cho Barometer Task nếu tạo task riêng

// --- Các hằng số khác (Giữ nguyên) ---
#define SAVE_STEP_INTERVAL 50
#define STEP_DETECT_MAX_GYRO 100.0
#define STEP_RESET_THRESHOLD (THRESHOLD * 0.8f)
#define I2C_MUTEX_TIMEOUT_MS 100

#define EEPROM_SIZE 512 
#define STEP_COUNT_ADDR 0 

// --- Battery Measurement ---
#define BATT_ADC_PIN    1  // <<< Đã xác nhận GPIO 1
#define BATT_VOLTAGE_DIVIDER_RATIO 2.0f // <<< Đã xác nhận R1=R2=100k -> Ratio=2.0  

#endif // CONFIG_H