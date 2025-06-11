// include/Config.h
#ifndef CONFIG_H
#define CONFIG_H

// --- Button Pins ---
#define BUTTON_PIN 0 // Sử dụng nút BOOT (GPIO 0) làm nút chính
                     // Nếu bạn dùng thư viện OneButton và có 2 nút vật lý,
                     // bạn có thể định nghĩa MODE_BUTTON_PIN và POWER_BUTTON_PIN ở đây.

// --- I2C Pins (QMI8658 specific - Thư viện tự quản lý, không cần ở đây) ---
// #define QMI8658_SDA_PIN 42
// #define QMI8658_SCL_PIN 41

// --- I2C Addresses (QUAN TRỌNG: Xác nhận lại bằng I2C Scanner trên board S3!) ---
#define QMI8658_ADDRESS   0x6B // HOẶC 0x7E (Địa chỉ QMI8658C)
#define BMP280_ADDRESS    0x77 // HOẶC 0x76 (Địa chỉ BMP280)
#define MAX30102_ADDRESS  0x57 // Địa chỉ phổ biến cho MAX3010x
#define MPU6050_ADDRESS 0x68 // Địa chỉ phổ biến cho MPU6050

// --- Power Control Pin (Nếu PlatformIO không tự định nghĩa TFT_I2C_POWER) ---
// #define TFT_I2C_POWER_PIN 21 // THAY BẰNG CHÂN GPIO ĐÚNG NẾU CẦN

// --- BLE UUIDs  ---
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789013"
#define WIFI_CONFIG_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define STATUS_UUID         "abce0001-ef00-1234-5678-90abcdef1234"

// --- StepCounter constants (QMI8658C) ---
#define STEP_LENGTH 0.75f    // Chiều dài bước chân (m)
#define THRESHOLD 1.1f       // Ngưỡng gia tốc phát hiện bước (g) - Cần tinh chỉnh cho QMI
#define STEP_DELAY 250       // Thời gian trễ tối thiểu giữa các bước (ms)
// ACC_THRESHOLD, GYRO_THRESHOLD không dùng nếu không có logic sleep/wake thủ công
// #define ACC_THRESHOLD 0.5f
// #define GYRO_THRESHOLD 10.0f
// ACC_FILTER_ALPHA, GYRO_FILTER_ALPHA không dùng nếu đọc trực tiếp từ QMI
// #define ACC_FILTER_ALPHA 0.9f
// #define GYRO_FILTER_ALPHA 0.9f

// --- HeartRateSpO2 constants (MAX30102 & Pulse_Custom logic) ---
// Cấu hình cho MAX30102_Custom driver
#define MAX_LED_BRIGHTNESS 0x24 // Độ sáng LED (0x00-0xFF), ví dụ ~7.6mA. Cần tinh chỉnh!
#define MAX_SAMPLE_AVERAGE 2    // Số mẫu lấy trung bình trong FIFO chip (0=1, 1=2, 2=4, 3=8, 4=16, 5=32) -> 2 tương ứng 4 mẫu
#define MAX_LED_MODE       3    // Chế độ LED (2 = Red+IR, 3 = Red+IR+Green). Dùng 2 hoặc 3 cho SpO2.
#define MAX_SAMPLE_RATE    100  // Tần số lấy mẫu của chip (Hz) (50, 100, 200, 400, 800, 1000, 1600, 3200)
#define MAX_PULSE_WIDTH    411  // Độ rộng xung (us) (69, 118, 215, 411)
#define MAX_ADC_RANGE      4096 // Dải ADC (nA) (2048, 4096, 8192, 16384)
#define MAX_SLOT1_LEDS  0x01 // Chỉ LED1 (IR) cho SLOT1
#define MAX_SLOT2_LEDS  0x02 // Chỉ LED2 (RED) cho SLOT2
#define MAX_SLOT3_LEDS  0x00 // Không dùng SLOT3
#define MAX_SLOT4_LEDS  0x00 // Không dùng SLOT4

// Ngưỡng và hằng số cho thuật toán Pulse_Custom
#define IR_DETECT_THRESHOLD 5000 // Ngưỡng tín hiệu IR tối thiểu để coi là có ngón tay (điều chỉnh theo thực tế)
#define PULSE_DC_FILTER_NSAMPLE 24  // Số mẫu cho bộ lọc DC (alpha = 1/N)
#define PULSE_MA_FILTER_NSLOT   4   // Số mẫu cho bộ lọc trung bình trượt làm mịn tín hiệu AC
#define PULSE_MIN_BEAT_AMPLITUDE 50 // Ngưỡng biên độ AC tối thiểu của nhịp đập (quan trọng, cần tinh chỉnh)
#define PULSE_MAX_BEAT_AMPLITUDE 2000// Ngưỡng biên độ AC tối đa (tránh nhiễu lớn)
#define BPM_MIN 30          // Nhịp tim tối thiểu (BPM)
#define BPM_MAX 220         // Nhịp tim tối đa (BPM)
#define SPO2_MIN_VALID 85   // SpO2 tối thiểu được coi là hợp lệ sau khi tính toán
#define SPO2_MAX_VALID 100  // SpO2 tối đa
// FILTER_ALPHA cũ (0.95f) không còn dùng trực tiếp nếu Pulse_Custom có alpha riêng

// --- Time configuration  ---
#define NTP_SERVER "time.google.com"
#define GMT_OFFSET_SEC (7 * 3600)
#define DAYLIGHT_OFFSET_SEC 0

// --- Power management  ---
// #define SENSOR_SHUTDOWN_DELAY 5000 // Có thể không cần nếu quản lý sleep riêng

// --- Task priorities (Điều chỉnh nếu cần) ---
#define TASK_PRIORITY_DISPLAY    1
#define TASK_PRIORITY_BLUETOOTH  1
#define TASK_PRIORITY_STEP       1 // Có thể tăng nếu cần xử lý nhanh hơn
#define TASK_PRIORITY_HEART_RATE 2 // Giữ ưu tiên cao
#define TASK_PRIORITY_BAROMETER  1 // Ưu tiên thấp cho cảm biến môi trường
#define TASK_PRIORITY_BATTERY    1 // Ưu tiên thấp cho đo pin
#define TASK_PRIORITY_TIME       1

// --- Các hằng số khác ---
#define SAVE_STEP_INTERVAL 50
#define STEP_DETECT_MAX_GYRO 150.0f // Ngưỡng con quay tối đa cho đếm bước (dps) - Cần tinh chỉnh cho QMI
#define STEP_RESET_THRESHOLD (THRESHOLD * 0.7f) // Ngưỡng reset cờ phát hiện bước
#define I2C_MUTEX_TIMEOUT_MS 100 // Timeout (ms) chờ I2C mutex

#define EEPROM_SIZE 512
#define STEP_COUNT_ADDR 0

// --- Battery Measurement  ---
#define BATT_ADC_PIN    1
#define BATT_VOLTAGE_DIVIDER_RATIO 2.0f

#endif // CONFIG_H