// Cấu hình cho màn hình GC9A01 và ESP32
#define USER_SETUP_INFO "User_Setup for GC9A01"

// Chọn driver màn hình
#define GC9A01_DRIVER

// Định nghĩa các chân SPI
#define TFT_MISO  -1  // Không sử dụng MISO cho GC9A01 (chỉ cần MOSI và SCK)
#define TFT_MOSI  23  // SDA (SPI MOSI)
#define TFT_SCLK  18  // SCL (SPI SCK)
#define TFT_CS    5   // Chip Select
#define TFT_DC    2   // Data/Command (Thay đổi từ GPIO 4 thành GPIO 2 theo yêu cầu)
#define TFT_RST   4   // Reset

// Tần số SPI (tối đa cho GC9A01 thường là 40MHz)
#define SPI_FREQUENCY  40000000

// Cấu hình thêm
#define TFT_WIDTH   240  // Độ phân giải ngang
#define TFT_HEIGHT  240  // Độ phân giải dọc

// Màu sắc mặc định (có thể thay đổi trong code)
#define TFT_RGB_ORDER TFT_RGB  // Thứ tự màu RGB
#define TFT_INVERSION_OFF      // Không đảo màu

// Tắt các tính năng không cần thiết để tiết kiệm bộ nhớ
#define DISABLE_ALL_LIBRARY_WARNINGS
#define LOAD_GLCD   // Font 1
#define LOAD_FONT2  // Font 2
#define LOAD_FONT4  // Font 4
#define LOAD_GFXFF  // FreeFonts

// Không sử dụng touch screen
#define TOUCH_CS -1