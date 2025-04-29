// include/User_Setup.h

// Đặt tên cho cấu hình này (tùy chọn)
#define USER_SETUP_INFO "ESP32-S3_TFT_135x240_ST7789" // <<< ĐỔI TÊN

// --- Chọn Driver ---
#define ST7789_DRIVER      // <<< KÍCH HOẠT DRIVER NÀY

// --- Kích thước màn hình ---
#define TFT_WIDTH  135     // <<< SỬA KÍCH THƯỚC
#define TFT_HEIGHT 240     // <<< SỬA KÍCH THƯỚC

// --- Offset cho màn hình không vuông (Quan trọng cho ST7789 135x240) ---
// Thử nghiệm các giá trị offset này. Chúng khác nhau tùy loại màn hình.
// Offset có thể là 40, 52, 53...
#define TFT_OFFSET_X 52    // <<< THÊM/SỬA OFFSET X (Thử 52 hoặc 53 trước)
#define TFT_OFFSET_Y 40    // <<< THÊM/SỬA OFFSET Y (Thử 40 trước)
// Nếu không có offset, hình ảnh có thể bị lệch

// --- Định nghĩa Chân Kết nối SPI và Điều khiển (Theo tài liệu) ---
#define TFT_MISO  37     // <<< SỬA CHÂN (Hoặc -1 nếu chắc chắn không dùng)
#define TFT_MOSI  35     // <<< SỬA CHÂN
#define TFT_SCLK  36     // <<< SỬA CHÂN
#define TFT_CS    7      // <<< SỬA CHÂN
#define TFT_DC   39      // <<< SỬA CHÂN
#define TFT_RST  40      // <<< SỬA CHÂN

// --- Định nghĩa Chân Đèn nền (Backlight) ---
#define TFT_BL   45      // <<< THÊM/SỬA CHÂN BL
// #define TFT_BACKLIGHT_ON HIGH // <<< Bỏ comment dòng này nếu HIGH là bật đèn nền (thường là vậy)
// #define TFT_BACKLIGHT_ON LOW  // <<< Hoặc dùng dòng này nếu LOW là bật

// --- Tần số SPI ---
#define SPI_FREQUENCY  40000000 // Giữ 40MHz hoặc thử 27MHz nếu không ổn định
// #define SPI_FREQUENCY  27000000

// --- Cấu hình Màu sắc và Đảo ngược (Giữ nguyên ban đầu, sửa nếu cần) ---
#define TFT_RGB_ORDER TFT_RGB  // Thử TFT_BGR nếu màu đỏ/xanh bị tráo
// #define TFT_INVERSION_ON    // Thử nếu màu bị đảo (đen thành trắng)
#define TFT_INVERSION_OFF

// --- Load Font (Giữ nguyên như cũ là tốt) ---
#define LOAD_GLCD   // Font 1
#define LOAD_FONT2  // Font 2
#define LOAD_FONT4  // Font 4
// #define LOAD_FONT6 // Có thể bỏ nếu không dùng
// #define LOAD_FONT7 // Có thể bỏ nếu không dùng
// #define LOAD_FONT8 // Có thể bỏ nếu không dùng
#define LOAD_GFXFF  // <<< RẤT QUAN TRỌNG cho DSEG7 và các font khác
#define SMOOTH_FONT // <<< RẤT QUAN TRỌNG cho font đẹp

// --- Touch Screen (Vẫn không dùng) ---
#define TOUCH_CS -1

// --- SPI Read Frequency (Giữ nguyên hoặc đặt theo SPI_FREQUENCY) ---
#define SPI_READ_FREQUENCY  20000000

// --- Touch SPI Frequency (Giữ nguyên) ---
#define SPI_TOUCH_FREQUENCY  2500000