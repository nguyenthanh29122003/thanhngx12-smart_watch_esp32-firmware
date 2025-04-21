// include/DisplayManager.h
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <TFT_eSPI.h>
#include <time.h>     // Cần cho struct tm
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Định nghĩa màu sắc (có thể chuyển vào Config.h)
#define DARK_BACKGROUND TFT_BLACK
#define DARK_TEXT TFT_WHITE
#define DARK_LINES 0x8410 // Màu xám tối cho đường kẻ/chấm
#define DARK_BOX 0x5ACB // Màu hộp tối
#define DARK_HIGHLIGHT TFT_ORANGE // Màu nhấn tối

#define LIGHT_BACKGROUND TFT_WHITE
#define LIGHT_TEXT TFT_BLACK
#define LIGHT_LINES TFT_SILVER // Màu xám sáng
#define LIGHT_BOX 0xDEFB // Màu hộp sáng (Gần giống xám)
#define LIGHT_HIGHLIGHT TFT_NAVY // Màu nhấn sáng

// Định nghĩa hằng số UI (có thể chuyển vào Config.h)
#define RADIUS 104       // Bán kính vòng tròn chính
#define CENTER_X 120     // Tọa độ tâm X
#define CENTER_Y 120     // Tọa độ tâm Y
#define NUM_POINTS 360   // Số điểm để tính toán (cho mỗi độ)
// #define BACKLIGHT_PIN 5 // Không dùng PWM trong phiên bản này
// #define PWM_LED_CHANNEL 0

// Font Names (kiểm tra tên chính xác trong thư viện TFT_eSPI)
#define DSEG7_CLASSIC_REGULAR_28 "DSEG7Classic-Regular28"
#define DSEG7_MODERN_BOLD_20 "DSEG7Modern-Bold20"
#define SECOND_HAND_LENGTH (RADIUS - 10)


class DisplayManager {
public:
    DisplayManager();
    void begin();
    void startTask();
    void stopTask();
    // Chỉ cần cập nhật thời gian và trạng thái WiFi (để vẽ icon nếu muốn)
    void updateData(bool wifiConnected, const struct tm* timeinfo, bool timeInitialized);
    void toggleScreen(); // Hàm mới, tự đảo trạng thái
    void toggleTheme();  // Hàm mới để đổi theme
    bool isScreenOn() const; // Hàm kiểm tra trạng thái màn hình

private:
    TFT_eSPI tft;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex; // Mutex bảo vệ dữ liệu nội bộ

    // Trạng thái nội bộ
    bool screenOn;
    int currentTheme; // 0: Dark, 1: Light
    bool timeInitializedLocal;
    struct tm timeinfoLocal;
    bool wifiConnectedLocal; // Lưu trạng thái WiFi

    // Biến trạng thái cho việc vẽ tối ưu
    String lastTimeString; // Lưu hh:mm:ss
    String lastDateString; // Lưu mm:dd
    String lastDayString;  // Lưu tên ngày
    int lastSecondAngle; // Lưu góc giây cuối cùng (-1 để vẽ lần đầu)

    // Mảng lưu tọa độ điểm (tính toán một lần)
    float x[NUM_POINTS], y[NUM_POINTS];
    float px[NUM_POINTS], py[NUM_POINTS];
    float lx[NUM_POINTS], ly[NUM_POINTS];
    int startHour[12], startMinute[60];

    // Hàm private để vẽ
    static void taskFunction(void* pvParameters);
    void updateDisplay(); // Hàm chính trong task
    void drawStaticUI(); // Vẽ các thành phần tĩnh theo theme
    void updateDynamicElements(int angle, const String& currentDay, const String& currentTime, const String& currentDate);
    void updateDateDisplay(const String& currentDate);
    void refreshDynamicElements(); // Vẽ lại phần động ngay lập tức
    void drawWifiIcon(); // Hàm vẽ icon WiFi
    void drawClockFace(); // Vẽ nền tĩnh và các vạch/số cố định
    void updateTimeDisplay(int angle, const String& currentDay, const String& currentTime, const String& currentDate);
    void drawSecondHand(int angle, uint16_t color); // Hàm vẽ kim giây  
};

#endif