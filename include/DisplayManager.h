// include/DisplayManager.h
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <TFT_eSPI.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- Màu sắc và Hằng số UI ---
#define DARK_BACKGROUND TFT_BLACK
#define DARK_TEXT TFT_WHITE
#define DARK_LINES 0x8410
#define DARK_BOX 0x5ACB
#define DARK_HIGHLIGHT TFT_ORANGE

#define LIGHT_BACKGROUND TFT_WHITE
#define LIGHT_TEXT TFT_BLACK
#define LIGHT_LINES TFT_SILVER
#define LIGHT_BOX 0xDEFB
#define LIGHT_HIGHLIGHT TFT_NAVY

#define RADIUS 104
#define CENTER_X 120
#define CENTER_Y 120
#define NUM_POINTS 360

#define COLOR_TIME_DARK   tft.color565(170, 250, 255) // #aafaff
#define COLOR_TIME_LIGHT  tft.color565(0, 180, 180)  // Màu Teal đậm hơn cho nền sáng
#define COLOR_DATE        TFT_LIGHTGREY             // #ccc
#define COLOR_SPO2        TFT_CYAN                  // #00e5ff
#define COLOR_HR          TFT_PINK                  // #ff5ba0
#define COLOR_STEPS       TFT_YELLOW                // #ffe15d

// --- Định nghĩa các chế độ màn hình ---
typedef enum {
    SCREEN_MODE_WATCHFACE = 0,
    SCREEN_MODE_SENSORS = 1,
    SCREEN_MODE_COUNT // Luôn là phần tử cuối để đếm số lượng màn hình
} ScreenMode;

class DisplayManager {
public:
    DisplayManager();
    void begin();
    void startTask();
    void stopTask();

    // --- Hàm cập nhật dữ liệu ---
    // Nhận tất cả dữ liệu cần cho các màn hình
    void updateData(bool wifiConnected,
                    int stepCount, float distance, // Dữ liệu StepCounter
                    int heartRate, int spo2,       // Dữ liệu HeartRateSpO2
                    const struct tm* timeinfo, bool timeInitialized); // Dữ liệu thời gian

    // --- Hàm điều khiển ---
    void toggleScreen();     // Bật/tắt màn hình (tự đảo trạng thái)
    void switchDisplayMode(); // Chuyển giữa các màn hình (WATCHFACE <-> SENSORS)
    bool isScreenOn() const;  // Kiểm tra trạng thái màn hình
    void toggleTheme();

private:
    TFT_eSPI tft;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex; // Mutex bảo vệ các biến trạng thái và dữ liệu nội bộ

    // --- Trạng thái Nội bộ ---
    bool screenOn;                 // Màn hình đang bật hay tắt?
    int currentTheme;              // 0: Dark, 1: Light (Giữ lại nếu muốn dùng)
    ScreenMode currentScreenMode;  // Màn hình nào đang hiển thị?
    bool timeInitializedLocal;     // Thời gian đã được đồng bộ chưa?
    struct tm timeinfoLocal;       // Bản sao cục bộ của thời gian
    bool wifiConnectedLocal;       // Trạng thái kết nối WiFi
    // Dữ liệu cảm biến cục bộ
    int stepCountLocal;
    float distanceLocal;
    int heartRateLocal;
    int spo2Local;

    // --- Biến Tối ưu Vẽ ---
    // Cho Watch Face
    String lastTimeString; // hh:mm:ss cuối cùng đã vẽ
    String lastDateString; // mm:dd cuối cùng đã vẽ
    String lastDayString;  // Tên ngày cuối cùng đã vẽ
    int lastSecondAngle;   // Góc giây cuối cùng đã vẽ
    // Cho Sensor Screen
    int lastDisplayedSteps;
    int lastDisplayedHR;
    int lastDisplayedSpO2;
    // Cho WiFi Icon
    bool lastWifiState;
    // Cờ yêu cầu vẽ lại toàn bộ màn hình
    bool needsRedrawWatchFace;
    bool needsRedrawSensorScreen;

    // --- Dữ liệu Tính toán UI (Cho Watch Face) ---
    float x[NUM_POINTS], y[NUM_POINTS];   // Tọa độ điểm trên vòng tròn
    float px[NUM_POINTS], py[NUM_POINTS]; // Tọa độ điểm cho vạch phút/giờ
    float lx[NUM_POINTS], ly[NUM_POINTS]; // Tọa độ điểm cuối vạch phút/giờ
    int startHour[12];                    // Index bắt đầu cho các mốc 30 độ
    int startMinute[60];                  // Index bắt đầu cho các mốc 6 độ (vạch phút)

    // --- Hàm Private ---
    static void taskFunction(void* pvParameters); // Hàm thực thi của Task
    void updateDisplay(); // Hàm chính được gọi trong Task để quyết định vẽ gì

    // --- Hàm Vẽ Chính cho Từng Màn Hình ---
    void drawWatchFaceScreen(); // Vẽ toàn bộ màn hình mặt đồng hồ
    void drawSensorDataScreen(); // Vẽ toàn bộ màn hình dữ liệu cảm biến

    // --- Hàm Vẽ Phụ Trợ ---
    void clearScreen();             // Xóa màn hình theo theme hiện tại
    void drawWifiIcon();            // Vẽ icon WiFi
    // Phụ trợ cho Watch Face
    void drawClockFace();           // Vẽ phần tĩnh của mặt đồng hồ (vạch, số, chữ...)
    void updateTimeDisplay(int angle, const String& currentDay, const String& currentTime); // Cập nhật phần động (giờ số, ngày chữ, vòng xoay)
    void updateDateDisplay(const String& currentDate); // Cập nhật hộp ngày tháng
};

#endif // DISPLAY_MANAGER_H