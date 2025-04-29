// include/DisplayManager.h
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <TFT_eSPI.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h" // Để lấy TASK_PRIORITY và các định nghĩa màu/UI nếu chuyển vào đây

// --- Màu sắc và Hằng số UI ---
// (Giữ nguyên các định nghĩa màu sắc DARK/LIGHT)
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

// Kích thước và Tâm (Xác định lại cho màn hình mới)
// Lấy từ User_Setup.h thông qua TFT_WIDTH/TFT_HEIGHT (nếu User_Setup.h được include trước)
// Hoặc định nghĩa lại ở đây để chắc chắn
#define SCREEN_WIDTH  135 // Giả định User_Setup.h đã đúng
#define SCREEN_HEIGHT 240
#define CENTER_X (SCREEN_WIDTH / 2)
#define CENTER_Y (SCREEN_HEIGHT / 2)
#define WATCHFACE_RADIUS (SCREEN_WIDTH / 2 - 10) // Bán kính cho mặt đồng hồ

#define NUM_POINTS 360   // Số điểm để tính toán (cho mỗi độ)

// Màu sắc chỉ số
#define COLOR_SPO2        TFT_CYAN
#define COLOR_HR          TFT_PINK
#define COLOR_STEPS       TFT_YELLOW
#define COLOR_TEMP        TFT_GREEN
#define COLOR_PRESSURE    TFT_BLUE
#define COLOR_ACCEL       TFT_RED
#define COLOR_GYRO        TFT_MAGENTA
#define COLOR_DATE        TFT_LIGHTGREY // Màu cho ngày tháng (có thể dùng chung)


// --- Định nghĩa các chế độ màn hình ---
typedef enum {
    SCREEN_MODE_WATCHFACE = 0,
    SCREEN_MODE_SENSORS_PRIMARY,
    SCREEN_MODE_ENVIRONMENT,
    SCREEN_MODE_IMU_DATA,
    SCREEN_MODE_COUNT
} ScreenMode;


class DisplayManager {
public:
    DisplayManager();
    bool begin(); // Trả về bool
    void startTask(UBaseType_t priority = 1); // Nhận priority
    void stopTask();

    // --- Hàm cập nhật dữ liệu ---
    void updateData(bool wifiConnected,
                    int stepCount, float distance,
                    int heartRate, int spo2,
                    float temperature, float pressure, // Thêm BMP280
                    float ax, float ay, float az,      // Thêm IMU
                    float gx, float gy, float gz,
                    const struct tm* timeinfo, bool timeInitialized);

    // --- Hàm điều khiển ---
    void toggleScreen();
    void switchDisplayMode();
    void toggleTheme(); // Đã thêm khai báo này
    bool isScreenOn() const;

private:
    TFT_eSPI tft;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex;

    // --- Trạng thái Nội bộ ---
    bool screenOn;
    int currentTheme; // 0: Dark, 1: Light
    ScreenMode currentScreenMode;
    bool timeInitializedLocal;
    struct tm timeinfoLocal;
    bool wifiConnectedLocal;
    // Dữ liệu cảm biến cục bộ
    int stepCountLocal;
    float distanceLocal; // Giữ lại nếu cần hiển thị
    int heartRateLocal;
    int spo2Local;
    float temperatureLocal;
    float pressureLocal;
    float axLocal, ayLocal, azLocal;
    float gxLocal, gyLocal, gzLocal;

    // --- Biến Tối ưu Vẽ ---
    String lastTimeString; // Dùng chung cho các màn hình (HH:MM)
    // Watch Face
    String lastDateStringWF; // Lưu chuỗi ngày đầy đủ (DAY DD MMM) cho WF
    String lastDayStringWF;  // Lưu tên ngày đầy đủ cho WF
    int lastSecondAngleWF;   // Lưu góc giây cho WF
    // Sensor Screens
    int lastDisplayedSteps;
    int lastDisplayedHR;
    int lastDisplayedSpO2;
    String lastDateStringSens; // Lưu chuỗi ngày/giờ cho Sensor/Env/IMU
    float lastTempEnv;
    float lastPresEnv;
    String lastTimeEnv;
    // String lastTimeEnv; // Không cần nếu dùng lastDateStringSens
    float lastAxIMU, lastAyIMU, lastAzIMU;
    float lastGxIMU, lastGyIMU, lastGzIMU;
    // WiFi Icon
    bool lastWifiState;
    // Cờ Redraw
    bool needsRedrawCurrentScreen; // Cờ chung

    // --- Dữ liệu Tính toán UI (Cho Watch Face) ---
    float x[NUM_POINTS], y[NUM_POINTS];
    float px[NUM_POINTS], py[NUM_POINTS];
    float lx[NUM_POINTS], ly[NUM_POINTS];
    int startHour[12];
    int startMinute[60];

    // --- Hàm Private ---
    static void taskFunction(void* pvParameters);
    void updateDisplay();

    // --- Hàm Vẽ Chính ---
    void drawWatchFaceScreen(bool redrawStatic);
    void drawSensorPrimaryScreen(bool redrawStatic);
    void drawEnvironmentScreen(bool redrawStatic);
    void drawImuDataScreen(bool redrawStatic);

    // --- Hàm Vẽ Phụ Trợ ---
    void clearScreen();
    void drawWifiIcon();
    // Phụ trợ Watch Face
    void drawClockFace();
    void updateWatchFaceTimeDisplay(int angle, const String& currentDay, const String& currentTime);
    void updateWatchFaceDateDisplay(const String& fullDateStr); // Sửa tên và kiểu tham số
    // Hàm vẽ tối ưu (ĐÃ SỬA THAM SỐ THAM CHIẾU)
    void drawStringOptimized(const String& text, int x, int y, int font, uint16_t textColor, uint16_t bgColor, int datum, String& lastText, int clearWidth = -1, int clearHeight = -1);
    void drawFloatOptimized(float value, int decimalPlaces, const String& unit, int x, int y, int font, uint16_t textColor, uint16_t bgColor, int datum, float& lastValue, const char* format = nullptr, int clearWidth = -1, int clearHeight = -1);
    void drawIntOptimized(int value, const String& unit, int x, int y, int font, uint16_t textColor, uint16_t bgColor, int datum, int& lastValue, const char* defaultText = "--", int clearWidth = -1, int clearHeight = -1, int validThreshold = -9999);
};

#endif // DISPLAY_MANAGER_H