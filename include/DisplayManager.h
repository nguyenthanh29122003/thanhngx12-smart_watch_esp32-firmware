// include/DisplayManager.h
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

// --- ADAFRUIT INCLUDES ---
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Specific driver for ST7789
#include <SPI.h>              // Required for SPI communication

// --- FONT INCLUDES ---
#include <Fonts/FreeSans9pt7b.h>       // Font for normal text
#include <Fonts/FreeSansBold12pt7b.h>  // Font for headings and important data
#include <Fonts/FreeMonoBold9pt7b.h>   // Monospace font for numerical data

// --- Standard & RTOS Includes ---
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"

// --- Display orientation constants ---
#define ORIENTATION_PORTRAIT  0  // Original vertical orientation
#define ORIENTATION_LANDSCAPE 1  // New horizontal orientation

// --- Screen dimensions based on orientation ---
#define SCREEN_WIDTH_LANDSCAPE  240  // Width in landscape mode
#define SCREEN_HEIGHT_LANDSCAPE 135  // Height in landscape mode

// --- Màu sắc và Hằng số UI ---
#define DARK_BACKGROUND   ST77XX_BLACK      // Standard Adafruit
#define DARK_TEXT         ST77XX_WHITE      // Standard Adafruit
#define DARK_LINES        0x8410            // Keep custom hex
#define DARK_BOX          0x5ACB            // Keep custom hex
#define DARK_HIGHLIGHT    ST77XX_ORANGE     // Standard Adafruit
#define LIGHT_BACKGROUND  ST77XX_WHITE      // Standard Adafruit
#define LIGHT_TEXT        ST77XX_BLACK      // Standard Adafruit
#define LIGHT_LINES       0xC618            // HEX for SILVER
#define LIGHT_BOX         0xDEFB            // Keep custom hex
#define LIGHT_HIGHLIGHT   0x000F            // HEX for NAVY

// Màu sắc chỉ số
#define COLOR_SPO2        ST77XX_CYAN       // Standard Adafruit
#define COLOR_HR          0xFDDF            // HEX for PINK
#define COLOR_STEPS       ST77XX_YELLOW     // Standard Adafruit
#define COLOR_TEMP        ST77XX_GREEN      // Standard Adafruit
#define COLOR_PRESSURE    ST77XX_BLUE       // Standard Adafruit
#define COLOR_ACCEL       ST77XX_RED        // Standard Adafruit
#define COLOR_GYRO        ST77XX_MAGENTA    // Standard Adafruit
#define COLOR_DATE        0xD69A            // HEX for LIGHTGREY
#define COLOR_TIME_DARK   0xAFBF            // Keep custom hex
#define COLOR_TIME_LIGHT  0x0594            // Keep custom hex
#define COLOR_DARKGREY    0x7BEF            // HEX for DARKGREY
#define NUM_POINTS 360

// --- Layout constants for landscape mode ---
#define CLOCK_CENTER_X     60   // X center of clock in landscape mode
#define CLOCK_CENTER_Y     67   // Y center of clock in landscape mode
#define CLOCK_RADIUS       55   // Radius of clock face in landscape mode
#define DATA_PANEL_X       130  // Starting X for data panel in landscape mode
#define DATA_PANEL_WIDTH   100  // Width of data panel in landscape mode

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
    bool begin(uint8_t orientation = ORIENTATION_LANDSCAPE); // Default to landscape mode
    void startTask(UBaseType_t priority = 1);
    void stopTask();

    void updateData(bool wifiConnected,
                    int stepCount, float distance,
                    int heartRate, int spo2,
                    float temperature, float pressure,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    const struct tm* timeinfo, bool timeInitialized);

    void toggleScreen();
    void switchDisplayMode();
    void toggleTheme();
    void setOrientation(uint8_t orientation); // New method to change orientation
    bool isScreenOn() const;
    uint8_t getOrientation() const; // Get current orientation

private:
    Adafruit_ST7789 tft;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex;

    bool screenOn;
    int currentTheme; // 0: Dark, 1: Light
    uint8_t displayOrientation; // Current orientation
    ScreenMode currentScreenMode;
    bool timeInitializedLocal;
    struct tm timeinfoLocal;
    bool wifiConnectedLocal;
    int stepCountLocal;
    float distanceLocal;
    int heartRateLocal;
    int spo2Local;
    float temperatureLocal;
    float pressureLocal;
    float axLocal, ayLocal, azLocal;
    float gxLocal, gyLocal, gzLocal;

    String lastTimeString;
    String lastDateStringWF;
    String lastDayStringWF;
    int lastSecondAngleWF;
    int lastDisplayedSteps;
    int lastDisplayedHR;
    int lastDisplayedSpO2;
    String lastDateStringSens;
    float lastTempEnv;
    float lastPresEnv;
    String lastTimeEnv;
    float lastAxIMU, lastAyIMU, lastAzIMU;
    float lastGxIMU, lastGyIMU, lastGzIMU;
    bool lastWifiState;
    bool needsRedrawCurrentScreen;

    float x[NUM_POINTS], y[NUM_POINTS];
    float px[NUM_POINTS], py[NUM_POINTS];
    float lx[NUM_POINTS], ly[NUM_POINTS];
    int startHour[12];
    int startMinute[60];

    static void taskFunction(void* pvParameters);
    void updateDisplay();

    // Screen drawing functions with orientation support
    void drawWatchFaceScreen(bool redrawStatic);
    void drawSensorPrimaryScreen(bool redrawStatic);
    void drawEnvironmentScreen(bool redrawStatic);
    void drawImuDataScreen(bool redrawStatic);
    void drawSensorDataPanel(bool redrawStatic); // New method for landscape data panel

    void clearScreen();
    void drawWifiIcon();
    void drawClockFace();
    void updateWatchFaceTimeDisplay(int angle, const String& currentDay, const String& currentTime);
    void updateWatchFaceDateDisplay(const String& fullDateStr);

    // Helper functions for drawing with orientation awareness
    void drawStringOptimized(const String& text, int x, int y, const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, uint8_t datum, String& lastText, int clearWidth = -1, int clearHeight = -1);
    void drawFloatOptimized(float value, int decimalPlaces, const String& unit, int x, int y, const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, uint8_t datum, float& lastValue, const char* format = nullptr, int clearWidth = -1, int clearHeight = -1);
    void drawIntOptimized(int value, const String& unit, int x, int y, const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, uint8_t datum, int& lastValue, const char* defaultText = "--", int clearWidth = -1, int clearHeight = -1, int validThreshold = -9999);
    
    // Get adjusted coordinates based on current orientation
    void getAdjustedCoordinates(int& x, int& y);
    int getScreenWidth() const;
    int getScreenHeight() const;
};

#endif // DISPLAY_MANAGER_H