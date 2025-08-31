// include/DisplayManager.h
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

// --- ADAFRUIT INCLUDES ---
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Specific driver for ST7789
#include <SPI.h>              // Required for SPI communication

// --- FONT INCLUDES ---
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>

// --- Standard & RTOS Includes ---
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "BluetoothManager.h"

// --- DISPLAY DIMENSIONS ---
// Modified for horizontal layout
#define SCREEN_WIDTH  240  // Swapped from original
#define SCREEN_HEIGHT 135  // Swapped from original
#define CENTER_X      (SCREEN_WIDTH / 2)
#define CENTER_Y      (SCREEN_HEIGHT / 2)

// --- UI CONSTANTS ---
// Adjusted for horizontal layout
#define HEADER_HEIGHT       20  // Reduced for horizontal layout
#define FOOTER_HEIGHT       15  // Reduced for horizontal layout
#define CONTENT_TOP         (HEADER_HEIGHT + 2)
#define CONTENT_HEIGHT      (SCREEN_HEIGHT - HEADER_HEIGHT - FOOTER_HEIGHT)
#define CARD_MARGIN         5
#define CARD_PADDING        8
#define CARD_CORNER_RADIUS  6
#define ICON_SIZE           16
#define ANIM_DURATION       300  // ms

// --- COLOR PALETTE (DARK THEME) ---
#define DARK_BACKGROUND   ST77XX_BLACK
#define DARK_SURFACE      0x2104      // Dark gray for cards
#define DARK_PRIMARY      0x03EF      // Teal
#define DARK_SECONDARY    0xFD20      // Coral
#define DARK_TEXT         ST77XX_WHITE
#define DARK_TEXT_SECONDARY 0xBDF7    // Light gray
#define DARK_DIVIDER      0x4208      // Medium gray
#define DARK_HIGHLIGHT    0x07FF      // Cyan

// --- COLOR PALETTE (LIGHT THEME) ---
#define LIGHT_BACKGROUND  ST77XX_WHITE
#define LIGHT_SURFACE     0xEF7D      // Light gray for cards
#define LIGHT_PRIMARY     0x03EF      // Teal
#define LIGHT_SECONDARY   0xFD20      // Coral
#define LIGHT_TEXT        ST77XX_BLACK
#define LIGHT_TEXT_SECONDARY 0x7BEF   // Medium gray
#define LIGHT_DIVIDER     0xC618      // Light gray
#define LIGHT_HIGHLIGHT   0x001F      // Blue

// --- DATA COLORS (CONSISTENT ACROSS THEMES) ---
#define COLOR_HEART       0xF800      // Red
#define COLOR_SPO2        0x07FF      // Cyan
#define COLOR_STEPS       0xFFE0      // Yellow
#define COLOR_TEMP        0x07E0      // Green
#define COLOR_PRESSURE    0x001F      // Blue
#define COLOR_BATTERY     0xAFE0      // Lime
#define COLOR_WARNING     0xFD00      // Orange
#define COLOR_SUCCESS     0x07E0      // Green
#define COLOR_ERROR       0xF800      // Red

// --- ANIMATION TYPES ---
typedef enum {
    ANIM_NONE = 0,
    ANIM_FADE_IN,
    ANIM_FADE_OUT,
    ANIM_SLIDE_LEFT,
    ANIM_SLIDE_RIGHT,
    ANIM_SLIDE_UP,
    ANIM_SLIDE_DOWN
} AnimationType;

// --- SCREEN MODES ---
typedef enum {
    SCREEN_MODE_WATCHFACE = 0,
    SCREEN_MODE_DASHBOARD,
    SCREEN_MODE_NAVIGATION,
    SCREEN_MODE_HEALTH,
    SCREEN_MODE_ENVIRONMENT,
    SCREEN_MODE_SETTINGS,
    SCREEN_MODE_COUNT
} ScreenMode;

// --- DISPLAY MANAGER CLASS ---
class DisplayManager {
public:
    DisplayManager();
    bool begin();
    void startTask(UBaseType_t priority = 1);
    void stopTask();

    void updateData(bool wifiConnected,
                    int stepCount, float distance,
                    int heartRate, int spo2,
                    float temperature, float pressure,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    const struct tm* timeinfo, bool timeInitialized,
                    const NavigationInfo& navInfo);

    void toggleScreen();
    void switchDisplayMode();
    void toggleTheme();
    bool isScreenOn() const;
    
    // Animation control
    void startAnimation(AnimationType type);
    bool isAnimating() const;
    
    // Force a complete redraw of the screen
    void forceRedraw();

private:
    Adafruit_ST7789 tft;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex;

    // Display state
    bool screenOn;
    int currentTheme;  // 0: Dark, 1: Light
    ScreenMode currentScreenMode;
    
    // Animation state
    AnimationType currentAnimation;
    unsigned long animStartTime;
    bool animating;
    int animProgress;  // 0-100

    // Data state
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
    NavigationInfo navInfoLocal;
    
    // Optimization variables
    String lastTimeString;
    String lastDateString;
    String lastDayString;
    int lastSecondAngle;
    int lastDisplayedSteps;
    int lastDisplayedHR;
    int lastDisplayedSpO2;
    float lastTempEnv;
    float lastPresEnv;
    String lastTimeEnv;
    float lastAxIMU, lastAyIMU, lastAzIMU;
    float lastGxIMU, lastGyIMU, lastGzIMU;
    bool lastWifiState;
    bool needsRedrawCurrentScreen;
    String lastNavDirection;
    String lastNavDistance;
    String lastNavStreet;

    // Watchface variables
    float watchX[360], watchY[360];  // Coordinates for watchface
    
    // Task function
    static void taskFunction(void* pvParameters);
    void updateDisplay();

    // Screen drawing functions
    void drawWatchFaceScreen(bool redrawStatic);
    void drawDashboardScreen(bool redrawStatic);
    void drawHealthScreen(bool redrawStatic);
    void drawEnvironmentScreen(bool redrawStatic);
    void drawSettingsScreen(bool redrawStatic);
    void drawNavigationScreen(bool redrawStatic);

    // Helper drawing functions
    void clearScreen();
    void drawHeader(const String& title);
    void drawFooter();
    void drawWifiIcon(int x, int y);
    void drawBatteryIcon(int x, int y, int percentage);
    void drawCard(int x, int y, int width, int height, const String& title, uint16_t color);
    void drawButton(int x, int y, int width, int height, const String& label, uint16_t color, bool pressed = false);
    void drawProgressBar(int x, int y, int width, int height, float percentage, uint16_t color);
    void drawWatchFace();
    void drawNavigationIcon(int x, int y, int size, const String& direction, uint16_t color);
    
    // Animation helpers
    void updateAnimation();
    void applyAnimationEffect(int x, int y, int width, int height);
    
    // Optimized drawing functions
    void drawStringOptimized(const String& text, int x, int y, const GFXfont* fontPtr, 
                            uint16_t textColor, uint16_t bgColor, uint8_t datum, 
                            String& lastText, int clearWidth = -1, int clearHeight = -1);
    
    void drawFloatOptimized(float value, int decimalPlaces, const String& unit, 
                           int x, int y, const GFXfont* fontPtr, 
                           uint16_t textColor, uint16_t bgColor, uint8_t datum, 
                           float& lastValue, const char* format = nullptr, 
                           int clearWidth = -1, int clearHeight = -1);
    
    void drawIntOptimized(int value, const String& unit, int x, int y, 
                         const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, 
                         uint8_t datum, int& lastValue, const char* defaultText = "--", 
                         int clearWidth = -1, int clearHeight = -1, int validThreshold = -9999);
};

#endif // DISPLAY_MANAGER_H