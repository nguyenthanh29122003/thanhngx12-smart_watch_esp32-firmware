#include "DisplayManager.h"
#include "Config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <cmath>

// --- TFT Pin Definitions ---
#define TFT_CS     7
#define TFT_DC     39
#define TFT_RST    40
#define TFT_BL     45

// --- Backlight Control ---
#define TFT_BACKLIGHT_ON HIGH

// --- TFT_eSPI Datum Definitions (for compatibility) ---
#define TL_DATUM 0 // Top left (default)
#define TC_DATUM 1 // Top centre
#define TR_DATUM 2 // Top right
#define ML_DATUM 3 // Middle left
#define MC_DATUM 4 // Middle centre
#define MR_DATUM 5 // Middle right
#define BL_DATUM 6 // Bottom left
#define BC_DATUM 7 // Bottom centre
#define BR_DATUM 8 // Bottom right
#define L_BASELINE  9 // Left character baseline
#define C_BASELINE 10 // Centre character baseline
#define R_BASELINE 11 // Right character baseline

// --- UI Constants ---
#define WATCHFACE_RADIUS (SCREEN_WIDTH / 2 - 10)
#define NUM_POINTS 360

// --- Static Data Arrays ---
const String daysOfWeek[7] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
const String daysOfWeekShort[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const String monthsShort[12] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};

// --- Font Selection ---
const GFXfont* FONT_SMALL = &FreeSans9pt7b;
const GFXfont* FONT_SMALL_BOLD = &FreeSansBold9pt7b;
const GFXfont* FONT_LARGE = &FreeSansBold12pt7b;
const GFXfont* FONT_MONO = &FreeMonoBold9pt7b;
const GFXfont* FONT_MONO_SMALL = &FreeMono9pt7b;

// --- Constructor ---
DisplayManager::DisplayManager()
    : tft(TFT_CS, TFT_DC, TFT_RST),
      taskHandle(NULL), screenOn(true), currentTheme(0),
      currentScreenMode(SCREEN_MODE_WATCHFACE),
      currentAnimation(ANIM_NONE), animating(false), animProgress(0),
      timeInitializedLocal(false), wifiConnectedLocal(false),
      stepCountLocal(0), distanceLocal(0.0f), heartRateLocal(0), spo2Local(-999),
      temperatureLocal(NAN), pressureLocal(NAN),
      axLocal(0.0f), ayLocal(0.0f), azLocal(0.0f), gxLocal(0.0f), gyLocal(0.0f), gzLocal(0.0f),
      lastTimeString(""), lastDateString(""), lastDayString(""), lastSecondAngle(-1),
      lastDisplayedSteps(-1), lastDisplayedHR(-1), lastDisplayedSpO2(-1000),
      lastTempEnv(NAN), lastPresEnv(NAN), lastTimeEnv(""),
      lastAxIMU(NAN), lastAyIMU(NAN), lastAzIMU(NAN), lastGxIMU(NAN), lastGyIMU(NAN), lastGzIMU(NAN),
      lastWifiState(false),
      needsRedrawCurrentScreen(true)
{
    memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) { Serial.println("CRITICAL: Failed to create Display data mutex!"); }
}

// --- begin() ---
bool DisplayManager::begin() {
    Serial.println("Initializing Display Manager (Adafruit ST7789)...");

    // Initialize Backlight
    if (TFT_BL >= 0) {
        Serial.printf("Initializing Backlight pin %d\n", TFT_BL);
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
    } else {
        Serial.println("Backlight pin not defined.");
    }

    // Initialize TFT
    tft.init(SCREEN_WIDTH, SCREEN_HEIGHT);
    Serial.println("TFT initialized");

    tft.setRotation(0);
    tft.fillScreen(DARK_BACKGROUND);

    Serial.println("Calculating Watch Face UI points...");
    // Calculate watchface points
    for (int i = 0; i < NUM_POINTS; i++) {
        float angleRad = radians(i - 90);
        watchX[i] = (WATCHFACE_RADIUS * cos(angleRad)) + CENTER_X;
        watchY[i] = (WATCHFACE_RADIUS * sin(angleRad)) + CENTER_Y;
    }
    
    Serial.println("Display Manager Initialized.");
    return true;
}

// --- Task Management ---
void DisplayManager::startTask(UBaseType_t priority) {
    xTaskCreate(taskFunction, "DisplayTask", 4096, this, priority, &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating Display Task!");
    else Serial.println("Display Task started.");
}

void DisplayManager::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle; taskHandle = NULL;
        tft.enableDisplay(false);
        if (TFT_BL >= 0) digitalWrite(TFT_BL, !TFT_BACKLIGHT_ON);
        vTaskDelete(tempHandle);
        Serial.println("Display task stopped.");
    }
}

void DisplayManager::taskFunction(void* pvParameters) {
    DisplayManager* instance = static_cast<DisplayManager*>(pvParameters);
    if (instance == nullptr) { vTaskDelete(NULL); return; }
    Serial.println("Display Task running...");
    while (true) {
        instance->updateDisplay();
        vTaskDelay(pdMS_TO_TICKS(16)); // ~60fps
    }
}

// --- Data Update ---
void DisplayManager::updateData(bool wifiConnected,
                                int stepCount, float distance,
                                int heartRate, int spo2,
                                float temperature, float pressure,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                const struct tm* timeinfo, bool timeInitialized) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        wifiConnectedLocal = wifiConnected;
        timeInitializedLocal = timeInitialized;
        if (timeInitialized && timeinfo != nullptr) timeinfoLocal = *timeinfo;
        else memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
        stepCountLocal = stepCount; distanceLocal = distance;
        heartRateLocal = heartRate; spo2Local = spo2;
        temperatureLocal = temperature; pressureLocal = pressure;
        axLocal = ax; ayLocal = ay; azLocal = az;
        gxLocal = gx; gyLocal = gy; gzLocal = gz;
        xSemaphoreGive(dataMutex);
    } else { Serial.println("Timeout taking display data mutex in updateData!"); }
}

// --- Animation Control ---
void DisplayManager::startAnimation(AnimationType type) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentAnimation = type;
        animStartTime = millis();
        animating = true;
        animProgress = 0;
        xSemaphoreGive(dataMutex);
    }
}

bool DisplayManager::isAnimating() const {
    bool result = false;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        result = animating;
        xSemaphoreGive(dataMutex);
    }
    return result;
}

void DisplayManager::updateAnimation() {
    if (!animating) return;
    
    unsigned long elapsed = millis() - animStartTime;
    if (elapsed >= ANIM_DURATION) {
        animating = false;
        animProgress = 100;
        return;
    }
    
    // Calculate progress (0-100)
    animProgress = (elapsed * 100) / ANIM_DURATION;
}

void DisplayManager::applyAnimationEffect(int x, int y, int width, int height) {
    if (!animating) return;
    
    // Apply different animation effects based on type
    switch (currentAnimation) {
        case ANIM_FADE_IN: {
            // Create a semi-transparent overlay that fades out
            uint8_t alpha = 255 - (255 * animProgress / 100);
            uint16_t overlayColor = currentTheme == 0 ? DARK_BACKGROUND : LIGHT_BACKGROUND;
            // Apply alpha blending (simplified)
            for (int i = 0; i < width; i += 4) {  // Skip pixels for performance
                for (int j = 0; j < height; j += 4) {
                    if (random(100) < alpha) {  // Dithering effect
                        tft.drawPixel(x + i, y + j, overlayColor);
                    }
                }
            }
            break;
        }
        
        case ANIM_SLIDE_LEFT: {
            // Slide content from right to left
            int offset = width - (width * animProgress / 100);
            if (offset > 0) {
                uint16_t bgColor = currentTheme == 0 ? DARK_BACKGROUND : LIGHT_BACKGROUND;
                tft.fillRect(x + width - offset, y, offset, height, bgColor);
            }
            break;
        }
        
        case ANIM_SLIDE_RIGHT: {
            // Slide content from left to right
            int offset = width - (width * animProgress / 100);
            if (offset > 0) {
                uint16_t bgColor = currentTheme == 0 ? DARK_BACKGROUND : LIGHT_BACKGROUND;
                tft.fillRect(x, y, offset, height, bgColor);
            }
            break;
        }
        
        default:
            break;
    }
}

// --- Main Update Loop ---
void DisplayManager::updateDisplay() {
    bool localScreenOn = false;
    ScreenMode localScreenMode;
    bool needsRedraw = false;

    // Safely get current state
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localScreenOn = screenOn;
        localScreenMode = currentScreenMode;
        needsRedraw = needsRedrawCurrentScreen;
        if (needsRedraw) needsRedrawCurrentScreen = false;
        
        // Update animation state
        updateAnimation();
        
        xSemaphoreGive(dataMutex);
    } else {
        return;
    }

    // If screen is off, do nothing
    if (!localScreenOn) return;

    // If a full redraw is needed, reset optimization variables
    if (needsRedraw) {
        clearScreen();
        lastTimeString = ""; lastDateString = ""; lastDayString = ""; lastSecondAngle = -1;
        lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
        lastTempEnv = NAN; lastPresEnv = NAN; lastTimeEnv = "";
        lastAxIMU = lastAyIMU = lastAzIMU = lastGxIMU = lastGyIMU = lastGzIMU = NAN;
        lastWifiState = !wifiConnectedLocal;
    }

    // Call the drawing function for the current mode
    switch (localScreenMode) {
        case SCREEN_MODE_WATCHFACE:     drawWatchFaceScreen(needsRedraw); break;
        case SCREEN_MODE_DASHBOARD:     drawDashboardScreen(needsRedraw); break;
        case SCREEN_MODE_HEALTH:        drawHealthScreen(needsRedraw); break;
        case SCREEN_MODE_ENVIRONMENT:   drawEnvironmentScreen(needsRedraw); break;
        case SCREEN_MODE_SETTINGS:      drawSettingsScreen(needsRedraw); break;
        default: break;
    }
}

// --- Screen Drawing Functions ---

void DisplayManager::drawWatchFaceScreen(bool redrawStatic) {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    bool localWifiConnected = false;

    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localWifiConnected = wifiConnectedLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    // Get theme colors
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t accentColor = (currentTheme == 0) ? DARK_PRIMARY : LIGHT_PRIMARY;
    uint16_t secondaryColor = (currentTheme == 0) ? DARK_SECONDARY : LIGHT_SECONDARY;

    // Clear screen if needed
    if (redrawStatic) {
        tft.fillScreen(bgColor);
        
        // Draw watchface
        drawWatchFace();
        
        // Draw status icons at top
        drawWifiIcon(10, 10);
        drawBatteryIcon(SCREEN_WIDTH - 25, 10, 85); // 85% battery
    }

    // Handle invalid time display
    if (!isTimeValid) {
        if (lastTimeString != "WaitingWF") {
            drawStringOptimized("Waiting for", CENTER_X, CENTER_Y - 10, FONT_SMALL, 
                               textColor, bgColor, MC_DATUM, lastTimeString, SCREEN_WIDTH);
            
            String dummyLast = "";
            drawStringOptimized("Time Sync...", CENTER_X, CENTER_Y + 10, FONT_SMALL, 
                               textColor, bgColor, MC_DATUM, dummyLast, SCREEN_WIDTH);
            lastTimeString = "WaitingWF";
            lastSecondAngle = -1; lastDateString = ""; lastDayString = "";
        }
        return;
    }

    // If we were waiting, force redraw
    if (lastTimeString == "WaitingWF") {
        redrawStatic = true;
        lastTimeString = ""; lastDateString = ""; lastDayString = ""; lastSecondAngle = -1;
    }

    // Format time and date strings
    int hour = localTimeCopy.tm_hour;
    int minute = localTimeCopy.tm_min;
    int second = localTimeCopy.tm_sec;
    int angle = second * 6; // Angle for second hand (0-360)
    
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", hour, minute, second);
    String currentTime = String(timeStr);
    
    String currentDay = daysOfWeek[localTimeCopy.tm_wday];
    String currentDate = String(localTimeCopy.tm_mday) + " " + 
                         monthsShort[localTimeCopy.tm_mon] + " " +
                         String(1900 + localTimeCopy.tm_year);

    // Check what changed
    bool timeChanged = (currentTime != lastTimeString);
    bool dayChanged = (currentDay != lastDayString);
    bool dateChanged = (currentDate != lastDateString);
    bool angleChanged = (angle != lastSecondAngle);

    // Draw digital time
    if (timeChanged || redrawStatic) {
        drawStringOptimized(currentTime, CENTER_X, CENTER_Y + 40, FONT_LARGE, 
                           accentColor, bgColor, MC_DATUM, lastTimeString, 120, 30);
    }
    
    // Draw date
    if (dateChanged || redrawStatic) {
        drawStringOptimized(currentDate, CENTER_X, CENTER_Y + 70, FONT_SMALL, 
                           textColor, bgColor, MC_DATUM, lastDateString, 120, 20);
    }
    
    // Draw day of week
    if (dayChanged || redrawStatic) {
        drawStringOptimized(currentDay, CENTER_X, 20, FONT_SMALL_BOLD, 
                           secondaryColor, bgColor, MC_DATUM, lastDayString, 120, 20);
    }
    
    // Draw analog hands
    // Hour hand
    float hourAngle = ((hour % 12) * 30 + (minute / 2)) * PI / 180.0;
    int hourHandLength = WATCHFACE_RADIUS - 30;
    int hourX = CENTER_X + hourHandLength * cos(hourAngle - PI/2);
    int hourY = CENTER_Y + hourHandLength * sin(hourAngle - PI/2);
    
    // Minute hand
    float minuteAngle = (minute * 6) * PI / 180.0;
    int minuteHandLength = WATCHFACE_RADIUS - 15;
    int minuteX = CENTER_X + minuteHandLength * cos(minuteAngle - PI/2);
    int minuteY = CENTER_Y + minuteHandLength * sin(minuteAngle - PI/2);
    
    // Second hand
    float secondAngle = (second * 6) * PI / 180.0;
    int secondHandLength = WATCHFACE_RADIUS - 10;
    int secondX = CENTER_X + secondHandLength * cos(secondAngle - PI/2);
    int secondY = CENTER_Y + secondHandLength * sin(secondAngle - PI/2);
    
    // Redraw hands if time changed or full redraw
    if (timeChanged || angleChanged || redrawStatic) {
        // Clear previous hands by redrawing the center of the watchface
        tft.fillCircle(CENTER_X, CENTER_Y, WATCHFACE_RADIUS - 10, bgColor);
        
        // Draw hour and minute hands
        tft.drawLine(CENTER_X, CENTER_Y, hourX, hourY, textColor);
        tft.drawLine(CENTER_X-1, CENTER_Y, hourX-1, hourY, textColor); // Thicker line
        
        tft.drawLine(CENTER_X, CENTER_Y, minuteX, minuteY, textColor);
        tft.drawLine(CENTER_X+1, CENTER_Y, minuteX+1, minuteY, textColor); // Thicker line
        
        // Draw second hand with accent color
        tft.drawLine(CENTER_X, CENTER_Y, secondX, secondY, secondaryColor);
        
        // Draw center circle
        tft.fillCircle(CENTER_X, CENTER_Y, 4, accentColor);
        tft.fillCircle(CENTER_X, CENTER_Y, 2, textColor);
    }
    
    // Update last values
    lastTimeString = currentTime;
    lastDayString = currentDay;
    lastDateString = currentDate;
    lastSecondAngle = angle;
    
    // Apply animation effect if animating
    if (animating) {
        applyAnimationEffect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    }
}

void DisplayManager::drawDashboardScreen(bool redrawStatic) {
    // Get data
    struct tm localTimeCopy;
    bool isTimeValid = false;
    int localSteps = 0;
    int localHR = 0;
    int localSpO2 = -999;
    float localTemp = NAN;
    
    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localSteps = stepCountLocal;
        localHR = heartRateLocal;
        localSpO2 = spo2Local;
        localTemp = temperatureLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }
    
    // Get theme colors
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t cardColor = (currentTheme == 0) ? DARK_SURFACE : LIGHT_SURFACE;
    uint16_t accentColor = (currentTheme == 0) ? DARK_PRIMARY : LIGHT_PRIMARY;
    
    // Draw header and footer if needed
    if (redrawStatic) {
        clearScreen();
        drawHeader("Dashboard");
        drawFooter();
    }
    
    // Draw cards for each metric
    int cardWidth = SCREEN_WIDTH - 2*CARD_MARGIN;
    int cardHeight = 45;
    int cardY = HEADER_HEIGHT + CARD_MARGIN;
    
    // Steps card
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "Steps", COLOR_STEPS);
    drawIntOptimized(localSteps, "", CARD_MARGIN + cardWidth - 10, cardY + 30, 
                    FONT_LARGE, textColor, cardColor, TR_DATUM, 
                    lastDisplayedSteps, "--", cardWidth - 20);
    
    // Heart rate card
    cardY += cardHeight + CARD_MARGIN;
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "Heart Rate", COLOR_HEART);
    drawIntOptimized(localHR, " BPM", CARD_MARGIN + cardWidth - 10, cardY + 30, 
                    FONT_LARGE, textColor, cardColor, TR_DATUM, 
                    lastDisplayedHR, "--", cardWidth - 20);
    
    // SpO2 card
    cardY += cardHeight + CARD_MARGIN;
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "SpO2", COLOR_SPO2);
    drawIntOptimized(localSpO2, "%", CARD_MARGIN + cardWidth - 10, cardY + 30, 
                    FONT_LARGE, textColor, cardColor, TR_DATUM, 
                    lastDisplayedSpO2, "--", cardWidth - 20);
    
    // Temperature card
    cardY += cardHeight + CARD_MARGIN;
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "Temperature", COLOR_TEMP);
    drawFloatOptimized(localTemp, 1, "°C", CARD_MARGIN + cardWidth - 10, cardY + 30, 
                      FONT_LARGE, textColor, cardColor, TR_DATUM, 
                      lastTempEnv, "%.1f", cardWidth - 20);
    
    // Apply animation effect if animating
    if (animating) {
        applyAnimationEffect(0, HEADER_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - HEADER_HEIGHT - FOOTER_HEIGHT);
    }
}

void DisplayManager::drawHealthScreen(bool redrawStatic) {
    // Get data
    int localHR = 0;
    int localSpO2 = -999;
    
    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localHR = heartRateLocal;
        localSpO2 = spo2Local;
        xSemaphoreGive(dataMutex);
    } else { return; }
    
    // Get theme colors
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t cardColor = (currentTheme == 0) ? DARK_SURFACE : LIGHT_SURFACE;
    
    // Draw header and footer if needed
    if (redrawStatic) {
        clearScreen();
        drawHeader("Health");
        drawFooter();
    }
    
    // Draw large heart rate display
    int cardWidth = SCREEN_WIDTH - 2*CARD_MARGIN;
    int cardHeight = 80;
    int cardY = HEADER_HEIGHT + CARD_MARGIN;
    
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "Heart Rate", COLOR_HEART);
    
    // Draw heart rate value
    char hrStr[10];
    sprintf(hrStr, "%d", localHR > 0 ? localHR : 0);
    String hrValue = String(hrStr);
    String lastHrValue = String(lastDisplayedHR > 0 ? lastDisplayedHR : 0);
    
    drawStringOptimized(hrValue, CENTER_X, cardY + 40, FONT_LARGE, 
                       COLOR_HEART, cardColor, MC_DATUM, lastHrValue, 60, 30);
    
    // Draw BPM label
    String bpmLabel = "BPM";
    String lastBpmLabel = "";
    drawStringOptimized(bpmLabel, CENTER_X, cardY + 65, FONT_SMALL, 
                       textColor, cardColor, MC_DATUM, lastBpmLabel, 40, 20);
    
    // Draw SpO2 display
    cardY += cardHeight + CARD_MARGIN;
    cardHeight = 80;
    
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "Blood Oxygen", COLOR_SPO2);
    
    // Draw SpO2 value
    char spo2Str[10];
    sprintf(spo2Str, "%d", localSpO2 > 0 ? localSpO2 : 0);
    String spo2Value = String(spo2Str);
    String lastSpo2Value = String(lastDisplayedSpO2 > 0 ? lastDisplayedSpO2 : 0);
    
    drawStringOptimized(spo2Value, CENTER_X, cardY + 40, FONT_LARGE, 
                       COLOR_SPO2, cardColor, MC_DATUM, lastSpo2Value, 60, 30);
    
    // Draw % label
    String percentLabel = "%";
    String lastPercentLabel = "";
    drawStringOptimized(percentLabel, CENTER_X, cardY + 65, FONT_SMALL, 
                       textColor, cardColor, MC_DATUM, lastPercentLabel, 20, 20);
    
    // Update last values
    lastDisplayedHR = localHR;
    lastDisplayedSpO2 = localSpO2;
    
    // Apply animation effect if animating
    if (animating) {
        applyAnimationEffect(0, HEADER_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - HEADER_HEIGHT - FOOTER_HEIGHT);
    }
}

void DisplayManager::drawEnvironmentScreen(bool redrawStatic) {
    // Get data
    float localTemp = NAN;
    float localPres = NAN;
    
    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localTemp = temperatureLocal;
        localPres = pressureLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }
    
    // Get theme colors
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t cardColor = (currentTheme == 0) ? DARK_SURFACE : LIGHT_SURFACE;
    
    // Draw header and footer if needed
    if (redrawStatic) {
        clearScreen();
        drawHeader("Environment");
        drawFooter();
    }
    
    // Draw temperature display
    int cardWidth = SCREEN_WIDTH - 2*CARD_MARGIN;
    int cardHeight = 80;
    int cardY = HEADER_HEIGHT + CARD_MARGIN;
    
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "Temperature", COLOR_TEMP);
    
    // Draw temperature value and unit
    drawFloatOptimized(localTemp, 1, "°C", CENTER_X, cardY + 45, 
                      FONT_LARGE, COLOR_TEMP, cardColor, MC_DATUM, 
                      lastTempEnv, "%.1f", 100, 30);
    
    // Draw pressure display
    cardY += cardHeight + CARD_MARGIN;
    cardHeight = 80;
    
    drawCard(CARD_MARGIN, cardY, cardWidth, cardHeight, "Pressure", COLOR_PRESSURE);
    
    // Convert pressure to hPa for display
    float presHpa = isnan(localPres) ? NAN : localPres / 100.0f;
    float lastPresHpa = isnan(lastPresEnv) ? NAN : lastPresEnv / 100.0f;
    
    // Draw pressure value and unit
    drawFloatOptimized(presHpa, 1, " hPa", CENTER_X, cardY + 45, 
                      FONT_LARGE, COLOR_PRESSURE, cardColor, MC_DATUM, 
                      lastPresHpa, "%.1f", 120, 30);
    
    // Update last values
    lastTempEnv = localTemp;
    lastPresEnv = localPres;
    
    // Apply animation effect if animating
    if (animating) {
        applyAnimationEffect(0, HEADER_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - HEADER_HEIGHT - FOOTER_HEIGHT);
    }
}

void DisplayManager::drawSettingsScreen(bool redrawStatic) {
    // Get theme colors
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t primaryColor = (currentTheme == 0) ? DARK_PRIMARY : LIGHT_PRIMARY;
    uint16_t secondaryColor = (currentTheme == 0) ? DARK_SECONDARY : LIGHT_SECONDARY;
    
    // Draw header and footer if needed
    if (redrawStatic) {
        clearScreen();
        drawHeader("Settings");
        drawFooter();
    }
    
    // Draw settings buttons
    int buttonWidth = SCREEN_WIDTH - 2*CARD_MARGIN;
    int buttonHeight = 35;
    int buttonY = HEADER_HEIGHT + CARD_MARGIN;
    int buttonSpacing = 10;
    
    // Theme toggle button
    drawButton(CARD_MARGIN, buttonY, buttonWidth, buttonHeight, 
              "Toggle Theme", primaryColor);
    
    // Display toggle button
    buttonY += buttonHeight + buttonSpacing;
    drawButton(CARD_MARGIN, buttonY, buttonWidth, buttonHeight, 
              "Display On/Off", secondaryColor);
    
    // Reset button
    buttonY += buttonHeight + buttonSpacing;
    drawButton(CARD_MARGIN, buttonY, buttonWidth, buttonHeight, 
              "Reset Device", COLOR_WARNING);
    
    // About section
    buttonY += buttonHeight + 2*buttonSpacing;
    
    // Draw about text
    String aboutTitle = "About";
    String lastAboutTitle = "";
    drawStringOptimized(aboutTitle, CENTER_X, buttonY, FONT_SMALL_BOLD, 
                       textColor, bgColor, MC_DATUM, lastAboutTitle, 60, 20);
    
    buttonY += 25;
    String versionText = "Firmware v1.0.0";
    String lastVersionText = "";
    drawStringOptimized(versionText, CENTER_X, buttonY, FONT_SMALL, 
                       textColor, bgColor, MC_DATUM, lastVersionText, 120, 20);
    
    // Apply animation effect if animating
    if (animating) {
        applyAnimationEffect(0, HEADER_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - HEADER_HEIGHT - FOOTER_HEIGHT);
    }
}

// --- Helper Drawing Functions ---

void DisplayManager::clearScreen() {
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    tft.fillScreen(bgColor);
}

void DisplayManager::drawHeader(const String& title) {
    uint16_t headerBgColor = (currentTheme == 0) ? DARK_PRIMARY : LIGHT_PRIMARY;
    uint16_t headerTextColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    
    // Draw header background
    tft.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, headerBgColor);
    
    // Draw title
    tft.setFont(FONT_SMALL_BOLD);
    tft.setTextColor(headerTextColor);
    
    // Calculate text position for centering
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    
    int textX = CENTER_X - w/2 - x1;
    int textY = (HEADER_HEIGHT - h)/2 - y1;
    
    tft.setCursor(textX, textY);
    tft.print(title);
    
    // Draw WiFi and battery icons
    drawWifiIcon(10, (HEADER_HEIGHT - 12)/2);
    drawBatteryIcon(SCREEN_WIDTH - 25, (HEADER_HEIGHT - 12)/2, 85); // 85% battery
}

void DisplayManager::drawFooter() {
    uint16_t footerBgColor = (currentTheme == 0) ? DARK_SURFACE : LIGHT_SURFACE;
    uint16_t footerTextColor = (currentTheme == 0) ? DARK_TEXT_SECONDARY : LIGHT_TEXT_SECONDARY;
    
    // Draw footer background
    tft.fillRect(0, SCREEN_HEIGHT - FOOTER_HEIGHT, SCREEN_WIDTH, FOOTER_HEIGHT, footerBgColor);
    
    // Get current time for footer
    struct tm localTimeCopy;
    bool isTimeValid = false;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        xSemaphoreGive(dataMutex);
    }
    
    // Draw time in footer
    String timeStr;
    if (isTimeValid) {
        char timeBuf[6];
        sprintf(timeBuf, "%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min);
        timeStr = String(timeBuf);
    } else {
        timeStr = "--:--";
    }
    
    tft.setFont(nullptr); // Use default font for footer
    tft.setTextColor(footerTextColor);
    
    // Calculate text position for centering
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
    
    int textX = CENTER_X - w/2 - x1;
    int textY = SCREEN_HEIGHT - FOOTER_HEIGHT/2 - h/2 - y1;
    
    tft.setCursor(textX, textY);
    tft.print(timeStr);
}

void DisplayManager::drawWifiIcon(int x, int y) {
    bool localWifiConnected;
    
    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        localWifiConnected = wifiConnectedLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }
    
    uint16_t iconColor;
    if (localWifiConnected) {
        iconColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    } else {
        iconColor = (currentTheme == 0) ? DARK_TEXT_SECONDARY : LIGHT_TEXT_SECONDARY;
    }
    
    // Draw WiFi icon (simplified)
    if (localWifiConnected) {
        // Connected - draw 3 arcs
        tft.drawCircle(x + 6, y + 10, 8, iconColor);
        tft.drawCircle(x + 6, y + 10, 5, iconColor);
        tft.fillCircle(x + 6, y + 10, 2, iconColor);
    } else {
        // Disconnected - draw crossed out icon
        tft.drawCircle(x + 6, y + 10, 8, iconColor);
        tft.drawLine(x, y, x + 12, y + 12, iconColor);
    }
    
    lastWifiState = localWifiConnected;
}

void DisplayManager::drawBatteryIcon(int x, int y, int percentage) {
    uint16_t iconColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t fillColor;
    
    // Determine fill color based on battery level
    if (percentage > 60) {
        fillColor = COLOR_SUCCESS;
    } else if (percentage > 20) {
        fillColor = COLOR_WARNING;
    } else {
        fillColor = COLOR_ERROR;
    }
    
    // Draw battery outline
    tft.drawRect(x, y, 20, 10, iconColor);
    tft.fillRect(x + 20, y + 2, 2, 6, iconColor); // Battery nub
    
    // Draw battery fill based on percentage
    int fillWidth = map(percentage, 0, 100, 0, 18);
    if (fillWidth > 0) {
        tft.fillRect(x + 1, y + 1, fillWidth, 8, fillColor);
    }
}

void DisplayManager::drawCard(int x, int y, int width, int height, const String& title, uint16_t color) {
    uint16_t cardBgColor = (currentTheme == 0) ? DARK_SURFACE : LIGHT_SURFACE;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    
    // Draw card background with rounded corners
    tft.fillRoundRect(x, y, width, height, CARD_CORNER_RADIUS, cardBgColor);
    
    // Draw colored accent border
    tft.drawRoundRect(x, y, width, height, CARD_CORNER_RADIUS, color);
    
    // Draw title if provided
    if (title.length() > 0) {
        tft.setFont(FONT_SMALL);
        tft.setTextColor(color);
        
        tft.setCursor(x + CARD_PADDING, y + CARD_PADDING + 10);
        tft.print(title);
    }
}

void DisplayManager::drawButton(int x, int y, int width, int height, const String& label, uint16_t color, bool pressed) {
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    
    // Draw button background with rounded corners
    uint16_t bgColor = pressed ? (color & 0x7BEF) : color; // Darken when pressed
    
    tft.fillRoundRect(x, y, width, height, CARD_CORNER_RADIUS, bgColor);
    
    // Draw button label
    tft.setFont(FONT_SMALL);
    tft.setTextColor(textColor);
    
    // Calculate text position for centering
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
    
    int textX = x + width/2 - w/2 - x1;
    int textY = y + height/2 - h/2 - y1;
    
    tft.setCursor(textX, textY);
    tft.print(label);
    
    // Draw button highlight effect
    if (!pressed) {
        // Top highlight
        tft.drawFastHLine(x + 2, y + 1, width - 4, textColor & 0xF7DE); // 50% transparent white
        tft.drawFastVLine(x + 1, y + 2, 2, textColor & 0xF7DE);
        tft.drawFastVLine(x + width - 2, y + 2, 2, textColor & 0xF7DE);
    }
}

void DisplayManager::drawProgressBar(int x, int y, int width, int height, float percentage, uint16_t color) {
    uint16_t bgColor = (currentTheme == 0) ? DARK_SURFACE : LIGHT_SURFACE;
    
    // Constrain percentage to 0-100
    percentage = constrain(percentage, 0.0f, 100.0f);
    
    // Draw background
    tft.fillRoundRect(x, y, width, height, height/2, bgColor);
    
    // Draw border
    tft.drawRoundRect(x, y, width, height, height/2, color);
    
    // Calculate fill width
    int fillWidth = (percentage / 100.0f) * (width - 4);
    if (fillWidth > 0) {
        // Draw fill with rounded ends
        if (fillWidth < height) {
            // For very small fills, use a circle
            tft.fillCircle(x + 2 + fillWidth/2, y + height/2, fillWidth/2, color);
        } else {
            // Normal fill with rounded ends
            tft.fillRoundRect(x + 2, y + 2, fillWidth, height - 4, (height - 4)/2, color);
        }
    }
}

void DisplayManager::drawWatchFace() {
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t faceColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t accentColor = (currentTheme == 0) ? DARK_PRIMARY : LIGHT_PRIMARY;
    
    // Draw outer circle
    tft.drawCircle(CENTER_X, CENTER_Y, WATCHFACE_RADIUS, faceColor);
    tft.drawCircle(CENTER_X, CENTER_Y, WATCHFACE_RADIUS - 1, faceColor); // Double thickness
    
    // Draw hour markers
    for (int i = 0; i < 12; i++) {
        float angle = i * 30 * PI / 180;
        int x1 = CENTER_X + (WATCHFACE_RADIUS - 10) * cos(angle - PI/2);
        int y1 = CENTER_Y + (WATCHFACE_RADIUS - 10) * sin(angle - PI/2);
        int x2 = CENTER_X + WATCHFACE_RADIUS * cos(angle - PI/2);
        int y2 = CENTER_Y + WATCHFACE_RADIUS * sin(angle - PI/2);
        
        // Draw thicker markers for 12, 3, 6, 9
        if (i % 3 == 0) {
            tft.drawLine(x1, y1, x2, y2, accentColor);
            tft.drawLine(x1+1, y1, x2+1, y2, accentColor); // Thicker line
        } else {
            tft.drawLine(x1, y1, x2, y2, faceColor);
        }
    }
    
    // Draw minute markers
    for (int i = 0; i < 60; i++) {
        if (i % 5 != 0) { // Skip hour markers
            float angle = i * 6 * PI / 180;
            int x1 = CENTER_X + (WATCHFACE_RADIUS - 5) * cos(angle - PI/2);
            int y1 = CENTER_Y + (WATCHFACE_RADIUS - 5) * sin(angle - PI/2);
            int x2 = CENTER_X + WATCHFACE_RADIUS * cos(angle - PI/2);
            int y2 = CENTER_Y + WATCHFACE_RADIUS * sin(angle - PI/2);
            
            tft.drawLine(x1, y1, x2, y2, faceColor);
        }
    }
    
    // Draw center dot
    tft.fillCircle(CENTER_X, CENTER_Y, 4, accentColor);
    tft.fillCircle(CENTER_X, CENTER_Y, 2, faceColor);
}

// --- Optimized Drawing Functions ---

void DisplayManager::drawStringOptimized(const String& text, int x, int y, const GFXfont* fontPtr, 
                                        uint16_t textColor, uint16_t bgColor, uint8_t datum, 
                                        String& lastText, int clearWidth, int clearHeight) {
    // Check if text actually changed or if a full redraw is forced
    if (text == lastText && !needsRedrawCurrentScreen) {
        return; // Nothing to draw
    }

    // Set font and color
    tft.setFont(fontPtr);
    tft.setTextColor(textColor);

    // Get bounds of the new text
    int16_t x1_new = 0, y1_new = 0;
    uint16_t w_new = 0, h_new = 0;
    if (!text.isEmpty()) {
       tft.getTextBounds(text, 0, 0, &x1_new, &y1_new, &w_new, &h_new);
    } else {
        // If new text is empty, use defaults for clearing
        if (clearWidth < 0) clearWidth = 30;
        if (clearHeight < 0) clearHeight = (fontPtr ? 20 : 8) + 4;
    }

    // If clearWidth/Height not provided, use new text bounds + padding
    if (clearWidth < 0) clearWidth = w_new + 4;
    if (clearHeight < 0) clearHeight = h_new + 4;

    // Calculate clear area and cursor position based on datum
    int clearX = x, clearY = y, cursorX = x, cursorY = y;
    
    // Horizontal alignment
    switch (datum) {
        case TC_DATUM:
        case MC_DATUM:
        case BC_DATUM:
            clearX = x - clearWidth / 2;
            cursorX = x - w_new / 2 - x1_new;
            break;
        case TR_DATUM:
        case MR_DATUM:
        case BR_DATUM:
            clearX = x - clearWidth;
            cursorX = x - w_new - x1_new;
            break;
    }

    // Vertical alignment
    switch (datum) {
        case ML_DATUM:
        case MC_DATUM:
        case MR_DATUM:
            clearY = y - clearHeight / 2;
            cursorY = y - h_new / 2 - y1_new;
            break;
        case BL_DATUM:
        case BC_DATUM:
        case BR_DATUM:
            clearY = y - clearHeight;
            cursorY = y - h_new - y1_new;
            break;
    }

    // Perform the clear operation
    tft.fillRect(clearX, clearY, clearWidth, clearHeight, bgColor);

    // Draw the new text
    if (!text.isEmpty()) {
        tft.setCursor(cursorX, cursorY);
        tft.print(text);
    }

    // Update the last text variable
    lastText = text;
}

void DisplayManager::drawFloatOptimized(float value, int decimalPlaces, const String& unit, 
                                       int x, int y, const GFXfont* fontPtr, 
                                       uint16_t textColor, uint16_t bgColor, uint8_t datum, 
                                       float& lastValue, const char* format, 
                                       int clearWidth, int clearHeight) {
    // Determine if value changed significantly or became/stopped being NAN
    bool valueChanged = false;
    bool currentIsNan = isnan(value);
    bool lastIsNan = isnan(lastValue);
    float epsilon = pow(10, -decimalPlaces - 1);

    if (currentIsNan != lastIsNan) {
        valueChanged = true;
    } else if (!currentIsNan && (abs(value - lastValue) > epsilon)) {
        valueChanged = true;
    }

    if (!valueChanged && !needsRedrawCurrentScreen) {
        return; // No significant change
    }

    // Format the current value string
    char currentValStr[20];
    if (currentIsNan) {
        strcpy(currentValStr, "--");
    } else {
        if (format) {
            snprintf(currentValStr, sizeof(currentValStr), format, value);
        } else {
            dtostrf(value, 0, decimalPlaces, currentValStr);
        }
    }
    String currentText = String(currentValStr) + unit;

    // Format the last value string
    String lastText = "--" + unit;
    if (!lastIsNan) {
         char lastValStr[20];
         if (format) snprintf(lastValStr, sizeof(lastValStr), format, lastValue);
         else dtostrf(lastValue, 0, decimalPlaces, lastValStr);
         lastText = String(lastValStr) + unit;
    }

    // Call the optimized string drawing function
    drawStringOptimized(currentText, x, y, fontPtr, textColor, bgColor, datum, lastText, clearWidth, clearHeight);

    // Update the last float value
    lastValue = value;
}

void DisplayManager::drawIntOptimized(int value, const String& unit, int x, int y, 
                                     const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, 
                                     uint8_t datum, int& lastValue, const char* defaultText, 
                                     int clearWidth, int clearHeight, int validThreshold) {
    // Check if value changed or full redraw needed
    if (value == lastValue && !needsRedrawCurrentScreen) {
        return;
    }

    // Format current and last strings
    String currentText;
    if (value >= validThreshold) {
        currentText = String(value) + unit;
    } else {
        currentText = String(defaultText) + unit;
    }

    String lastText;
    if (lastValue >= validThreshold) {
        lastText = String(lastValue) + unit;
    } else {
        lastText = String(defaultText) + unit;
    }

    // Call the optimized string drawing function
    drawStringOptimized(currentText, x, y, fontPtr, textColor, bgColor, datum, lastText, clearWidth, clearHeight);

    // Update the last integer value
    lastValue = value;
}

// --- Control Functions ---
void DisplayManager::toggleScreen() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        screenOn = !screenOn;
        tft.enableDisplay(screenOn);
        if (TFT_BL >= 0) {
            digitalWrite(TFT_BL, screenOn ? TFT_BACKLIGHT_ON : !TFT_BACKLIGHT_ON);
        }
        if (screenOn) {
            needsRedrawCurrentScreen = true;
            startAnimation(ANIM_FADE_IN);
        }
        xSemaphoreGive(dataMutex);
        Serial.print("Screen toggled: "); Serial.println(screenOn ? "ON" : "OFF");
    }
}

void DisplayManager::switchDisplayMode() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Determine animation direction based on current and next mode
        ScreenMode nextMode = (ScreenMode)(((int)currentScreenMode + 1) % SCREEN_MODE_COUNT);
        
        // Start appropriate animation
        startAnimation(ANIM_SLIDE_LEFT);
        
        // Update mode
        currentScreenMode = nextMode;
        needsRedrawCurrentScreen = true;
        
        Serial.print("Switched display mode to: "); Serial.println((int)currentScreenMode);
        xSemaphoreGive(dataMutex);
    }
}

void DisplayManager::toggleTheme() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentTheme = 1 - currentTheme;
        if (needsRedrawCurrentScreen  == pdTRUE) {
            currentTheme = 1 - currentTheme;
            needsRedrawCurrentScreen = true;
            startAnimation(ANIM_FADE_IN);
            Serial.print("Switched theme to: "); Serial.println(currentTheme == 0 ? "Dark" : "Light");
            xSemaphoreGive(dataMutex);
        }
    }
}

bool DisplayManager::isScreenOn() const {
    bool isOn = false;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        isOn = screenOn;
        xSemaphoreGive(dataMutex);
    }
    return isOn;
}

// Helper function to calculate cursor position based on datum
void calculateCursorPos(int x, int y, uint8_t datum,
                        int16_t text_x_offset, int16_t text_y_offset,
                        uint16_t text_w, uint16_t text_h,
                        int& cursorX, int& cursorY)
{
    cursorX = x; // Default: Top-Left alignment

    // Horizontal alignment
    switch (datum) {
        case TC_DATUM:
        case MC_DATUM:
        case BC_DATUM:
            cursorX = x - text_w / 2 - text_x_offset; // Center align
            break;
        case TR_DATUM:
        case MR_DATUM:
        case BR_DATUM:
            cursorX = x - text_w - text_x_offset; // Right align
            break;
    }

    // Vertical alignment
    switch (datum) {
        case ML_DATUM:
        case MC_DATUM:
        case MR_DATUM:
            cursorY = y - text_h / 2 - text_y_offset;
            break;
        case BL_DATUM:
        case BC_DATUM:
        case BR_DATUM:
            cursorY = y - text_h - text_y_offset;
            break;
        default:
            cursorY = y - text_y_offset; // Top alignment
    }

    // Prevent cursor going off-screen
    if (cursorX < 0) cursorX = 0;
    if (cursorY < 0) cursorY = 0;
}