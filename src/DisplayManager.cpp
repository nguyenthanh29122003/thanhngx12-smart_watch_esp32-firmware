// src/DisplayManager.cpp
#include "DisplayManager.h"
#include "Config.h"   // If color/font definitions are here
#include <Arduino.h>
#include <WiFi.h>     // Only if drawing WiFi icon
#include <cmath>      // For isnan, abs, sqrt, pow

// --- TFT Pin Definitions (from old User_Setup.h) ---
#define TFT_CS     7
#define TFT_DC     39
#define TFT_RST    40 // Use -1 if RST pin is not used or tied to MCU RST
#define TFT_BL     45 // Backlight control pin

// --- Backlight Control ---
#define TFT_BACKLIGHT_ON HIGH // Or LOW depending on your hardware

// --- TFT_eSPI Datum Definitions (for compatibility in logic) ---
// Copied from TFT_eSPI library for reference in draw...Optimized functions
#define TL_DATUM 0 // Top left (default)
#define TC_DATUM 1 // Top centre
#define TR_DATUM 2 // Top right
#define ML_DATUM 3 // Middle left
#define MC_DATUM 4 // Middle centre
#define MR_DATUM 5 // Middle right
#define BL_DATUM 6 // Bottom left
#define BC_DATUM 7 // Bottom centre
#define BR_DATUM 8 // Bottom right
#define L_BASELINE  9 // Left character baseline (Line the 'A' character would sit on)
#define C_BASELINE 10 // Centre character baseline
#define R_BASELINE 11 // Right character baseline
// Note: Adafruit GFX primarily uses top-left for setCursor.
// Baseline datums are harder to replicate directly.

// --- UI Constants for Landscape Mode ---
#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH  240  // Width in landscape mode (was height in portrait)
#endif
#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT 135  // Height in landscape mode (was width in portrait)
#endif
#ifndef CENTER_X
#define CENTER_X (SCREEN_WIDTH / 2)
#endif
#ifndef CENTER_Y
#define CENTER_Y (SCREEN_HEIGHT / 2)
#endif
#ifndef WATCHFACE_RADIUS
#define WATCHFACE_RADIUS (SCREEN_HEIGHT / 2 - 10)  // Adjusted for landscape
#endif
#ifndef NUM_POINTS
#define NUM_POINTS 360
#endif

// --- Layout Constants for Landscape Mode ---
#define CLOCK_CENTER_X     60   // X center of clock in landscape mode
#define CLOCK_CENTER_Y     67   // Y center of clock in landscape mode
#define DATA_PANEL_X       130  // Starting X for data panel in landscape mode
#define DATA_PANEL_WIDTH   100  // Width of data panel in landscape mode

// --- Static Data Arrays (Unchanged) ---
const String daysOfWeek[7] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
const String daysOfWeekShort[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const String hourLabels[12] = {"00", "05", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55"}; // Minute/second labels

// --- Font Selection (Choose appropriate Adafruit GFX fonts) ---
// You MUST #include these in DisplayManager.h
const GFXfont* FONT_SMALL = &FreeSans9pt7b;       // Example replacement for Font 2
const GFXfont* FONT_LARGE = &FreeSansBold12pt7b;  // Example replacement for Font 4
const GFXfont* FONT_MONO = &FreeMonoBold9pt7b;   // Example monospace font if needed
// Use 'nullptr' for the default Adafruit classic font (similar to Font 1)


// --- Constructor ---
DisplayManager::DisplayManager()
    : tft(TFT_CS, TFT_DC, TFT_RST), // Initialize Adafruit TFT object here
      taskHandle(NULL), screenOn(true), currentTheme(0),
      displayOrientation(ORIENTATION_LANDSCAPE), // Default to landscape mode
      currentScreenMode(SCREEN_MODE_WATCHFACE),
      timeInitializedLocal(false), wifiConnectedLocal(false),
      stepCountLocal(0), distanceLocal(0.0f), heartRateLocal(0), spo2Local(-999),
      temperatureLocal(NAN), pressureLocal(NAN),
      axLocal(0.0f), ayLocal(0.0f), azLocal(0.0f), gxLocal(0.0f), gyLocal(0.0f), gzLocal(0.0f),
      lastTimeString(""), lastDateStringWF(""), lastDayStringWF(""), lastSecondAngleWF(-1),
      lastDisplayedSteps(-1), lastDisplayedHR(-1), lastDisplayedSpO2(-1000),
      lastDateStringSens(""),
      lastTempEnv(NAN), lastPresEnv(NAN), lastTimeEnv(""),
      lastAxIMU(NAN), lastAyIMU(NAN), lastAzIMU(NAN), lastGxIMU(NAN), lastGyIMU(NAN), lastGzIMU(NAN),
      lastWifiState(false),
      needsRedrawCurrentScreen(true)
{
    memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) { Serial.println("CRITICAL: Failed to create Display data mutex!"); }
}

// Helper to calculate cursor position based on datum and text bounds
void calculateCursorPos(int x, int y, uint8_t datum,
                        int16_t text_x_offset, int16_t text_y_offset, // From getTextBounds x1, y1
                        uint16_t text_w, uint16_t text_h,            // From getTextBounds w, h
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

    // Vertical alignment (Approximate for Adafruit GFX)
    // Adafruit's y position for setCursor refers to the top-left corner.
    // text_y_offset (y1) is the offset from the cursor Y to the top pixel of the glyph.
    // text_h is the total height.
    cursorY = y; // Default: Top alignment

    switch (datum) {
        case ML_DATUM:
        case MC_DATUM:
        case MR_DATUM:
            // Try to center vertically. Baseline/descender makes this tricky.
            // A common approximation is aligning the middle of the bounding box.
            cursorY = y - text_h / 2 - text_y_offset;
            break;
        case BL_DATUM:
        case BC_DATUM:
        case BR_DATUM:
            // Align bottom.
            cursorY = y - text_h - text_y_offset;
            break;
    }
     // Prevent cursor going off-screen top/left
     if (cursorX < 0) cursorX = 0;
     if (cursorY < 0) cursorY = 0;
}

void DisplayManager::drawStringOptimized(const String& text, int x, int y, const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, uint8_t datum,
                                         String& lastText, int clearWidth, int clearHeight) {
    // Check if text actually changed or if a full redraw is forced
    if (text == lastText && !needsRedrawCurrentScreen) {
        return; // Nothing to draw
    }

    // --- Text Drawing Logic ---
    tft.setFont(fontPtr);
    tft.setTextColor(textColor); // Adafruit uses foreground color only for print

    // --- Erasing Logic ---
    // Get bounds of the *previous* text to determine clear area
    // Note: This requires setting the font temporarily if it changed, which is complex.
    // A simpler approach: Use provided clearWidth/Height or estimate based on *new* text.
    int16_t x1_new = 0, y1_new = 0;
    uint16_t w_new = 0, h_new = 0;
    if (!text.isEmpty()) {
       tft.getTextBounds(text, 0, 0, &x1_new, &y1_new, &w_new, &h_new); // Use 0,0 temporarily
    } else {
        // If new text is empty, use last known dimensions or defaults for clearing
        if (clearWidth < 0) clearWidth = 30; // Default clear size
        if (clearHeight < 0) clearHeight = (fontPtr ? fontPtr->yAdvance : 8) + 4;
    }

    // If clearWidth/Height not provided, use new text bounds + padding
    if (clearWidth < 0) clearWidth = w_new + 4; // Add padding
    if (clearHeight < 0) clearHeight = h_new + 4; // Add padding

    // Calculate clear area top-left corner based on datum
    // (Using the same logic as cursor calculation but for the clear rectangle)
    int clearX = 0, clearY = 0;
    // We need the bounds of the *last* text for perfect clearing.
    // Approximate using the *new* text bounds or fixed clearWidth/Height.
    calculateCursorPos(x, y, datum, 0, 0, clearWidth, clearHeight, clearX, clearY);

    // Perform the clear operation
    tft.fillRect(clearX, clearY, clearWidth, clearHeight, bgColor);

    // --- Drawing New Text ---
    if (!text.isEmpty()) {
        int cursorX = 0, cursorY = 0;
        // Calculate the actual cursor position for the new text based on datum
        calculateCursorPos(x, y, datum, x1_new, y1_new, w_new, h_new, cursorX, cursorY);

        // Set cursor and print
        tft.setCursor(cursorX, cursorY);
        tft.print(text);
    }

    // Update the last text variable
    lastText = text;
}

void DisplayManager::drawFloatOptimized(float value, int decimalPlaces, const String& unit, int x, int y, const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, uint8_t datum,
                                        float& lastValue, const char* format, int clearWidth, int clearHeight) {
    // Determine if value changed significantly or became/stopped being NAN
    bool valueChanged = false;
    bool currentIsNan = isnan(value);
    bool lastIsNan = isnan(lastValue);
    float epsilon = pow(10, -decimalPlaces - 1); // Small value for float comparison

    if (currentIsNan != lastIsNan) {
        valueChanged = true;
    } else if (!currentIsNan && (abs(value - lastValue) > epsilon)) {
        valueChanged = true;
    }

    if (!valueChanged && !needsRedrawCurrentScreen) {
        return; // No significant change
    }

    // Format the current value string
    char currentValStr[20]; // Increased buffer size
    if (currentIsNan) {
        strcpy(currentValStr, "--"); // Display "--" for NAN
    } else {
        if (format) {
            snprintf(currentValStr, sizeof(currentValStr), format, value);
        } else {
            dtostrf(value, 0, decimalPlaces, currentValStr);
        }
    }
    String currentText = String(currentValStr) + unit;

    // Format the last value string (for calculating clear area if needed, although drawStringOptimized approximates)
    String lastText = "--" + unit; // Default last text assumes NAN or initial state
    if (!lastIsNan) {
         char lastValStr[20];
         if (format) snprintf(lastValStr, sizeof(lastValStr), format, lastValue);
         else dtostrf(lastValue, 0, decimalPlaces, lastValStr);
         lastText = String(lastValStr) + unit;
    }

    // Call the optimized string drawing function
    // Pass the *lastText* string primarily for context, the function will calculate clear area.
    drawStringOptimized(currentText, x, y, fontPtr, textColor, bgColor, datum, lastText, clearWidth, clearHeight);

    // Update the last float value only after drawing
    lastValue = value;
}

void DisplayManager::drawIntOptimized(int value, const String& unit, int x, int y, const GFXfont* fontPtr, uint16_t textColor, uint16_t bgColor, uint8_t datum,
                                      int& lastValue, const char* defaultText, int clearWidth, int clearHeight, int validThreshold) {

    // Check if value changed or full redraw needed
    if (value == lastValue && !needsRedrawCurrentScreen) {
        return;
    }

    // Format current and last strings
    String currentText;
    if (value >= validThreshold) {
        currentText = String(value) + unit;
    } else {
        currentText = String(defaultText) + unit; // Use default text if below threshold
    }

    String lastText;
    if (lastValue >= validThreshold) {
        lastText = String(lastValue) + unit;
    } else {
        lastText = String(defaultText) + unit;
    }

    // Call the optimized string drawing function
    // Pass the *lastText* string for context.
    drawStringOptimized(currentText, x, y, fontPtr, textColor, bgColor, datum, lastText, clearWidth, clearHeight);

    // Update the last integer value only after drawing
    lastValue = value;
}

// --- begin() - Modified for orientation support ---
bool DisplayManager::begin(uint8_t orientation) {
    Serial.println("Initializing Display Manager (Adafruit ST7789)...");

    // Set orientation
    displayOrientation = orientation;

    // --- Initialize Backlight ---
    if (TFT_BL >= 0) {
        Serial.printf("Initializing Backlight pin %d\n", TFT_BL);
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on
    } else {
        Serial.println("Backlight pin not defined.");
    }

    // --- Initialize TFT ---
    // For landscape mode, we swap width and height in initialization
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        tft.init(SCREEN_HEIGHT, SCREEN_WIDTH); // Swap dimensions for landscape
        Serial.println("TFT initialized in landscape mode");
    } else {
        tft.init(SCREEN_WIDTH, SCREEN_HEIGHT); // Original portrait dimensions
        Serial.println("TFT initialized in portrait mode");
    }

    // Set rotation based on orientation
    // ST7789 rotation: 0=portrait, 1=landscape(CW), 2=portrait(180°), 3=landscape(CCW)
    tft.setRotation(displayOrientation == ORIENTATION_LANDSCAPE ? 1 : 0);
    tft.fillScreen(DARK_BACKGROUND); // Initial clear with default theme bg

    // Optional: Invert display if colors are wrong
    // tft.invertDisplay(true);

    Serial.println("Calculating Watch Face UI points...");
    // --- Watch Face Point Calculation (Updated for landscape) ---
    int clockCenterX = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_X : CENTER_X;
    int clockCenterY = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_Y : CENTER_Y;
    int clockRadius = displayOrientation == ORIENTATION_LANDSCAPE ? 
                     min(CLOCK_CENTER_Y, SCREEN_HEIGHT - CLOCK_CENTER_Y) - 10 : 
                     WATCHFACE_RADIUS;
    
    for (int i = 0; i < NUM_POINTS; i++) {
        float angleRad = radians(i - 90);
        x[i]  = (clockRadius * cos(angleRad)) + clockCenterX;
        y[i]  = (clockRadius * sin(angleRad)) + clockCenterY;
        float outerRadius = clockRadius + 8;
        float innerRadius = clockRadius + 2;
        px[i] = (outerRadius * cos(angleRad)) + clockCenterX;
        py[i] = (outerRadius * sin(angleRad)) + clockCenterY;
        lx[i] = (innerRadius * cos(angleRad)) + clockCenterX;
        ly[i] = (innerRadius * sin(angleRad)) + clockCenterY;
        if (i % 30 == 0) startHour[i / 30] = i;
        if (i % 6 == 0) startMinute[i / 6] = i;
    }
    Serial.println("Display Manager Initialized.");
    return true;
}

// --- Set Orientation Method (New) ---
void DisplayManager::setOrientation(uint8_t orientation) {
    if (orientation == displayOrientation) return; // No change needed
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        displayOrientation = orientation;
        
        // Set the display rotation
        tft.setRotation(displayOrientation == ORIENTATION_LANDSCAPE ? 1 : 0);
        
        // Force a complete redraw
        needsRedrawCurrentScreen = true;
        
        // Recalculate watch face points
        int clockCenterX = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_X : CENTER_X;
        int clockCenterY = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_Y : CENTER_Y;
        int clockRadius = displayOrientation == ORIENTATION_LANDSCAPE ? 
                         min(CLOCK_CENTER_Y, SCREEN_HEIGHT - CLOCK_CENTER_Y) - 10 : 
                         WATCHFACE_RADIUS;
        
        for (int i = 0; i < NUM_POINTS; i++) {
            float angleRad = radians(i - 90);
            x[i]  = (clockRadius * cos(angleRad)) + clockCenterX;
            y[i]  = (clockRadius * sin(angleRad)) + clockCenterY;
            float outerRadius = clockRadius + 8;
            float innerRadius = clockRadius + 2;
            px[i] = (outerRadius * cos(angleRad)) + clockCenterX;
            py[i] = (outerRadius * sin(angleRad)) + clockCenterY;
            lx[i] = (innerRadius * cos(angleRad)) + clockCenterX;
            ly[i] = (innerRadius * sin(angleRad)) + clockCenterY;
        }
        
        xSemaphoreGive(dataMutex);
        Serial.print("Display orientation changed to: ");
        Serial.println(displayOrientation == ORIENTATION_LANDSCAPE ? "Landscape" : "Portrait");
    }
}

// --- Get Orientation Method (New) ---
uint8_t DisplayManager::getOrientation() const {
    return displayOrientation;
}

// --- Helper Methods for Orientation (New) ---
void DisplayManager::getAdjustedCoordinates(int& x, int& y) {
    // This function adjusts coordinates based on current orientation
    // For now, it's a placeholder - implement as needed for specific UI elements
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        // Apply any landscape-specific adjustments if needed
    }
    // No adjustment needed for portrait mode (original coordinates)
}

int DisplayManager::getScreenWidth() const {
    return displayOrientation == ORIENTATION_LANDSCAPE ? SCREEN_WIDTH : SCREEN_HEIGHT;
}

int DisplayManager::getScreenHeight() const {
    return displayOrientation == ORIENTATION_LANDSCAPE ? SCREEN_HEIGHT : SCREEN_WIDTH;
}

// --- Task Management (Unchanged) ---
void DisplayManager::startTask(UBaseType_t priority) {
    xTaskCreate(taskFunction, "DisplayTask", 4096, this, priority, &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating Display Task!");
    else Serial.println("Display Task started.");
}

void DisplayManager::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle; taskHandle = NULL;
        tft.enableDisplay(false); // Use Adafruit function to turn display off
        if (TFT_BL >= 0) digitalWrite(TFT_BL, !TFT_BACKLIGHT_ON); // Turn backlight off
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
        vTaskDelay(pdMS_TO_TICKS(100)); // Update rate (10Hz)
    }
}

// --- Data Update (Unchanged) ---
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
        else memset(&timeinfoLocal, 0, sizeof(timeinfoLocal)); // Clear local time if not valid
        stepCountLocal = stepCount; distanceLocal = distance;
        heartRateLocal = heartRate; spo2Local = spo2;
        temperatureLocal = temperature; pressureLocal = pressure;
        axLocal = ax; ayLocal = ay; azLocal = az;
        gxLocal = gx; gyLocal = gy; gzLocal = gz;
        xSemaphoreGive(dataMutex);
    } else { Serial.println("Timeout taking display data mutex in updateData!"); }
}

// --- Main Update Loop (Unchanged) ---
void DisplayManager::updateDisplay() {
    bool localScreenOn = false;
    ScreenMode localScreenMode;
    bool needsRedraw = false;

    // Safely get current state
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localScreenOn = screenOn;
        localScreenMode = currentScreenMode;
        needsRedraw = needsRedrawCurrentScreen;
        if (needsRedraw) needsRedrawCurrentScreen = false; // Reset redraw flag
        xSemaphoreGive(dataMutex);
    } else {
        // Could not get mutex, skip this update cycle
        return;
    }

    // If screen is off, do nothing
    if (!localScreenOn) return;

    // If a full redraw is needed (mode changed, screen toggled on), reset optimization variables
    if (needsRedraw) {
        clearScreen(); // Clear the entire screen
        // Reset all "last..." variables to force redrawing elements
        lastTimeString = ""; lastDateStringWF = ""; lastDayStringWF = ""; lastSecondAngleWF = -1;
        lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
        lastDateStringSens = ""; lastTempEnv = NAN; lastPresEnv = NAN; lastTimeEnv = "";
        lastAxIMU = lastAyIMU = lastAzIMU = lastGxIMU = lastGyIMU = lastGzIMU = NAN;
        lastWifiState = !wifiConnectedLocal; // Force redraw if state differs
    }

    // Call the drawing function for the current mode
    switch (localScreenMode) {
        case SCREEN_MODE_WATCHFACE:     drawWatchFaceScreen(needsRedraw); break;
        case SCREEN_MODE_SENSORS_PRIMARY: drawSensorPrimaryScreen(needsRedraw); break;
        case SCREEN_MODE_ENVIRONMENT:   drawEnvironmentScreen(needsRedraw); break;
        case SCREEN_MODE_IMU_DATA:      drawImuDataScreen(needsRedraw); break;
        default: break; // Unknown mode
    }
}

// --- Screen Drawing Functions (Modified for Landscape) ---

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
    } else { return; } // Skip if data unavailable

    // Draw static elements if needed
    if (redrawStatic) {
        drawClockFace();
        // Reset optimization vars for this screen
        lastTimeString = ""; lastDateStringWF = ""; lastDayStringWF = "";
        lastSecondAngleWF = -1; lastWifiState = !localWifiConnected;
    }

    // Handle invalid time display
    if (!isTimeValid) {
        if (lastTimeString != "WaitingWF") { // Draw only if message changed
            uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
            uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
            
            // Adjust position for landscape mode
            int centerX = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_X : CENTER_X;
            int centerY = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_Y : CENTER_Y;
            
            // Use optimized drawing function
            drawStringOptimized("Waiting for", centerX, centerY - 10, FONT_SMALL, textColor, bgColor, MC_DATUM, lastTimeString, 100); // Placeholder for lastText
            // Need to draw the second line separately or combine them
            String dummyLast = ""; // Need a dummy last text for the second line draw
            drawStringOptimized("Time Sync...", centerX, centerY + 10, FONT_SMALL, textColor, bgColor, MC_DATUM, dummyLast, 100);
            lastTimeString = "WaitingWF"; // Set the combined state marker

            lastSecondAngleWF = -1; lastDateStringWF = ""; lastDayStringWF = ""; // Reset other related vars
        }
        return; // Don't draw time elements if time is invalid
    }
     // If we were waiting, force redraw of dynamic elements
    if (lastTimeString == "WaitingWF") {
         redrawStatic = true; // Force update of time/date elements
         lastTimeString = ""; lastDateStringWF = ""; lastDayStringWF = ""; lastSecondAngleWF = -1; // Clear wait state
    }

    // Get current time/date strings
    int angle = localTimeCopy.tm_sec * 6; // Angle for second hand/marker
    char timeStr[12]; sprintf(timeStr, "%02d:%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, localTimeCopy.tm_sec);
    String currentTime = String(timeStr);
    String currentDay = daysOfWeek[localTimeCopy.tm_wday];
    char monthStr[4]; strftime(monthStr, sizeof(monthStr), "%b", &localTimeCopy); // Get abbreviated month
    String currentFullDateStr = String(daysOfWeekShort[localTimeCopy.tm_wday]) + " " + String(localTimeCopy.tm_mday) + " " + String(monthStr);

    // Check what changed
    bool timeChanged = (currentTime != lastTimeString);
    bool dayChanged = (currentDay != lastDayStringWF); // May not be needed if date string includes day
    bool dateChanged = (currentFullDateStr != lastDateStringWF);
    bool angleChanged = (angle != lastSecondAngleWF);
    bool wifiChanged = (localWifiConnected != lastWifiState);

    // Update dynamic elements
    if (angleChanged || timeChanged || redrawStatic) { // Update time display and second hand/marker
        updateWatchFaceTimeDisplay(angle, currentDay, currentTime);
    }
    if (dateChanged || redrawStatic) { // Update date display
        updateWatchFaceDateDisplay(currentFullDateStr);
    }
    if (wifiChanged || redrawStatic) { // Update WiFi icon
        drawWifiIcon(); // This function internally updates lastWifiState
    }

    // Update "last" state variables for next cycle
    lastTimeString = currentTime;
    lastDayStringWF = currentDay; // If used
    lastDateStringWF = currentFullDateStr;
    lastSecondAngleWF = angle;
    // lastWifiState is updated within drawWifiIcon()
    
    // In landscape mode, draw additional data panel on the right side
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        drawSensorDataPanel(redrawStatic);
    }
}

// --- New method for landscape mode data panel ---
void DisplayManager::drawSensorDataPanel(bool redrawStatic) {
    int localSteps = 0;
    int localHR = 0;
    int localSpO2 = -999;
    float localTemp = NAN;

    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localSteps = stepCountLocal;
        localHR = heartRateLocal;
        localSpO2 = spo2Local;
        localTemp = temperatureLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    // Define colors based on theme
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t stepsColor = COLOR_STEPS;
    uint16_t hrColor = COLOR_HR;
    uint16_t spo2Color = COLOR_SPO2;
    uint16_t tempColor = COLOR_TEMP;

    // Define layout for data panel
    int panelX = DATA_PANEL_X;
    int panelY = 20;
    int lineHeight = 25;
    int labelX = panelX;
    int valueX = panelX + DATA_PANEL_WIDTH - 10;

    // Draw panel background if needed
    if (redrawStatic) {
        tft.fillRect(panelX - 10, 0, SCREEN_WIDTH - panelX + 10, SCREEN_HEIGHT, bgColor);
        // Optional: Draw a separator line
        tft.drawLine(panelX - 10, 0, panelX - 10, SCREEN_HEIGHT, textColor);
    }

    // Draw data items
    // 1. Steps
    String dummy = "";
    drawStringOptimized("Steps:", labelX, panelY, FONT_SMALL, stepsColor, bgColor, TL_DATUM, dummy);
    drawIntOptimized(localSteps, "", valueX, panelY, FONT_SMALL, textColor, bgColor, TR_DATUM, lastDisplayedSteps, "--", 80);
    
    // 2. Heart Rate
    panelY += lineHeight;
    drawStringOptimized("HR:", labelX, panelY, FONT_SMALL, hrColor, bgColor, TL_DATUM, dummy);
    drawIntOptimized(localHR, " BPM", valueX, panelY, FONT_SMALL, textColor, bgColor, TR_DATUM, lastDisplayedHR, "--", 80);
    
    // 3. SpO2
    panelY += lineHeight;
    drawStringOptimized("SpO2:", labelX, panelY, FONT_SMALL, spo2Color, bgColor, TL_DATUM, dummy);
    drawIntOptimized(localSpO2, " %", valueX, panelY, FONT_SMALL, textColor, bgColor, TR_DATUM, lastDisplayedSpO2, "--", 80, -9999);
    
    // 4. Temperature
    panelY += lineHeight;
    drawStringOptimized("Temp:", labelX, panelY, FONT_SMALL, tempColor, bgColor, TL_DATUM, dummy);
    drawFloatOptimized(localTemp, 1, " C", valueX, panelY, FONT_SMALL, textColor, bgColor, TR_DATUM, lastTempEnv, "%.1f", 80);
}

void DisplayManager::drawSensorPrimaryScreen(bool redrawStatic) {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    int localSteps = 0;
    int localHR = 0;
    int localSpO2 = -999;

    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localSteps = stepCountLocal;
        localHR = heartRateLocal;
        localSpO2 = spo2Local;
        xSemaphoreGive(dataMutex);
    } else { return; }

    // Define colors based on theme
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t dateColor = (currentTheme == 0) ? COLOR_DATE : COLOR_DARKGREY; // Specific color for date
    uint16_t spo2Color = COLOR_SPO2;
    uint16_t hrColor   = COLOR_HR;
    uint16_t stepsColor= COLOR_STEPS;

    // Reset optimization vars if full redraw
    if (redrawStatic) {
        lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
        lastDateStringSens = ""; // Reset date/time string for this screen
    }

    // Define layout constants - adjusted for landscape mode
    int startX, startY, lineSpacing, labelX, valueX, dataWidth;
    
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        startX = 20;
        startY = 30;
        lineSpacing = 30; // Horizontal spacing between items
        labelX = startX; // X position for labels (left aligned)
        valueX = labelX + 60; // X position for values
        dataWidth = 80; // Width for clearing values
    } else {
        // Original portrait layout
        startY = 30; // Initial Y position
        lineSpacing = 55; // Spacing between sensor readings
        labelX = 15; // X position for labels (left aligned)
        valueX = SCREEN_WIDTH - 15; // X position for values (right aligned)
        dataWidth = SCREEN_WIDTH - 30; // Max width for clearing values
    }
    
    int valueYOffset = 18; // Offset Y for value text relative to label
    int labelYOffset = 18; // Offset Y for label text

    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        // Landscape layout - horizontal arrangement
        int itemX = startX;
        
        // 1. Steps
        String dummy = "";
        drawStringOptimized("Steps", itemX, startY, FONT_SMALL, stepsColor, bgColor, TL_DATUM, dummy);
        drawIntOptimized(localSteps, "", itemX, startY + valueYOffset, FONT_SMALL, textColor, bgColor, TL_DATUM, lastDisplayedSteps, "--", dataWidth);
        
        // 2. Heart Rate
        itemX += lineSpacing * 2;
        drawStringOptimized("HR", itemX, startY, FONT_SMALL, hrColor, bgColor, TL_DATUM, dummy);
        drawIntOptimized(localHR, " BPM", itemX, startY + valueYOffset, FONT_SMALL, textColor, bgColor, TL_DATUM, lastDisplayedHR, "--", dataWidth);
        
        // 3. SpO2
        itemX += lineSpacing * 2;
        drawStringOptimized("SpO2", itemX, startY, FONT_SMALL, spo2Color, bgColor, TL_DATUM, dummy);
        drawIntOptimized(localSpO2, " %", itemX, startY + valueYOffset, FONT_SMALL, textColor, bgColor, TL_DATUM, lastDisplayedSpO2, "--", dataWidth, -9999);
        
        // 4. Time and Date (Bottom)
        int bottomY = SCREEN_HEIGHT - 20;
        String currentDateTimeStr;
        if (isTimeValid) {
            char dateTimeBuf[20];
            sprintf(dateTimeBuf, "%02d:%02d | %s %02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, 
                    daysOfWeekShort[localTimeCopy.tm_wday], localTimeCopy.tm_mday);
            currentDateTimeStr = String(dateTimeBuf);
        } else {
            currentDateTimeStr = "TIME N/A";
        }
        drawStringOptimized(currentDateTimeStr, CENTER_X, bottomY, FONT_SMALL, dateColor, bgColor, MC_DATUM, lastDateStringSens, SCREEN_WIDTH);
    } else {
        // Original portrait layout - vertical arrangement
        // 1. Steps
        drawIntOptimized(localSteps, "", valueX, startY + valueYOffset, FONT_LARGE, textColor, bgColor, TR_DATUM, lastDisplayedSteps, "--", dataWidth);
        if (redrawStatic || localSteps != lastDisplayedSteps) {
            String dummy = "";
            drawStringOptimized("Steps", labelX, startY + labelYOffset, FONT_LARGE, stepsColor, bgColor, TL_DATUM, dummy);
        }

        // 2. Heart Rate
        startY += lineSpacing;
        drawIntOptimized(localHR, " BPM", valueX, startY + valueYOffset, FONT_LARGE, textColor, bgColor, TR_DATUM, lastDisplayedHR, "--", dataWidth);
        if (redrawStatic || localHR != lastDisplayedHR) {
            String dummy = "";
            drawStringOptimized("HR", labelX, startY + labelYOffset, FONT_LARGE, hrColor, bgColor, TL_DATUM, dummy);
        }

        // 3. SpO2
        startY += lineSpacing;
        drawIntOptimized(localSpO2, " %", valueX, startY + valueYOffset, FONT_LARGE, textColor, bgColor, TR_DATUM, lastDisplayedSpO2, "--", dataWidth, -9999);
        if (redrawStatic || localSpO2 != lastDisplayedSpO2) {
            String dummy = "";
            drawStringOptimized("SpO2", labelX, startY + labelYOffset, FONT_LARGE, spo2Color, bgColor, TL_DATUM, dummy);
        }

        // 4. Time and Date (Bottom, Centered)
        int bottomY = SCREEN_HEIGHT - 20; // Y position for bottom text
        String currentDateTimeStr;
        if (isTimeValid) {
            char dateTimeBuf[20];
            sprintf(dateTimeBuf, "%02d:%02d | %s %02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, daysOfWeekShort[localTimeCopy.tm_wday], localTimeCopy.tm_mday);
            currentDateTimeStr = String(dateTimeBuf);
        } else {
            currentDateTimeStr = "TIME N/A";
        }
        drawStringOptimized(currentDateTimeStr, CENTER_X, bottomY, FONT_SMALL, dateColor, bgColor, MC_DATUM, lastDateStringSens, SCREEN_WIDTH);
    }
}

void DisplayManager::drawEnvironmentScreen(bool redrawStatic) {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    float localTemp = NAN;
    float localPres = NAN;

    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localTemp = temperatureLocal;
        localPres = pressureLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    // Define colors based on theme
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t tempColor = COLOR_TEMP;
    uint16_t presColor = COLOR_PRESSURE;
    uint16_t timeColor = textColor; // Use standard text color for time

    // Reset optimization vars if full redraw
    if (redrawStatic) {
        lastTempEnv = NAN; lastPresEnv = NAN;
        lastTimeEnv = ""; // Reset time string for this screen
    }

    // Define layout constants - adjusted for landscape mode
    int startY, lineSpacing, labelX, valueX, dataWidth;
    
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        startY = 40;
        lineSpacing = 50; // Horizontal spacing in landscape
        labelX = 30;
        valueX = SCREEN_WIDTH - 30;
        dataWidth = 100;
    } else {
        // Original portrait layout
        startY = 50;
        lineSpacing = 65;
        labelX = 15;
        valueX = SCREEN_WIDTH - 15;
        dataWidth = SCREEN_WIDTH - 30;
    }
    
    int valueYOffset = 18; // Adjust based on font
    int labelYOffset = 18; // Adjust based on font

    // 1. Temperature
    drawFloatOptimized(localTemp, 1, " C", valueX, startY + valueYOffset, FONT_LARGE, textColor, bgColor, TR_DATUM, lastTempEnv, "%.1f", dataWidth);
    if (redrawStatic || localTemp != lastTempEnv) { // Use != for float comparison (handles NAN change)
        String dummy = "";
        drawStringOptimized("Temp", labelX, startY + labelYOffset, FONT_LARGE, tempColor, bgColor, TL_DATUM, dummy);
    }

    // 2. Pressure
    startY += lineSpacing;
    float presHpa = isnan(localPres) ? NAN : localPres / 100.0f;
    float lastPresHpaEnv = isnan(lastPresEnv) ? NAN : lastPresEnv / 100.0f; // Compare in hPa
    drawFloatOptimized(presHpa, 1, " hPa", valueX, startY + valueYOffset, FONT_LARGE, textColor, bgColor, TR_DATUM, lastPresHpaEnv /* Compare hPa */, "%.1f", dataWidth);
    if (redrawStatic || localPres != lastPresEnv) { // Use original Pa for change detection
        String dummy = "";
        drawStringOptimized("Press", labelX, startY + labelYOffset, FONT_LARGE, presColor, bgColor, TL_DATUM, dummy);
    }
    lastPresEnv = localPres; // Update last value in Pa

    // 3. Time (HH:MM) at the bottom
    int bottomY = SCREEN_HEIGHT - 20;
    String currentTimeStr;
    if (isTimeValid) {
        char timeBuf[6]; sprintf(timeBuf, "%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min);
        currentTimeStr = String(timeBuf);
    } else {
        currentTimeStr = "--:--";
    }
    drawStringOptimized(currentTimeStr, CENTER_X, bottomY, FONT_SMALL, timeColor, bgColor, MC_DATUM, lastTimeEnv, 80); // Clear width 80
}

void DisplayManager::drawImuDataScreen(bool redrawStatic) {
    float localAx=NAN, localAy=NAN, localAz=NAN, localGx=NAN, localGy=NAN, localGz=NAN;

    // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localAx = axLocal; localAy = ayLocal; localAz = azLocal;
        localGx = gxLocal; localGy = gyLocal; localGz = gzLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    // Define colors
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t accelColor = COLOR_ACCEL;
    uint16_t gyroColor = COLOR_GYRO;

    // Reset optimization vars if full redraw
    if (redrawStatic) {
        lastAxIMU = lastAyIMU = lastAzIMU = lastGxIMU = lastGyIMU = lastGzIMU = NAN;
    }

    // Define layout constants - adjusted for landscape mode
    int startY, lineSpacing, labelX, valueX, valueWidth;
    const GFXfont* imuFont;
    
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        startY = 15;
        lineSpacing = 20; // Tighter spacing for landscape
        labelX = 10;
        valueX = 15;
        valueWidth = 100;
        imuFont = FONT_SMALL; // Use small font for landscape
    } else {
        // Original portrait layout
        startY = 15;
        lineSpacing = 22;
        labelX = 10;
        valueX = 15;
        valueWidth = SCREEN_WIDTH - valueX - 10;
        imuFont = FONT_SMALL;
    }
    
    int headerYOffset = 0;
    int valueYOffset = 0;

    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        // Landscape layout - two columns side by side
        int leftColX = 20;
        int rightColX = SCREEN_WIDTH / 2 + 10;
        
        // Left column - Accelerometer
        String dummy = "";
        drawStringOptimized("Accel (g)", leftColX, startY + headerYOffset, imuFont, accelColor, bgColor, TL_DATUM, dummy);
        startY += lineSpacing + 5;
        
        drawFloatOptimized(localAx, 2, "", leftColX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastAxIMU, "X: %+.2f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localAy, 2, "", leftColX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastAyIMU, "Y: %+.2f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localAz, 2, "", leftColX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastAzIMU, "Z: %+.2f", valueWidth);
        
        // Right column - Gyroscope
        startY = 15; // Reset Y position for right column
        drawStringOptimized("Gyro (dps)", rightColX, startY + headerYOffset, imuFont, gyroColor, bgColor, TL_DATUM, dummy);
        startY += lineSpacing + 5;
        
        drawFloatOptimized(localGx, 1, "", rightColX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastGxIMU, "X: %+.1f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localGy, 1, "", rightColX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastGyIMU, "Y: %+.1f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localGz, 1, "", rightColX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastGzIMU, "Z: %+.1f", valueWidth);
    } else {
        // Original portrait layout - vertical arrangement
        // --- Accelerometer Section ---
        if (redrawStatic) {
            String dummy = "";
            drawStringOptimized("Accel (g)", labelX, startY + headerYOffset, imuFont, accelColor, bgColor, TL_DATUM, dummy);
        }
        startY += lineSpacing + 5; // Extra space after header

        // Accel X, Y, Z (Using specific format strings in drawFloatOptimized)
        drawFloatOptimized(localAx, 2, "", valueX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastAxIMU, "X: %+.2f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localAy, 2, "", valueX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastAyIMU, "Y: %+.2f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localAz, 2, "", valueX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastAzIMU, "Z: %+.2f", valueWidth);

        // --- Gyroscope Section ---
        startY += lineSpacing + 10; // Extra space before next header
        if (redrawStatic) {
            String dummy = "";
            drawStringOptimized("Gyro (dps)", labelX, startY + headerYOffset, imuFont, gyroColor, bgColor, TL_DATUM, dummy);
        }
        startY += lineSpacing + 5; // Extra space after header

        // Gyro X, Y, Z
        drawFloatOptimized(localGx, 1, "", valueX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastGxIMU, "X: %+.1f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localGy, 1, "", valueX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastGyIMU, "Y: %+.1f", valueWidth);
        startY += lineSpacing;
        drawFloatOptimized(localGz, 1, "", valueX, startY + valueYOffset, imuFont, textColor, bgColor, TL_DATUM, lastGzIMU, "Z: %+.1f", valueWidth);
    }
}

// --- Helper Drawing Functions ---

void DisplayManager::clearScreen() {
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    tft.fillScreen(bgColor);
}

void DisplayManager::drawWifiIcon() {
    bool localWifiConnected;
     // Safely get data
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localWifiConnected = wifiConnectedLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    // Check if state changed or redraw forced
    if (localWifiConnected == lastWifiState && !needsRedrawCurrentScreen) {
        return; // Nothing to do
    }

    // Adjust position based on orientation
    int iconX, iconY;
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        iconX = SCREEN_WIDTH - 20;
        iconY = 5;
    } else {
        iconX = SCREEN_WIDTH - 20;
        iconY = 5;
    }
    
    int iconW = 15;
    int iconH = 12;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t iconColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;

    // Clear the icon area first
    tft.fillRect(iconX, iconY, iconW, iconH, bgColor);

    if (localWifiConnected) {
        // Draw a simple WiFi icon (e.g., 3 arcs)
        tft.drawCircle(iconX + iconW / 2, iconY + iconH, iconW / 2, iconColor);
        tft.drawCircle(iconX + iconW / 2, iconY + iconH, iconW / 3, iconColor);
        tft.drawCircle(iconX + iconW / 2, iconY + iconH, iconW / 6, iconColor);
        // You might need drawPixel for the center dot or adjust drawCircle parameters
        tft.fillCircle(iconX + iconW / 2, iconY + iconH, 1, iconColor); // Small dot at bottom center
    }

    lastWifiState = localWifiConnected; // Update the state
}

void DisplayManager::drawClockFace() {
    uint16_t faceColor = (currentTheme == 0) ? DARK_LINES : LIGHT_LINES;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t hourColor = (currentTheme == 0) ? DARK_HIGHLIGHT : LIGHT_HIGHLIGHT; // Color for hour markers

    // Determine clock center based on orientation
    int clockCenterX = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_X : CENTER_X;
    int clockCenterY = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_Y : CENTER_Y;

    // Draw minute/second markers (thin lines)
    for (int i = 0; i < NUM_POINTS; i += 6) { // Every 6 degrees (60 markers)
        tft.drawLine(lx[i], ly[i], px[i], py[i], faceColor);
    }

    // Draw hour markers (thicker lines or different shape)
    for (int i = 0; i < NUM_POINTS; i += 30) { // Every 30 degrees (12 markers)
        // Make hour lines slightly thicker by drawing twice
        tft.drawLine(lx[i], ly[i], px[i], py[i], hourColor);
    }

    // Draw center dot
    tft.fillCircle(clockCenterX, clockCenterY, 3, faceColor);
}

void DisplayManager::updateWatchFaceTimeDisplay(int angle, const String& currentDay, const String& currentTime) {
    uint16_t textColor = (currentTheme == 0) ? COLOR_TIME_DARK : COLOR_TIME_LIGHT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t secondColor = (currentTheme == 0) ? DARK_HIGHLIGHT : LIGHT_HIGHLIGHT;

    // Determine clock center based on orientation
    int clockCenterX = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_X : CENTER_X;
    int clockCenterY = displayOrientation == ORIENTATION_LANDSCAPE ? CLOCK_CENTER_Y : CENTER_Y;

    // --- 1. Update Digital Time ---
    int timeY = clockCenterY - 10; // Adjust Y position
    // Use the existing lastTimeString variable for optimization
    drawStringOptimized(currentTime, clockCenterX, timeY, FONT_LARGE, textColor, bgColor, MC_DATUM, lastTimeString, 100, 25);

    // --- 2. Update Day String (Optional, below time) ---
    int dayY = clockCenterY + 15; // Adjust Y position
    // Use the existing lastDayStringWF variable for optimization
    drawStringOptimized(currentDay, clockCenterX, dayY, FONT_SMALL, textColor, bgColor, MC_DATUM, lastDayStringWF, 100, 20);

    // --- 3. Update Second Marker/Hand ---
    // Erase the previous second hand/marker by drawing it in background color
    if (lastSecondAngleWF >= 0) {
        tft.drawLine(clockCenterX, clockCenterY, x[lastSecondAngleWF], y[lastSecondAngleWF], bgColor);
        tft.drawLine(clockCenterX+1, clockCenterY, x[lastSecondAngleWF]+1, y[lastSecondAngleWF], bgColor); // Thicker erase
    }

    // Draw the new second hand/marker
    if (angle >= 0) {
        tft.drawLine(clockCenterX, clockCenterY, x[angle], y[angle], secondColor);
        tft.drawLine(clockCenterX+1, clockCenterY, x[angle]+1, y[angle], secondColor); // Thicker draw
    }

    // Center dot needs redraw if erased by second hand erase
    uint16_t faceColor = (currentTheme == 0) ? DARK_LINES : LIGHT_LINES;
    tft.fillCircle(clockCenterX, clockCenterY, 3, faceColor);
}

void DisplayManager::updateWatchFaceDateDisplay(const String& fullDateStr) {
    uint16_t dateColor = (currentTheme == 0) ? COLOR_DATE : COLOR_DARKGREY;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    
    // Adjust position based on orientation
    int dateX, dateY;
    if (displayOrientation == ORIENTATION_LANDSCAPE) {
        dateX = CLOCK_CENTER_X;
        dateY = 15;
    } else {
        dateX = CENTER_X;
        dateY = 15;
    }

    // Use the existing lastDateStringWF variable for optimization
    drawStringOptimized(fullDateStr, dateX, dateY, FONT_SMALL, dateColor, bgColor, MC_DATUM, lastDateStringWF, 120);
}

// --- Optimized Drawing Functions (Unchanged) ---
// These functions remain the same as they already handle positioning based on datum

// --- Control Functions ---
void DisplayManager::toggleScreen() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        screenOn = !screenOn;
        tft.enableDisplay(screenOn); // Use Adafruit function
        if (TFT_BL >= 0) {
            digitalWrite(TFT_BL, screenOn ? TFT_BACKLIGHT_ON : !TFT_BACKLIGHT_ON); // Toggle backlight
        }
        if (screenOn) {
            needsRedrawCurrentScreen = true; // Force redraw when turning screen back on
        }
        xSemaphoreGive(dataMutex);
        Serial.print("Screen toggled: "); Serial.println(screenOn ? "ON" : "OFF");
    }
}

void DisplayManager::switchDisplayMode() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentScreenMode = (ScreenMode)(((int)currentScreenMode + 1) % SCREEN_MODE_COUNT);
        needsRedrawCurrentScreen = true; // Force redraw for the new mode
        Serial.print("Switched display mode to: "); Serial.println((int)currentScreenMode);
        xSemaphoreGive(dataMutex);
    }
}

void DisplayManager::toggleTheme() {
     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentTheme = 1 - currentTheme; // Toggle 0 and 1
        needsRedrawCurrentScreen = true; // Force redraw with new theme colors
        Serial.print("Switched theme to: "); Serial.println(currentTheme == 0 ? "Dark" : "Light");
        xSemaphoreGive(dataMutex);
    }
}

bool DisplayManager::isScreenOn() const {
    // Directly return the state variable (no mutex needed for simple read if acceptable)
    // Or use mutex for guaranteed consistency:
    bool isOn = false;
     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { // Shorter timeout for read
         isOn = screenOn;
         xSemaphoreGive(dataMutex);
     }
    return isOn;
}

// Helper function to get font height (approximates for default font)
int16_t GFXfont_height(const GFXfont* font) {
    if (font) {
        return font->yAdvance;
    } else {
        return 8; // Height of default Adafruit font size 1
    }
}