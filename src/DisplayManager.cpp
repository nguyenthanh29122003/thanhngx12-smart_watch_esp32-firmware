// src/DisplayManager.cpp
#include "DisplayManager.h"
#include "Config.h"   // Nếu có định nghĩa màu/font ở đây
#include <Arduino.h>
#include <WiFi.h>     // Chỉ cần nếu vẽ icon WiFi
#include <cmath>      // Cho isnan, abs, sqrt

// --- Hằng số UI (Đảm bảo đã định nghĩa ở .h hoặc Config.h) ---
// Lấy từ .h nếu đã định nghĩa ở đó
#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH  135
#endif
#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT 240
#endif
#ifndef CENTER_X
#define CENTER_X (SCREEN_WIDTH / 2)
#endif
#ifndef CENTER_Y
#define CENTER_Y (SCREEN_HEIGHT / 2)
#endif
#ifndef WATCHFACE_RADIUS
#define WATCHFACE_RADIUS (SCREEN_WIDTH / 2 - 10)
#endif
#ifndef NUM_POINTS
#define NUM_POINTS 360
#endif

// Màu đặc biệt (Định nghĩa lại bằng giá trị hex cho an toàn)
#define COLOR_TIME_DARK   0xAFBF // ~ #aafaff
#define COLOR_TIME_LIGHT  0x0594 // ~ #00b4b4
#define COLOR_DATE        TFT_LIGHTGREY // Giữ màu chuẩn
// Các màu khác (SPO2, HR, ...) đã được định nghĩa trong .h

// --- Mảng dữ liệu tĩnh ---
const String daysOfWeek[7] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
const String daysOfWeekShort[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const String hourLabels[12] = {"00", "05", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55"}; // Nhãn phút/giây

// --- Constructor ---
DisplayManager::DisplayManager()
    : tft(), taskHandle(NULL), screenOn(true), currentTheme(0),
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

// --- Hàm begin() ---
bool DisplayManager::begin() {
    Serial.println("Initializing Display Manager (ST7789)...");
    tft.init();
    tft.setRotation(0);
    tft.setSwapBytes(true);

    Serial.println("Calculating Watch Face UI points...");
    for (int i = 0; i < NUM_POINTS; i++) {
        float angleRad = radians(i - 90);
        x[i]  = (WATCHFACE_RADIUS * cos(angleRad)) + CENTER_X;
        y[i]  = (WATCHFACE_RADIUS * sin(angleRad)) + CENTER_Y;
        float outerRadius = WATCHFACE_RADIUS + 8;
        float innerRadius = WATCHFACE_RADIUS + 2;
        px[i] = (outerRadius * cos(angleRad)) + CENTER_X;
        py[i] = (outerRadius * sin(angleRad)) + CENTER_Y;
        lx[i] = (innerRadius * cos(angleRad)) + CENTER_X;
        ly[i] = (innerRadius * sin(angleRad)) + CENTER_Y;
        if (i % 30 == 0) startHour[i / 30] = i;
        if (i % 6 == 0) startMinute[i / 6] = i;
    }
    Serial.println("Display Manager Initialized.");
    return true;
}

// --- Quản lý Task ---
void DisplayManager::startTask(UBaseType_t priority) {
    xTaskCreate(taskFunction, "DisplayTask", 4096, this, priority, &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating Display Task!");
    else Serial.println("Display Task started.");
}

void DisplayManager::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle; taskHandle = NULL;
        vTaskDelete(tempHandle);
        tft.writecommand(TFT_DISPOFF); Serial.println("Display task stopped.");
    }
}

void DisplayManager::taskFunction(void* pvParameters) {
    DisplayManager* instance = static_cast<DisplayManager*>(pvParameters);
    if (instance == nullptr) { vTaskDelete(NULL); return; }
    Serial.println("Display Task running...");
    while (true) {
        instance->updateDisplay();
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

// --- Cập nhật dữ liệu ---
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
        stepCountLocal = stepCount; distanceLocal = distance;
        heartRateLocal = heartRate; spo2Local = spo2;
        temperatureLocal = temperature; pressureLocal = pressure;
        axLocal = ax; ayLocal = ay; azLocal = az;
        gxLocal = gx; gyLocal = gy; gzLocal = gz;
        xSemaphoreGive(dataMutex);
    } else { Serial.println("Timeout taking display data mutex in updateData!"); }
}

// --- Hàm Cập nhật Chính ---
void DisplayManager::updateDisplay() {
    bool localScreenOn = false;
    ScreenMode localScreenMode;
    bool needsRedraw = false;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localScreenOn = screenOn;
        localScreenMode = currentScreenMode;
        needsRedraw = needsRedrawCurrentScreen;
        if (needsRedraw) needsRedrawCurrentScreen = false;
        xSemaphoreGive(dataMutex);
    } else { return; }

    if (!localScreenOn) return;

    if (needsRedraw) {
        clearScreen();
        lastTimeString = ""; lastDateStringWF = ""; lastDayStringWF = ""; lastSecondAngleWF = -1;
        lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
        lastDateStringSens = ""; lastTempEnv = NAN; lastPresEnv = NAN; lastTimeEnv = "";
        lastAxIMU = lastAyIMU = lastAzIMU = lastGxIMU = lastGyIMU = lastGzIMU = NAN;
        lastWifiState = !wifiConnectedLocal;
    }

    switch (localScreenMode) {
        case SCREEN_MODE_WATCHFACE:     drawWatchFaceScreen(needsRedraw); break;
        case SCREEN_MODE_SENSORS_PRIMARY: drawSensorPrimaryScreen(needsRedraw); break;
        case SCREEN_MODE_ENVIRONMENT:   drawEnvironmentScreen(needsRedraw); break;
        case SCREEN_MODE_IMU_DATA:      drawImuDataScreen(needsRedraw); break;
        default: break;
    }
}

// --- Hàm Vẽ cho Từng Màn Hình ---

void DisplayManager::drawWatchFaceScreen(bool redrawStatic) {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    bool localWifiConnected = false;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localWifiConnected = wifiConnectedLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    if (redrawStatic) {
        drawClockFace();
        lastTimeString = ""; lastDateStringWF = ""; lastDayStringWF = "";
        lastSecondAngleWF = -1; lastWifiState = !localWifiConnected;
    }

    if (!isTimeValid) {
        if (lastTimeString != "WaitingWF") {
            uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
            uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
            tft.setTextColor(textColor, bgColor);
            tft.setTextDatum(MC_DATUM); tft.setTextFont(2);
            tft.fillRect(0, CENTER_Y - 20, SCREEN_WIDTH, 40, bgColor);
            tft.drawString("Waiting for", CENTER_X, CENTER_Y - 8);
            tft.drawString("Time Sync...", CENTER_X, CENTER_Y + 8);
            lastTimeString = "WaitingWF"; lastSecondAngleWF = -1; lastDateStringWF = ""; lastDayStringWF = "";
        }
        return;
    }
    if (lastTimeString == "WaitingWF") { redrawStatic = true; /* Reset last... */ }

    int angle = localTimeCopy.tm_sec * 6;
    char timeStr[9]; sprintf(timeStr, "%02d:%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, localTimeCopy.tm_sec);
    String currentTime = String(timeStr);
    String currentDay = daysOfWeek[localTimeCopy.tm_wday];
    char monthStr[4]; strftime(monthStr, sizeof(monthStr), "%b", &localTimeCopy);
    String currentFullDateStr = String(daysOfWeekShort[localTimeCopy.tm_wday]) + " " + String(localTimeCopy.tm_mday) + " " + String(monthStr);

    bool timeChanged = (currentTime != lastTimeString);
    bool dayChanged = (currentDay != lastDayStringWF);
    bool dateChanged = (currentFullDateStr != lastDateStringWF);
    bool angleChanged = (angle != lastSecondAngleWF);
    bool wifiChanged = (localWifiConnected != lastWifiState);

    if (angleChanged || timeChanged || dayChanged || redrawStatic) {
        updateWatchFaceTimeDisplay(angle, currentDay, currentTime);
    }
    if (dateChanged || redrawStatic) {
        updateWatchFaceDateDisplay(currentFullDateStr);
    }
    if (wifiChanged || redrawStatic) {
        drawWifiIcon();
    }

    lastTimeString = currentTime;
    lastDayStringWF = currentDay;
    lastDateStringWF = currentFullDateStr;
    lastSecondAngleWF = angle;
    // lastWifiState được cập nhật trong drawWifiIcon
}

void DisplayManager::drawSensorPrimaryScreen(bool redrawStatic) {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    int localSteps = 0;
    int localHR = 0;
    int localSpO2 = -999;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localSteps = stepCountLocal;
        localHR = heartRateLocal;
        localSpO2 = spo2Local;
        xSemaphoreGive(dataMutex);
    } else { return; }

    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t dateColor = textColor;
    uint16_t spo2Color = COLOR_SPO2;
    uint16_t hrColor   = COLOR_HR;
    uint16_t stepsColor= COLOR_STEPS;

    if (redrawStatic) {
        // Reset các biến tối ưu khi redraw toàn bộ
        lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
        lastDateStringSens = ""; // Reset biến cho màn hình này
    }

    int startY = 30;
    int lineSpacing = 55;
    int dataX_Label = 15;
    int dataX_Value = SCREEN_WIDTH - 15;
    int dataWidth = SCREEN_WIDTH - 30;

    // 1. Steps
    // Cần truyền tham chiếu lastDisplayedSteps& vào hàm optimized
    drawIntOptimized(localSteps, "", dataX_Value, startY + 2, 4, textColor, bgColor, TR_DATUM, lastDisplayedSteps, "--", dataWidth);
    if (redrawStatic || localSteps != lastDisplayedSteps ) { // Chỉ vẽ lại nhãn nếu cần
        tft.setTextColor(stepsColor, bgColor); tft.setTextDatum(TL_DATUM); tft.setTextFont(4);
        tft.drawString("Steps", dataX_Label, startY + 2);
    }

    // 2. Heart Rate
    startY += lineSpacing;
    drawIntOptimized(localHR, " BPM", dataX_Value, startY + 2, 4, textColor, bgColor, TR_DATUM, lastDisplayedHR, "--", dataWidth);
     if (redrawStatic || localHR != lastDisplayedHR) {
        tft.setTextColor(hrColor, bgColor); tft.setTextDatum(TL_DATUM); tft.setTextFont(4);
        tft.drawString("HR", dataX_Label, startY + 2);
    }

    // 3. SpO2
    startY += lineSpacing;
    drawIntOptimized(localSpO2, " %", dataX_Value, startY + 2, 4, textColor, bgColor, TR_DATUM, lastDisplayedSpO2, "--", dataWidth, SPO2_MIN);
    if (redrawStatic || localSpO2 != lastDisplayedSpO2) {
        tft.setTextColor(spo2Color, bgColor); tft.setTextDatum(TL_DATUM); tft.setTextFont(4);
        tft.drawString("SpO2", dataX_Label, startY + 2);
    }

    // 4. Thời gian và Ngày (HH:MM | DAY DD) ở dưới
    int bottomY = SCREEN_HEIGHT - 25;
    if (isTimeValid) {
        char dateTimeStr[20];
        sprintf(dateTimeStr, "%02d:%02d | %s %02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, daysOfWeekShort[localTimeCopy.tm_wday], localTimeCopy.tm_mday);
        String currentDateTime = String(dateTimeStr);
        drawStringOptimized(currentDateTime, CENTER_X, bottomY, 2, dateColor, bgColor, MC_DATUM, lastDateStringSens, SCREEN_WIDTH);
    } else {
        drawStringOptimized("TIME N/A", CENTER_X, bottomY, 2, dateColor, bgColor, MC_DATUM, lastDateStringSens, SCREEN_WIDTH);
    }

    tft.setTextDatum(MC_DATUM); // Reset datum
}

void DisplayManager::drawEnvironmentScreen(bool redrawStatic) {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    float localTemp = NAN;
    float localPres = NAN;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localTemp = temperatureLocal;
        localPres = pressureLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t tempColor = COLOR_TEMP;
    uint16_t presColor = COLOR_PRESSURE;

    if (redrawStatic) {
        lastTempEnv = NAN; lastPresEnv = NAN;
        lastTimeEnv = "";
    }

    int startY = 50;
    int lineSpacing = 60;
    int dataX_Label = 10;
    int dataX_Value = SCREEN_WIDTH - 10;
    int dataWidth = SCREEN_WIDTH - 20;

    // 1. Temperature
    drawFloatOptimized(localTemp, 1, " C", dataX_Value, startY + 2, 4, textColor, bgColor, TR_DATUM, lastTempEnv, "%.1f", dataWidth);
    if (redrawStatic || localTemp != lastTempEnv) {
        tft.setTextColor(tempColor, bgColor); tft.setTextDatum(TL_DATUM); tft.setTextFont(4);
        tft.drawString("Temp", dataX_Label, startY + 2);
    }

    // 2. Pressure
    startY += lineSpacing;
    float presHpa = isnan(localPres) ? NAN : localPres / 100.0f;
    // Lưu ý: lastPresEnv lưu giá trị Pa, nên cần so sánh cẩn thận hoặc tạo biến lastPresHpaEnv
    float lastPresHpaEnv = isnan(lastPresEnv) ? NAN : lastPresEnv / 100.0f;
    drawFloatOptimized(presHpa, 1, " hPa", dataX_Value, startY + 2, 4, textColor, bgColor, TR_DATUM, lastPresHpaEnv /* Sửa lastValue */, "%.1f", dataWidth);
    if (redrawStatic || localPres != lastPresEnv) {
        tft.setTextColor(presColor, bgColor); tft.setTextDatum(TL_DATUM); tft.setTextFont(4);
        tft.drawString("Press", dataX_Label, startY + 2);
    }
    lastPresEnv = localPres; // Cập nhật giá trị Pa gốc

    // 3. Thời gian (HH:MM) ở dưới
    int bottomY = SCREEN_HEIGHT - 30;
    if (isTimeValid) {
        char timeStr[6]; sprintf(timeStr, "%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min);
        String currentTime = String(timeStr);
        drawStringOptimized(currentTime, CENTER_X, bottomY, 2, textColor, bgColor, MC_DATUM, lastTimeEnv, 80);
    } else {
        drawStringOptimized("--:--", CENTER_X, bottomY, 2, textColor, bgColor, MC_DATUM, lastTimeEnv, 80);
    }

    tft.setTextDatum(MC_DATUM);
}

void DisplayManager::drawImuDataScreen(bool redrawStatic) {
    float localAx=NAN, localAy=NAN, localAz=NAN, localGx=NAN, localGy=NAN, localGz=NAN;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localAx = axLocal; localAy = ayLocal; localAz = azLocal;
        localGx = gxLocal; localGy = gyLocal; localGz = gzLocal;
        xSemaphoreGive(dataMutex);
    } else { return; }

    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t accelColor = COLOR_ACCEL;
    uint16_t gyroColor = COLOR_GYRO;

    if (redrawStatic) {
        lastAxIMU = lastAyIMU = lastAzIMU = lastGxIMU = lastGyIMU = lastGzIMU = NAN;
    }

    int startY = 15;
    int lineSpacing = 20;
    int labelX = 10;
    int valueX = 15; // Dịch giá trị sang phải hơn
    int valueWidth = SCREEN_WIDTH - valueX - 5;

    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM); // Căn trái cho tất cả

    // Tiêu đề Accelerometer
    if (redrawStatic) {
        tft.setTextColor(accelColor, bgColor);
        tft.drawString("Accel (g)", labelX, startY);
    }
    startY += lineSpacing + 5;

    // Accel X, Y, Z
    drawFloatOptimized(localAx, 2, "", valueX, startY, 2, textColor, bgColor, TL_DATUM, lastAxIMU, "X: %.2f", valueWidth); // Thêm nhãn X:
    startY += lineSpacing;
    drawFloatOptimized(localAy, 2, "", valueX, startY, 2, textColor, bgColor, TL_DATUM, lastAyIMU, "Y: %.2f", valueWidth); // Thêm nhãn Y:
    startY += lineSpacing;
    drawFloatOptimized(localAz, 2, "", valueX, startY, 2, textColor, bgColor, TL_DATUM, lastAzIMU, "Z: %.2f", valueWidth); // Thêm nhãn Z:

    // Tiêu đề Gyroscope
    startY += lineSpacing + 10;
    if (redrawStatic) {
        tft.setTextColor(gyroColor, bgColor);
        tft.drawString("Gyro (dps)", labelX, startY);
    }
    startY += lineSpacing + 5;

    // Gyro X, Y, Z
    drawFloatOptimized(localGx, 1, "", valueX, startY, 2, textColor, bgColor, TL_DATUM, lastGxIMU, "X: %.1f", valueWidth); // Thêm nhãn X:
    startY += lineSpacing;
    drawFloatOptimized(localGy, 1, "", valueX, startY, 2, textColor, bgColor, TL_DATUM, lastGyIMU, "Y: %.1f", valueWidth); // Thêm nhãn Y:
    startY += lineSpacing;
    drawFloatOptimized(localGz, 1, "", valueX, startY, 2, textColor, bgColor, TL_DATUM, lastGzIMU, "Z: %.1f", valueWidth); // Thêm nhãn Z:

    tft.setTextDatum(MC_DATUM); // Reset
}


// --- Các Hàm Vẽ Phụ Trợ ---

// drawClockFace (Giữ nguyên)
void DisplayManager::drawClockFace() { /* ... */ }

// updateWatchFaceTimeDisplay (Giữ nguyên)
void DisplayManager::updateWatchFaceTimeDisplay(int angle, const String& currentDay, const String& currentTime) { /* ... */ }

// updateWatchFaceDateDisplay (SỬA LẠI ĐỂ NHẬN THAM CHIẾU)
void DisplayManager::updateWatchFaceDateDisplay(const String& fullDateStr) {
    uint16_t dateColor = (currentTheme == 0) ? COLOR_DATE : TFT_DARKGREY;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    int dateY = 15;

    // Gọi hàm vẽ tối ưu, truyền tham chiếu lastDateStringWF
    drawStringOptimized(fullDateStr, CENTER_X, dateY, 2, dateColor, bgColor, MC_DATUM, lastDateStringWF, SCREEN_WIDTH);
}

// clearScreen (Giữ nguyên)
void DisplayManager::clearScreen() { /* ... */ }

// drawWifiIcon (Giữ nguyên)
void DisplayManager::drawWifiIcon() { /* ... */ }


// --- Hàm Điều Khiển (Giữ nguyên) ---
void DisplayManager::toggleScreen() { /* ... */ }
void DisplayManager::switchDisplayMode() { /* ... */ }
void DisplayManager::toggleTheme() { /* ... */ }
bool DisplayManager::isScreenOn() const { return screenOn; }


// --- HÀM VẼ TỐI ƯU TIỆN ÍCH (SỬA LẠI THAM SỐ THAM CHIẾU VÀ CẬP NHẬT LAST...) ---

void DisplayManager::drawStringOptimized(const String& text, int x, int y, int font, uint16_t textColor, uint16_t bgColor, int datum,
                                         String& lastText, // <<< ĐÚNG: NHẬN THAM CHIẾU
                                         int clearWidth, int clearHeight) {
    if (text == lastText && !needsRedrawCurrentScreen) {
        return;
    }

    tft.setTextDatum(datum); tft.setTextFont(font); tft.setTextColor(textColor, bgColor);

    int textW = tft.textWidth(lastText); int textH = tft.fontHeight(font);
    if (clearWidth < 0) clearWidth = textW > 0 ? textW + 4 : 60;
    if (clearHeight < 0) clearHeight = textH > 0 ? textH + 2 : 20;
    int clearX = x, clearY = y;
    // ... (logic tính clearX, clearY dựa trên datum) ...
     switch (datum) { case TC_DATUM: case MC_DATUM: case BC_DATUM: clearX = x - clearWidth / 2; break; case TR_DATUM: case MR_DATUM: case BR_DATUM: clearX = x - clearWidth; break; }
     switch (datum) { case ML_DATUM: case MC_DATUM: case MR_DATUM: clearY = y - clearHeight / 2; break; case BL_DATUM: case BC_DATUM: case BR_DATUM: clearY = y - clearHeight; break; }


    tft.fillRect(clearX, clearY, clearWidth, clearHeight, bgColor);
    tft.drawString(text, x, y);

    // --- CẬP NHẬT lastText NGAY ĐÂY (vì là tham chiếu) ---
    lastText = text;
}

void DisplayManager::drawFloatOptimized(float value, int decimalPlaces, const String& unit, int x, int y, int font, uint16_t textColor, uint16_t bgColor, int datum,
                                        float& lastValue, // <<< ĐÚNG: NHẬN THAM CHIẾU
                                        const char* format, int clearWidth, int clearHeight) {
    char currentValStr[15];
    bool valueChanged = false;
    bool currentIsNan = isnan(value);
    bool lastIsNan = isnan(lastValue);

    if (currentIsNan) {
        strcpy(currentValStr, "--");
        if (!lastIsNan) valueChanged = true;
    } else {
        if (format) sprintf(currentValStr, format, value);
        else dtostrf(value, 0, decimalPlaces, currentValStr);
        if (lastIsNan || abs(value - lastValue) > pow(10, -decimalPlaces -1)) { // So sánh với epsilon nhỏ
             valueChanged = true;
        }
    }

    if (!valueChanged && !needsRedrawCurrentScreen) {
        // Không cần cập nhật lastValue vì nó không đổi đáng kể
        return;
    }

    String currentText = String(currentValStr) + unit;
    String lastText = "--" + unit; // Mặc định để xóa
    if (!lastIsNan) {
         char lastValStr[15];
         if (format) sprintf(lastValStr, format, lastValue);
         else dtostrf(lastValue, 0, decimalPlaces, lastValStr);
         lastText = String(lastValStr) + unit;
    }

    // Gọi hàm vẽ string (truyền lastText dạng giá trị để tính toán xóa)
    drawStringOptimized(currentText, x, y, font, textColor, bgColor, datum, lastText, clearWidth, clearHeight);

    // Cập nhật lastValue qua tham chiếu
    lastValue = value;
}

void DisplayManager::drawIntOptimized(int value, const String& unit, int x, int y, int font, uint16_t textColor, uint16_t bgColor, int datum,
                                      int& lastValue, // <<< ĐÚNG: NHẬN THAM CHIẾU
                                      const char* defaultText, int clearWidth, int clearHeight, int validThreshold) {

    if (value == lastValue && !needsRedrawCurrentScreen) {
        return;
    }

    String currentText;
    if (value >= validThreshold) currentText = String(value) + unit;
    else currentText = String(defaultText) + unit;

    String lastText;
    if (lastValue >= validThreshold) lastText = String(lastValue) + unit;
    else lastText = String(defaultText) + unit;

    // Gọi hàm vẽ string
    drawStringOptimized(currentText, x, y, font, textColor, bgColor, datum, lastText, clearWidth, clearHeight);

    // Cập nhật lastValue qua tham chiếu
    lastValue = value;
}