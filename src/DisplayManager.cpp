// src/DisplayManager.cpp
#include "DisplayManager.h"
#include <fonts.h> 
#include "Config.h"
#include <Arduino.h>
#include <WiFi.h>    // Chỉ cần nếu vẽ icon WiFi

// --- Hằng số UI và Màu sắc (Được định nghĩa trong .h) ---

// --- Mảng dữ liệu tĩnh ---
const String daysOfWeek[7] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
// Nhãn phút/giây cho vòng xoay (Thứ tự: 00 ở 12h, 05 ở 1h,...)
const String hourLabels[12] = {"00", "55", "50", "45", "40", "35", "30", "25", "20", "15", "10", "05"};

const String daysOfWeekShort[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"}; // Tên viết tắt

// --- Constructor ---
DisplayManager::DisplayManager()
    : tft(), taskHandle(NULL), screenOn(true), currentTheme(0),
      currentScreenMode(SCREEN_MODE_WATCHFACE), // Bắt đầu với màn hình đồng hồ
      timeInitializedLocal(false), wifiConnectedLocal(false),
      stepCountLocal(0), distanceLocal(0.0f), heartRateLocal(0), spo2Local(-999),
      lastTimeString(""), lastDateString(""), lastDayString(""), lastSecondAngle(-1),
      lastDisplayedSteps(-1), lastDisplayedHR(-1), lastDisplayedSpO2(-1000),
      lastWifiState(false), // Khởi tạo biến tối ưu WiFi
      needsRedrawWatchFace(true), // <-- Khởi tạo cờ redraw
      needsRedrawSensorScreen(true)
{
    memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("Error creating Display data mutex!");
    }
}

// --- Hàm begin() ---
void DisplayManager::begin() {
    Serial.println("Initializing Display Manager...");
    tft.init();
    tft.setRotation(0);
    tft.setSwapBytes(true);
    tft.fillScreen(DARK_BACKGROUND); // Chỉ xóa nền ban đầu

    // Tính toán tọa độ điểm cho mặt đồng hồ
    Serial.println("Calculating Watch Face UI points...");
    for (int i = 0; i < NUM_POINTS; i++) {
        float angleRad = radians(i - 90); // Bắt đầu từ 12h (góc -90)
        x[i] = (RADIUS * cos(angleRad)) + CENTER_X;
        y[i] = (RADIUS * sin(angleRad)) + CENTER_Y;
        px[i] = ((RADIUS - 16) * cos(angleRad)) + CENTER_X;
        py[i] = ((RADIUS - 16) * sin(angleRad)) + CENTER_Y;
        lx[i] = ((RADIUS - 26) * cos(angleRad)) + CENTER_X;
        ly[i] = ((RADIUS - 26) * sin(angleRad)) + CENTER_Y;
        if (i % 30 == 0) startHour[i / 30] = i;
        if (i % 6 == 0) startMinute[i / 6] = i;
    }
    Serial.println("Display Manager Initialized.");
    // Không vẽ gì ở đây, Task sẽ vẽ màn hình đầu tiên
}

// --- Quản lý Task (start, stop, function) ---
void DisplayManager::startTask() {
    xTaskCreate(taskFunction, "DisplayTask", 4096, this, 1, &taskHandle);
    if (taskHandle == NULL) Serial.println("Error creating Display Task!");
}

void DisplayManager::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle); taskHandle = NULL;
        tft.writecommand(TFT_DISPOFF); Serial.println("Display task stopped.");
    }
}

void DisplayManager::taskFunction(void* pvParameters) {
    DisplayManager* instance = static_cast<DisplayManager*>(pvParameters);
    while (true) {
        instance->updateDisplay();
        vTaskDelay(100 / portTICK_PERIOD_MS); // 10Hz
    }
}

// --- Cập nhật dữ liệu ---
void DisplayManager::updateData(bool wifiConnected,
                                int stepCount, float distance,
                                int heartRate, int spo2,
                                const struct tm* timeinfo, bool timeInitialized) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        wifiConnectedLocal = wifiConnected;
        timeInitializedLocal = timeInitialized;
        if (timeInitialized && timeinfo != nullptr) timeinfoLocal = *timeinfo;
        stepCountLocal = stepCount; distanceLocal = distance;
        heartRateLocal = heartRate; spo2Local = spo2;
        xSemaphoreGive(dataMutex);
    } else { Serial.println("Timeout taking display data mutex in updateData!"); }
}

// --- Hàm Cập nhật Chính ---
void DisplayManager::updateDisplay() {
    bool localScreenOn = false;
    ScreenMode localScreenMode;

    // Lấy trạng thái màn hình và mode hiện tại
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localScreenOn = screenOn;
        localScreenMode = currentScreenMode;
        xSemaphoreGive(dataMutex);
    } else { Serial.println("Timeout taking display data mutex in updateDisplay!"); return; }

    // Thoát nếu màn hình tắt
    if (!localScreenOn) return;

    // Gọi hàm vẽ tương ứng
    switch (localScreenMode) {
        case SCREEN_MODE_WATCHFACE:
            drawWatchFaceScreen();
            break;
        case SCREEN_MODE_SENSORS:
            drawSensorDataScreen();
            break;
        default: // Xử lý trường hợp không hợp lệ (nếu có)
            clearScreen();
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("INVALID SCREEN", CENTER_X, CENTER_Y, 2);
            break;
    }
}


void DisplayManager::drawWatchFaceScreen() {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    bool localWifiConnected = false;
    bool redrawAllStatic = false;

    // Lấy dữ liệu và cờ redraw
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localWifiConnected = wifiConnectedLocal;
        redrawAllStatic = needsRedrawWatchFace;
        if (redrawAllStatic) needsRedrawWatchFace = false;
        xSemaphoreGive(dataMutex);
    } else { return; }

    // Vẽ lại nền tĩnh nếu cờ yêu cầu
    if (redrawAllStatic) {
        clearScreen();
        drawClockFace(); // Hàm này cần đảm bảo đặt lại font/size cuối cùng
        lastTimeString = ""; lastDateString = ""; lastDayString = "";
        lastSecondAngle = -1; lastWifiState = !localWifiConnected;
    }

    // Xử lý màn hình chờ
    if (!isTimeValid) {
        if (lastTimeString != "Waiting") {
            clearScreen();
            uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
            uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
            tft.setTextColor(textColor, bgColor);
            tft.setTextDatum(MC_DATUM);
            // ----> ĐẶT FONT/SIZE CHO CHỮ CHỜ <----
            tft.setTextFont(2); // Ví dụ: Font 2
            tft.drawString("Connect via Bluetooth", CENTER_X, CENTER_Y - 8);
            tft.drawString("to get time.", CENTER_X, CENTER_Y + 8);
            lastTimeString = "Waiting"; lastSecondAngle = -1; lastDateString = ""; lastDayString = "";
        }
        return;
    }

    // --- Thời gian hợp lệ ---
    if (lastTimeString == "Waiting") {
        Serial.println("Time is now valid, redrawing static clock face.");
        clearScreen();
        drawClockFace(); // Hàm này vẽ phần tĩnh
        lastTimeString = ""; lastDateString = ""; lastDayString = "";
        lastSecondAngle = -1; lastWifiState = !localWifiConnected;
         // Đảm bảo phần động cũng được vẽ lại hoàn toàn
         redrawAllStatic = true; // Đặt lại cờ này để các phần dưới vẽ lại
    }

    // --- Vẽ phần động ---
    int angle = localTimeCopy.tm_sec * 6;
    char timeStr[9]; sprintf(timeStr, "%02d:%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, localTimeCopy.tm_sec);
    char dateStr[6]; sprintf(dateStr, "%02d:%02d", localTimeCopy.tm_mon + 1, localTimeCopy.tm_mday);
    String currentTime = String(timeStr);
    String currentDate = String(dateStr);
    String currentDay = daysOfWeek[localTimeCopy.tm_wday];

    bool timeChanged = (currentTime != lastTimeString);
    bool dayChanged = (currentDay != lastDayString);
    bool dateChanged = (currentDate != lastDateString);
    bool angleChanged = (angle != lastSecondAngle);
    bool wifiChanged = (localWifiConnected != lastWifiState);

    // Cập nhật phần thời gian động (vòng xoay, giờ số, ngày chữ)
    // Hàm updateTimeDisplay cần đảm bảo set đúng font/size bên trong nó
    if (angleChanged || timeChanged || dayChanged || redrawAllStatic) { // Thêm redrawAllStatic
        updateTimeDisplay(angle, currentDay, currentTime);
    }
    // Cập nhật hộp ngày/tháng
    // Hàm updateDateDisplay cần đảm bảo set đúng font/size bên trong nó
    if (dateChanged || redrawAllStatic) {
        updateDateDisplay(currentDate);
    }
    // Cập nhật icon WiFi
    // Hàm drawWifiIcon cần đảm bảo set đúng font/size bên trong nó
    if (wifiChanged || redrawAllStatic) {
        drawWifiIcon();
    }

    // Lưu trạng thái cuối
    lastTimeString = currentTime; lastDayString = currentDay;
    lastDateString = currentDate; lastSecondAngle = angle;
}

// ===== HÀM VẼ MÀN HÌNH SENSOR ĐÃ SỬA LẠI BỐ CỤC =====
void DisplayManager::drawSensorDataScreen() {
    struct tm localTimeCopy;
    bool isTimeValid = false;
    int localSteps = 0;
    int localHR = 0;
    int localSpO2 = -999;
    bool redrawAll = false; // Cờ redraw cục bộ

    // Lấy dữ liệu và cờ redraw
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        isTimeValid = timeInitializedLocal;
        if (isTimeValid) localTimeCopy = timeinfoLocal;
        localSteps = stepCountLocal;
        localHR = heartRateLocal;
        localSpO2 = spo2Local;
        // localWifiConnected = wifiConnectedLocal; // Vẫn lấy nhưng chưa dùng
        redrawAll = needsRedrawSensorScreen;
        if (redrawAll) needsRedrawSensorScreen = false; // Reset cờ
        xSemaphoreGive(dataMutex);
    } else { return; }

    // --- Vẽ lại nền nếu cần ---
    if (redrawAll) {
        clearScreen(); // Xóa màn hình
        // Reset các biến tối ưu vẽ
        lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
        lastTimeString = ""; lastDateString = ""; // Reset cả date string
    }

    // --- Thiết lập màu sắc ---
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t timeColor = (currentTheme == 0) ? COLOR_TIME_DARK : COLOR_TIME_LIGHT;
    // uint16_t dateColor = (currentTheme == 0) ? COLOR_DATE : TFT_DARKGREY; // Màu ngày tháng ở vị trí mới
    uint16_t dateColor = textColor; // Cho ngày tháng cùng màu text chính
    uint16_t spo2Color = (currentTheme == 0) ? COLOR_SPO2 : tft.color565(0, 150, 150);
    uint16_t hrColor   = (currentTheme == 0) ? COLOR_HR : tft.color565(200, 0, 100);
    uint16_t stepsColor= (currentTheme == 0) ? COLOR_STEPS : tft.color565(200, 150, 0);

    char buffer[20];

    // --- Vẽ Thời gian lớn (HH:MM) - Giữ nguyên ở cột trái ---
    int leftColumnX = 100;
    int timeY_Line1 = 55;
    int timeTextHeight = 48; // Ước tính Font 1 Size 6
    int timeY_Line2 = timeY_Line1 + timeTextHeight + 2; // Y cho MM = 105
    int timeBottomY = timeY_Line2 + timeTextHeight; // Y đáy của MM = 153

    if (isTimeValid) {
        char hourStr[3]; sprintf(hourStr, "%02d", localTimeCopy.tm_hour);
        char minStr[3]; sprintf(minStr, "%02d", localTimeCopy.tm_min);
        String currentTime = String(hourStr) + String(minStr);
        if (redrawAll || lastTimeString != currentTime) {
            int timeClearWidth = 80;
            tft.fillRect(leftColumnX - timeClearWidth, timeY_Line1, timeClearWidth, timeBottomY - timeY_Line1, bgColor);
            tft.setTextColor(timeColor, bgColor);
            tft.setTextDatum(TR_DATUM);
            tft.setTextFont(1); tft.setTextSize(6);
            tft.drawString(hourStr, leftColumnX, timeY_Line1);
            tft.drawString(minStr, leftColumnX, timeY_Line2);
            lastTimeString = currentTime;
        }
    } else {
        if (lastTimeString != "Wait") {
            int timeClearWidth = 80;
            tft.fillRect(leftColumnX - timeClearWidth, timeY_Line1, timeClearWidth, timeBottomY - timeY_Line1, bgColor);
            tft.setTextColor(timeColor, bgColor); tft.setTextDatum(TR_DATUM);
            tft.setTextFont(1); tft.setTextSize(6);
            tft.drawString("--", leftColumnX, timeY_Line1);
            tft.drawString("--", leftColumnX, timeY_Line2);
            lastTimeString = "Wait";
        }
    }

    // --- Vẽ Cột Phải: Chỉ số cảm biến VÀ Ngày tháng ---
    int rightColumnX = leftColumnX + 20; // X bắt đầu cột phải = 120
    int sensorY = timeY_Line1 - 5;       // Y bắt đầu cho dòng đầu tiên (SpO2), dịch lên chút
    int sensorLineHeight = 28;        // Khoảng cách Y giữa các dòng
    int sensorTextWidth = 110;        // Chiều rộng ước tính để xóa

    tft.setTextDatum(TL_DATUM); // Căn trái Top Left
    tft.setTextFont(2);         // Font 2 cho tất cả ở cột phải
    tft.setTextSize(1);         // Size 1

    // 1. SpO2
    if (localSpO2 != lastDisplayedSpO2 || redrawAll) {
        tft.fillRect(rightColumnX, sensorY, sensorTextWidth, 18, bgColor); // Xóa vùng cũ
        tft.setTextColor(spo2Color, bgColor);
        if (localSpO2 >= SPO2_MIN) sprintf(buffer, "SpO2 %3d %%", localSpO2);
        else strcpy(buffer, "SpO2 --- %");
        tft.drawString(buffer, rightColumnX, sensorY);
        lastDisplayedSpO2 = localSpO2;
    }

    // 2. Heart Rate
    sensorY += sensorLineHeight;
    if (localHR != lastDisplayedHR || redrawAll) {
        tft.fillRect(rightColumnX, sensorY, sensorTextWidth, 18, bgColor);
        tft.setTextColor(hrColor, bgColor);
        if (localHR > 0) sprintf(buffer, "HR ❤️  %3d BPM", localHR); // Có thể dùng icon tim
        else strcpy(buffer, "HR   --- BPM");
        tft.drawString(buffer, rightColumnX, sensorY);
        lastDisplayedHR = localHR;
    }

    // 3. Steps
    sensorY += sensorLineHeight;
    if (localSteps != lastDisplayedSteps || redrawAll) {
        tft.fillRect(rightColumnX, sensorY, sensorTextWidth, 18, bgColor);
        tft.setTextColor(stepsColor, bgColor);
        sprintf(buffer, "Steps %5d", localSteps); // 5 chữ số
        tft.drawString(buffer, rightColumnX, sensorY);
        lastDisplayedSteps = localSteps;
    }

    // 4. Ngày tháng (MM-DD DAY) - Vẽ ở dòng tiếp theo cột phải
    sensorY += sensorLineHeight;
    if (isTimeValid) {
        char dateStr[12]; sprintf(dateStr, "%02d-%02d %s", localTimeCopy.tm_mday, localTimeCopy.tm_mon + 1, daysOfWeekShort[localTimeCopy.tm_wday]);
        String currentDate = String(dateStr);
        if (redrawAll || lastDateString != currentDate) {
            tft.fillRect(rightColumnX, sensorY, sensorTextWidth, 18, bgColor); // Xóa vùng ngày cũ
            tft.setTextColor(dateColor, bgColor); // Màu cho ngày tháng
            tft.setTextFont(2); // Đảm bảo đúng font/size
            tft.setTextSize(1);
            tft.setTextDatum(TL_DATUM); // Căn trái
            tft.drawString(currentDate, rightColumnX, sensorY); // Vẽ ở vị trí mới
            lastDateString = currentDate;
        }
    } else { // Thời gian không hợp lệ, xóa vùng ngày tháng
        if (lastDateString != "Wait") {
             tft.fillRect(rightColumnX, sensorY, sensorTextWidth, 18, bgColor);
             lastDateString = "Wait";
        }
    }

    // --- Đặt lại Font/Size/Datum ---
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
}

// --- Các Hàm Vẽ Phụ Trợ ---

// drawClockFace (Vẽ nền tĩnh của mặt đồng hồ)
void DisplayManager::drawClockFace() {
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t linesColor = (currentTheme == 0) ? DARK_LINES : LIGHT_LINES;
    uint16_t boxColor = (currentTheme == 0) ? DARK_BOX : LIGHT_BOX;
    uint16_t highlightColor = (currentTheme == 0) ? DARK_HIGHLIGHT : LIGHT_HIGHLIGHT;

    // Không fillScreen ở đây vì nó được gọi sau clearScreen() trong drawWatchFaceScreen

    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(textColor, bgColor);

    // Vẽ hộp ngày tháng
    tft.fillRect(70, 86, 12, 20, boxColor); tft.fillRect(84, 86, 12, 20, boxColor);
    tft.fillRect(150, 86, 12, 20, boxColor); tft.fillRect(164, 86, 12, 20, boxColor);
    // Vẽ nhãn MONTH/DAY
    tft.setTextFont(1); tft.setTextDatum(BC_DATUM);
    tft.drawString("MONTH", 84, 84); tft.drawString("DAY", 162, 84);
    // Vẽ các chữ tĩnh khác
    tft.setTextDatum(MC_DATUM); tft.setTextColor(highlightColor, bgColor);
    tft.setTextFont(1); tft.drawString("ESP32 WATCH", CENTER_X, 174);
    tft.setTextFont(4); tft.drawString("***", CENTER_X, 108);
    // Vẽ các chấm phút
    for (int i = 0; i < 60; i++) {
        tft.fillCircle(px[startMinute[i]], py[startMinute[i]], (i % 5 == 0) ? 2 : 1, linesColor);
    }
    // Vẽ tam giác chỉ 12h
    tft.fillTriangle(CENTER_X, CENTER_Y - RADIUS + 5, CENTER_X - 5, CENTER_Y - RADIUS + 15, CENTER_X + 5, CENTER_Y - RADIUS + 15, highlightColor);
    tft.setTextDatum(MC_DATUM); // Reset datum
}

// updateTimeDisplay (Vẽ phần động của mặt đồng hồ)
void DisplayManager::updateTimeDisplay(int angle, const String& currentDay, const String& currentTime) {
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
    uint16_t linesColor = (currentTheme == 0) ? DARK_LINES : LIGHT_LINES;

    // --- Phần xoay nhãn ---
    if (angle != lastSecondAngle) {
        tft.setTextFont(2); tft.setTextDatum(MC_DATUM);
        for (int i = 0; i < 12; i++) {
            // --- Logic vẽ nhãn xoay (Giữ nguyên) ---
            if (lastSecondAngle != -1) { /* Xóa cũ */
                int oldPos = (startHour[i] + lastSecondAngle) % 360;
                tft.setTextColor(bgColor, bgColor);
                tft.drawString(hourLabels[i], x[oldPos], y[oldPos]);
                tft.drawLine(px[oldPos], py[oldPos], lx[oldPos], ly[oldPos], bgColor);
            }
            /* Vẽ mới */
            int newPos = (startHour[i] + angle) % 360;
            tft.setTextColor(textColor, bgColor);
            tft.drawString(hourLabels[i], x[newPos], y[newPos]);
            tft.drawLine(px[newPos], py[newPos], lx[newPos], ly[newPos], linesColor);
        }
    }
    // --- Cập nhật ngày trong tuần ---
    if (currentDay != lastDayString) {
        tft.setTextFont(2); tft.setTextDatum(MC_DATUM);
        tft.fillRect(CENTER_X - 40, CENTER_Y - 8, 80, 16, bgColor); // Xóa nền
        tft.setTextColor(textColor, bgColor);
        tft.drawString(currentDay, CENTER_X, CENTER_Y);
    }
    // --- Cập nhật giờ và giây số ---
    if (currentTime != lastTimeString) {
        tft.setTextDatum(MC_DATUM);
        // --- Logic xóa/vẽ giờ:phút:giây (Giữ nguyên) ---
        /* Xóa giây cũ */
        tft.setFreeFont(&DSEG7_Modern_Bold_20); tft.setTextColor(bgColor, bgColor);
        tft.drawString(lastTimeString.substring(6, 8), CENTER_X, CENTER_Y - 36);
        /* Xóa giờ:phút cũ */
        tft.setFreeFont(&DSEG7_Classic_Regular_28);
        tft.drawString(lastTimeString.substring(0, 5), CENTER_X, CENTER_Y + 28);
        /* Vẽ giây mới */
        tft.setFreeFont(&DSEG7_Modern_Bold_20); 
        tft.setTextColor(textColor, bgColor);
        tft.drawString(currentTime.substring(6, 8), CENTER_X, CENTER_Y - 36);
        /* Vẽ giờ:phút mới */
        tft.setFreeFont(&DSEG7_Classic_Regular_28);
        tft.drawString(currentTime.substring(0, 5), CENTER_X, CENTER_Y + 28);
    }
    tft.setTextFont(0); // Reset font
}

// updateDateDisplay (Cập nhật hộp ngày/tháng)
void DisplayManager::updateDateDisplay(const String& currentDate) {
    uint16_t boxColor = (currentTheme == 0) ? DARK_BOX : LIGHT_BOX;
    uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;

    tft.setTextFont(2); tft.setTextDatum(MC_DATUM);
    tft.setTextColor(textColor, boxColor);

    // Tháng (MM)
    tft.drawString(currentDate.substring(0, 1), 76, 96);
    tft.drawString(currentDate.substring(1, 2), 90, 96);
    // Ngày (DD)
    tft.drawString(currentDate.substring(3, 4), 156, 96);
    tft.drawString(currentDate.substring(4, 5), 170, 96);

    tft.setTextFont(0); // Reset font
}

// --- Hàm Điều Khiển ---

// toggleScreen
void DisplayManager::toggleScreen() {
    bool turnOn;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        screenOn = !screenOn;
        turnOn = screenOn;
        // Khi bật màn hình, đặt cờ redraw cho màn hình HIỆN TẠI
        if (turnOn) {
            if (currentScreenMode == SCREEN_MODE_WATCHFACE) needsRedrawWatchFace = true;
            else if (currentScreenMode == SCREEN_MODE_SENSORS) needsRedrawSensorScreen = true;
            // Reset biến tối ưu để vẽ lại hết
             lastTimeString = ""; lastDateString = ""; lastDayString = ""; lastSecondAngle = -1;
             lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
             lastWifiState = !wifiConnectedLocal; // Đảo để vẽ lại icon
        }
        xSemaphoreGive(dataMutex);

        // Thực hiện ngoài mutex
        if (turnOn) { tft.writecommand(TFT_DISPON); Serial.println("Screen turned ON"); }
        else { tft.writecommand(TFT_DISPOFF); Serial.println("Screen turned OFF"); }
    } else { Serial.println("Timeout taking display data mutex in toggleScreen!"); }
}

// switchDisplayMode
void DisplayManager::switchDisplayMode() {
    if (!isScreenOn()) return; // Chỉ chuyển khi màn hình bật

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentScreenMode = (ScreenMode)((currentScreenMode + 1) % SCREEN_MODE_COUNT);
        ScreenMode newMode = currentScreenMode;
        // Đặt cờ redraw cho màn hình MỚI
        if (newMode == SCREEN_MODE_WATCHFACE) needsRedrawWatchFace = true;
        else if (newMode == SCREEN_MODE_SENSORS) needsRedrawSensorScreen = true;
        // Reset biến tối ưu
         lastTimeString = ""; lastDateString = ""; lastDayString = ""; lastSecondAngle = -1;
         lastDisplayedSteps = -1; lastDisplayedHR = -1; lastDisplayedSpO2 = -1000;
         lastWifiState = !wifiConnectedLocal;

        xSemaphoreGive(dataMutex);
        Serial.printf("Switched to screen mode: %d\n", newMode);
    } else { Serial.println("Timeout taking display data mutex in switchDisplayMode!"); }
}

// isScreenOn (Giữ nguyên)
bool DisplayManager::isScreenOn() const { return screenOn; }

// drawWifiIcon (Giữ nguyên như đã sửa)
void DisplayManager::drawWifiIcon() {
    bool isConnected;
    // static bool lastWifiState = !wifiConnectedLocal; // Chuyển thành biến thành viên

    // Lấy trạng thái wifi an toàn
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) { // Timeout ngắn
        isConnected = wifiConnectedLocal;
        xSemaphoreGive(dataMutex);
    } else {
        return; // Bỏ qua nếu không lấy được mutex nhanh
    }


    if (isConnected != lastWifiState) { // Chỉ vẽ lại nếu trạng thái thay đổi
        uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
        // Nên vẽ icon ở vị trí cố định trên cả 2 màn hình nếu muốn
        int iconX = 210; int iconY = 10; int iconW = 25; int iconH = 16;

        tft.fillRect(iconX, iconY, iconW, iconH, bgColor); // Xóa vùng icon cũ

        if (isConnected) {
            tft.setTextColor((currentTheme == 0) ? TFT_GREEN : TFT_BLUE, bgColor); // Màu icon
            tft.setTextDatum(TR_DATUM); // Top Right datum
            tft.drawString("WiFi", iconX + iconW - 1 , iconY, 2); // Font 2
        }
        lastWifiState = isConnected; // Cập nhật trạng thái đã vẽ
        tft.setTextDatum(MC_DATUM); // Reset datum về mặc định
    }
}

void DisplayManager::clearScreen() {
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    tft.fillScreen(bgColor);
}

void DisplayManager::toggleTheme() {
    // Chỉ đổi theme nếu màn hình đang bật
    if (!isScreenOn()) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentTheme = !currentTheme; // Đảo theme (0 -> 1, 1 -> 0)
        int localTheme = currentTheme;

        // Đặt cờ redraw cho màn hình hiện tại để vẽ lại với theme mới
        if (currentScreenMode == SCREEN_MODE_WATCHFACE) {
            needsRedrawWatchFace = true;
        } else if (currentScreenMode == SCREEN_MODE_SENSORS) {
            needsRedrawSensorScreen = true;
        }
        // Reset các biến tối ưu để buộc vẽ lại hoàn toàn
        lastTimeString = ""; lastDateString = ""; lastDayString = "";
        lastSecondAngle = -1; lastDisplayedSteps = -1; lastDisplayedHR = -1;
        lastDisplayedSpO2 = -1000; lastWifiState = !wifiConnectedLocal;

        xSemaphoreGive(dataMutex);

        Serial.printf("Theme toggled to: %s\n", localTheme == 0 ? "Dark" : "Light");
        // Không cần vẽ lại ngay, DisplayTask sẽ tự xử lý
    } else {
        Serial.println("Timeout taking display data mutex in toggleTheme!");
    }
}
// tft.setFreeFont(&DSEG7_Modern_Bold_20); 