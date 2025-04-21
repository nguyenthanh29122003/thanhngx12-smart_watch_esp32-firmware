// src/DisplayManager.cpp
#include "DisplayManager.h"
#include <fonts.h>
#include "Config.h" // Có thể cần cho các hằng số khác
#include <Arduino.h> // Cần cho radians, cos, sin, sprintf, millis
#include <WiFi.h>    // Để dùng TFT_eSPI
// --- Đảm bảo các hằng số này được định nghĩa ---
// (Có thể đã có trong DisplayManager.h hoặc Config.h)
#ifndef CENTER_X
#define CENTER_X 120
#endif
#ifndef CENTER_Y
#define CENTER_Y 120
#endif
#ifndef RADIUS
#define RADIUS 104
#endif
#ifndef NUM_POINTS
#define NUM_POINTS 360
#endif
// Màu sắc và font cũng cần được định nghĩa
// Mảng tên ngày và nhãn giờ
const String daysOfWeek[7] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
// Sửa lại cho đúng thứ tự: 12 ở trên cùng (270 độ), 3 ở bên phải (0 độ)
// Góc 0: 3h, Góc 30: 2h, ..., Góc 270: 12h, Góc 300: 11h, ...
// Hoặc dùng label phút/giây như code gốc:
const String hourLabels[12] = {"00", "55", "50", "45", "40", "35", "30", "25", "20", "15", "10", "05"};
const String minuteLabels[12] = {"0", "5", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55"};
// Constructor
DisplayManager::DisplayManager()
: tft(), taskHandle(NULL), screenOn(true), currentTheme(0), // Bắt đầu Dark Mode
timeInitializedLocal(false), wifiConnectedLocal(false),
lastTimeString(""), lastDateString(""), lastDayString(""), lastSecondAngle(-1)
{
memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
dataMutex = xSemaphoreCreateMutex();
if (dataMutex == NULL) {
Serial.println("Error creating Display data mutex!");
}
}
// Hàm begin()
void DisplayManager::begin() {
Serial.println("Initializing Display Manager...");
tft.init();
tft.setRotation(0); // Đảm bảo đúng hướng màn hình
tft.setSwapBytes(true); // Quan trọng cho font và màu
tft.fillScreen(DARK_BACKGROUND);
// Tính toán tọa độ các điểm (chỉ làm một lần)
Serial.println("Calculating UI points...");
for (int i = 0; i < NUM_POINTS; i++) {
    float angleRad = radians(i - 90); // Bắt đầu từ 12h (góc -90 độ)
    x[i] = (RADIUS * cos(angleRad)) + CENTER_X;
    y[i] = (RADIUS * sin(angleRad)) + CENTER_Y;
    px[i] = ((RADIUS - 16) * cos(angleRad)) + CENTER_X;
    py[i] = ((RADIUS - 16) * sin(angleRad)) + CENTER_Y;
    lx[i] = ((RADIUS - 26) * cos(angleRad)) + CENTER_X;
    ly[i] = ((RADIUS - 26) * sin(angleRad)) + CENTER_Y;

    // Lưu chỉ số bắt đầu cho nhãn giờ (mỗi 30 độ)
    if (i % 30 == 0) startHour[i / 30] = i;
    // Lưu chỉ số bắt đầu cho vạch phút (mỗi 6 độ)
    if (i % 6 == 0) startMinute[i / 6] = i;
}

// Vẽ giao diện tĩnh lần đầu
drawStaticUI();
Serial.println("Display Manager Initialized.");

}
// Hàm startTask()
void DisplayManager::startTask() {
xTaskCreate(
taskFunction,       // Hàm task
"DisplayTask",      // Tên task
4096,               // Tăng Stack size
this,               // Tham số
1,                  // Độ ưu tiên
&taskHandle         // Handle
);
if (taskHandle == NULL) {
Serial.println("Error creating Display Task!");
}
}
// Hàm stopTask()
void DisplayManager::stopTask() {
if (taskHandle != NULL) {
vTaskDelete(taskHandle);
taskHandle = NULL;
tft.writecommand(TFT_DISPOFF); // Tắt màn hình
Serial.println("Display task stopped.");
}
}
// Hàm taskFunction()
void DisplayManager::taskFunction(void* pvParameters) {
DisplayManager* instance = static_cast<DisplayManager*>(pvParameters);
while (true) {
instance->updateDisplay();
// Delay ngắn hơn để cập nhật giây mượt hơn
vTaskDelay(100 / portTICK_PERIOD_MS); // 10Hz
}
}
// Hàm updateData()
void DisplayManager::updateData(bool wifiConnected, const struct tm* timeinfo, bool timeInitialized) {
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
wifiConnectedLocal = wifiConnected; // Cập nhật trạng thái WiFi
timeInitializedLocal = timeInitialized;
if (timeInitialized && timeinfo != nullptr) {
timeinfoLocal = *timeinfo;
}
xSemaphoreGive(dataMutex);
} else {
Serial.println("Timeout taking display data mutex in updateData!");
}
}
// Hàm updateDisplay()
void DisplayManager::updateDisplay() {
bool localScreenOn = false;
struct tm localTimeCopy;
bool isTimeValid = false;
bool localWifiConnected = false;
// Lấy trạng thái và dữ liệu an toàn
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    localScreenOn = screenOn;
    isTimeValid = timeInitializedLocal;
    if (isTimeValid) {
        localTimeCopy = timeinfoLocal;
    }
    localWifiConnected = wifiConnectedLocal; // Lấy trạng thái WiFi
    xSemaphoreGive(dataMutex);
} else {
     Serial.println("Timeout taking display data mutex in updateDisplay!");
     return;
}

// Chỉ vẽ nếu màn hình đang bật
if (!localScreenOn) {
    return;
}

// Nếu thời gian chưa sẵn sàng
if (!isTimeValid) {
     uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
     uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
     // Chỉ vẽ lại màn hình chờ nếu trạng thái "lastTimeString" khác "Waiting"
     if (lastTimeString != "Waiting") {
         tft.fillScreen(bgColor);
         tft.setTextColor(textColor, bgColor);
         tft.setTextDatum(MC_DATUM);
         tft.drawString("Waiting for time...", CENTER_X, CENTER_Y, 2); // Font 2
         lastTimeString = "Waiting"; // Đánh dấu đã vẽ màn hình chờ
         lastSecondAngle = -1; // Reset các trạng thái khác
         lastDateString = "";
         lastDayString = "";
     }
     return;
}

// Thời gian hợp lệ, tiến hành vẽ đồng hồ
int angle = localTimeCopy.tm_sec * 6; // Góc = giây * 6

char timeStr[9]; // HH:MM:SS
char dateStr[6]; // MM:DD
sprintf(timeStr, "%02d:%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, localTimeCopy.tm_sec);
sprintf(dateStr, "%02d:%02d", localTimeCopy.tm_mon + 1, localTimeCopy.tm_mday);
String currentTime = String(timeStr);
String currentDate = String(dateStr);
String currentDay = daysOfWeek[localTimeCopy.tm_wday];

// --- Tối ưu hóa việc vẽ lại ---
bool timeChanged = (currentTime != lastTimeString);
bool dayChanged = (currentDay != lastDayString);
bool dateChanged = (currentDate != lastDateString);
bool angleChanged = (angle != lastSecondAngle);

// Cập nhật các thành phần động nếu có thay đổi
if (angleChanged || timeChanged || dayChanged ) {
     updateDynamicElements(angle, currentDay, currentTime, currentDate);
}
if (dateChanged) {
    updateDateDisplay(currentDate);
}

// Vẽ icon WiFi (chỉ khi trạng thái thay đổi hoặc cần vẽ lại)
// (Cần thêm biến lastWifiState để tối ưu)
drawWifiIcon(); // Hàm này sẽ kiểm tra wifiConnectedLocal

// Cập nhật trạng thái cuối cùng
if (timeChanged) lastTimeString = currentTime;
if (dayChanged) lastDayString = currentDay;
if (dateChanged) lastDateString = currentDate;
if (angleChanged) lastSecondAngle = angle;

}
// --- Các hàm vẽ private ---
void DisplayManager::drawStaticUI() {
uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
uint16_t linesColor = (currentTheme == 0) ? DARK_LINES : LIGHT_LINES;
uint16_t boxColor = (currentTheme == 0) ? DARK_BOX : LIGHT_BOX;
uint16_t highlightColor = (currentTheme == 0) ? DARK_HIGHLIGHT : LIGHT_HIGHLIGHT;
tft.fillScreen(bgColor);

tft.setTextDatum(MC_DATUM);
tft.setTextColor(textColor, bgColor);

// Vẽ hộp ngày tháng
tft.fillRect(70, 86, 12, 20, boxColor);
tft.fillRect(84, 86, 12, 20, boxColor);
tft.fillRect(150, 86, 12, 20, boxColor);
tft.fillRect(164, 86, 12, 20, boxColor);

// Vẽ nhãn MONTH/DAY
tft.setTextFont(1); // Font nhỏ
tft.setTextDatum(BC_DATUM);
tft.drawString("MONTH", 84, 84); // Tọa độ y dịch lên chút
tft.drawString("DAY", 162, 84);

// Vẽ các chữ tĩnh khác
tft.setTextDatum(MC_DATUM);
tft.setTextColor(highlightColor, bgColor);
tft.setTextFont(1); // Font nhỏ cho chữ tĩnh
tft.drawString("ESP32 WATCH", CENTER_X, 174); // Thay chữ
tft.setTextFont(4); // Font lớn hơn cho ***
tft.drawString("***", CENTER_X, 108);

// Vẽ các chấm phút
for (int i = 0; i < 60; i++) {
    tft.fillCircle(px[startMinute[i]], py[startMinute[i]], (i % 5 == 0) ? 2 : 1, linesColor); // Chấm to hơn ở 5 phút
}

// Vẽ tam giác chỉ 12h
 tft.fillTriangle(CENTER_X , CENTER_Y - RADIUS + 5, // Đỉnh trên
                  CENTER_X - 5, CENTER_Y - RADIUS + 15, // Góc trái dưới
                  CENTER_X + 5, CENTER_Y - RADIUS + 15, // Góc phải dưới
                  highlightColor);

 // Reset trạng thái để buộc vẽ lại phần động
 lastTimeString = "";
 lastDateString = "";
 lastDayString = "";
 lastSecondAngle = -1;
 refreshDynamicElements(); // Vẽ phần động ngay lập tức

}
void DisplayManager::updateDynamicElements(int angle, const String& currentDay, const String& currentTime, const String& currentDate) {
uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
uint16_t linesColor = (currentTheme == 0) ? DARK_LINES : LIGHT_LINES;
// Xóa và vẽ lại các nhãn giờ/phút xoay
if (angle != lastSecondAngle) {
    tft.setTextFont(2); // Font số 2 cho nhãn
    tft.setTextDatum(MC_DATUM);
    for (int i = 0; i < 12; i++) {
        // Xóa vị trí cũ
        if (lastSecondAngle != -1) {
            int oldPos = (startHour[i] + lastSecondAngle) % 360;
            tft.setTextColor(bgColor, bgColor);
            tft.drawString(hourLabels[i], x[oldPos], y[oldPos]);
            tft.drawLine(px[oldPos], py[oldPos], lx[oldPos], ly[oldPos], bgColor);
        }
        // Vẽ vị trí mới
        int newPos = (startHour[i] + angle) % 360;
        tft.setTextColor(textColor, bgColor);
        tft.drawString(hourLabels[i], x[newPos], y[newPos]);
        tft.drawLine(px[newPos], py[newPos], lx[newPos], ly[newPos], linesColor);
    }
}

// Cập nhật ngày trong tuần (chỉ khi thay đổi)
if (currentDay != lastDayString) {
     tft.setTextFont(2); // Font số 2
     tft.setTextDatum(MC_DATUM);
     // Xóa ngày cũ (vẽ chữ nhật nền)
     tft.fillRect(CENTER_X - 40, CENTER_Y - 8, 80, 16, bgColor);
     // Vẽ ngày mới
     tft.setTextColor(textColor, bgColor);
     tft.drawString(currentDay, CENTER_X, CENTER_Y);
}


// Cập nhật giờ và giây (chỉ khi thay đổi)
if (currentTime != lastTimeString) {
    tft.setTextDatum(MC_DATUM);
    // Xóa giây cũ
    tft.setFreeFont(&DSEG7_Modern_Bold_20); // Đảm bảo font này tồn tại
    tft.setTextColor(bgColor, bgColor);
    tft.drawString(lastTimeString.substring(6, 8), CENTER_X, CENTER_Y - 36);

    // Xóa giờ:phút cũ
    tft.setFreeFont(&DSEG7_Classic_Regular_28);
    tft.drawString(lastTimeString.substring(0, 5), CENTER_X, CENTER_Y + 28);

    // Vẽ giây mới
    tft.setFreeFont(&DSEG7_Modern_Bold_20);
    tft.setTextColor(textColor, bgColor);
    tft.drawString(currentTime.substring(6, 8), CENTER_X, CENTER_Y - 36);

    // Vẽ giờ:phút mới
    tft.setFreeFont(&DSEG7_Classic_Regular_28);
    tft.drawString(currentTime.substring(0, 5), CENTER_X, CENTER_Y + 28);
}
// Đặt lại font mặc định sau khi dùng font free
tft.setTextFont(0);

}
void DisplayManager::updateDateDisplay(const String& currentDate) {
uint16_t boxColor = (currentTheme == 0) ? DARK_BOX : LIGHT_BOX;
uint16_t textColor = (currentTheme == 0) ? DARK_TEXT : LIGHT_TEXT;
// Vẽ lại toàn bộ phần ngày tháng khi có thay đổi
tft.setTextFont(2); // Font số 2
tft.setTextDatum(MC_DATUM);
tft.setTextColor(textColor, boxColor); // Chữ màu text, nền màu hộp

// Tháng (MM)
tft.drawString(currentDate.substring(0, 1), 76, 96); // Số đầu
tft.drawString(currentDate.substring(1, 2), 90, 96); // Số sau

// Ngày (DD)
tft.drawString(currentDate.substring(3, 4), 156, 96); // Số đầu
tft.drawString(currentDate.substring(4, 5), 170, 96); // Số sau

tft.setTextFont(0); // Reset font

}
// Hàm refreshDynamicElements()
void DisplayManager::refreshDynamicElements() {
struct tm localTimeCopy;
bool isTimeValid = false;
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    isTimeValid = timeInitializedLocal;
    if (isTimeValid) {
        localTimeCopy = timeinfoLocal;
    }
    xSemaphoreGive(dataMutex);
}

if (isTimeValid) {
    int angle = localTimeCopy.tm_sec * 6;
    char timeStr[9];
    char dateStr[6];
    sprintf(timeStr, "%02d:%02d:%02d", localTimeCopy.tm_hour, localTimeCopy.tm_min, localTimeCopy.tm_sec);
    sprintf(dateStr, "%02d:%02d", localTimeCopy.tm_mon + 1, localTimeCopy.tm_mday);
    String currentTime = String(timeStr);
    String currentDate = String(dateStr);
    String currentDay = daysOfWeek[localTimeCopy.tm_wday];

    // Buộc vẽ lại bằng cách xóa trạng thái cũ
    lastTimeString = ""; lastDateString = ""; lastDayString = ""; lastSecondAngle = -1;

    updateDynamicElements(angle, currentDay, currentTime, currentDate);
    updateDateDisplay(currentDate);

    // Cập nhật lại trạng thái
    lastTimeString = currentTime; lastDateString = currentDate; lastDayString = currentDay; lastSecondAngle = angle;
}

}
// Hàm toggleScreen() - Tự đảo trạng thái
void DisplayManager::toggleScreen() {
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
screenOn = !screenOn; // Đảo trạng thái
bool localScreenOn = screenOn; // Lấy trạng thái mới
xSemaphoreGive(dataMutex);
// Thực hiện hành động ngoài mutex
     if (localScreenOn) {
         tft.writecommand(TFT_DISPON);
         // Khi bật màn hình, vẽ lại toàn bộ
         drawStaticUI(); // Hàm này đã bao gồm refreshDynamicElements
         Serial.println("Screen turned ON");
     } else {
         tft.writecommand(TFT_DISPOFF); // Tắt hiển thị
         Serial.println("Screen turned OFF");
     }
 } else {
     Serial.println("Timeout taking display data mutex in toggleScreen!");
 }

}
// Hàm toggleTheme()
void DisplayManager::toggleTheme() {
// Chỉ đổi theme nếu màn hình đang bật
if (!isScreenOn()) return;
if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    currentTheme = !currentTheme; // Đảo theme
    int localTheme = currentTheme;
    xSemaphoreGive(dataMutex);

    Serial.printf("Theme changed to: %s\n", localTheme == 0 ? "Dark" : "Light");
    // Vẽ lại toàn bộ giao diện với theme mới
    drawStaticUI(); // Hàm này đã bao gồm refreshDynamicElements
 } else {
     Serial.println("Timeout taking display data mutex in toggleTheme!");
 }

}
// Hàm isScreenOn()
bool DisplayManager::isScreenOn() const {
// Có thể cần mutex nếu đọc/ghi screenOn phức tạp hơn
return screenOn;
}
// Hàm vẽ icon WiFi
void DisplayManager::drawWifiIcon() {
bool isConnected;
static bool lastWifiState = !wifiConnectedLocal; // Khởi tạo khác để vẽ lần đầu
isConnected = wifiConnectedLocal;


if (isConnected != lastWifiState) { // Chỉ vẽ lại nếu trạng thái thay đổi
    uint16_t bgColor = (currentTheme == 0) ? DARK_BACKGROUND : LIGHT_BACKGROUND;
    int iconX = 210; // Vị trí icon X
    int iconY = 10;  // Vị trí icon Y
    int iconW = 25;
    int iconH = 16;

    tft.fillRect(iconX, iconY, iconW, iconH, bgColor); // Xóa vùng icon cũ

    if (isConnected) {
        tft.setTextColor((currentTheme == 0) ? TFT_GREEN : TFT_BLUE, bgColor); // Màu icon
        tft.setTextDatum(TR_DATUM); // Top Right datum
        tft.drawString("WiFi", iconX + iconW -1 , iconY, 2); // Font 2
    }
    lastWifiState = isConnected; // Cập nhật trạng thái đã vẽ
     tft.setTextDatum(MC_DATUM); // Reset datum về mặc định
}

}