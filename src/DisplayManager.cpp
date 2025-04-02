// #include "DisplayManager.h"
// #include "Config.h"

// DisplayManager::DisplayManager() 
//     : tft(), taskHandle(NULL), screenOn(true), displayActive(false),
//       wifiConnectedLocal(false), stepCountLocal(0), distanceLocal(0), 
//       heartRateLocal(0), spo2Local(-999), timeInitializedLocal(false) {
//     memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
//     dataMutex = xSemaphoreCreateMutex();
// }

// void DisplayManager::begin() {
//     tft.init();
//     tft.setRotation(0);
//     tft.fillScreen(TFT_BLACK);
//     tft.setTextColor(TFT_WHITE, TFT_BLACK);
//     tft.setTextSize(1); // Giảm size font để tối ưu
//     displayActive = true;
//     Serial.println("Display initialized");
// }

// void DisplayManager::startTask() {
//     xTaskCreate(
//         taskFunction,       // Hàm task
//         "DisplayTask",      // Tên task
//         2048,               // Stack size (giữ 2048 vì vẽ đồ họa cần bộ nhớ)
//         this,               // Tham số
//         1,                  // Độ ưu tiên
//         &taskHandle         // Handle
//     );
// }

// void DisplayManager::stopTask() {
//     if (taskHandle != NULL) {
//         vTaskDelete(taskHandle);
//         taskHandle = NULL;
//         tft.writecommand(TFT_DISPOFF); // Tắt màn hình
//         displayActive = false;
//         Serial.println("Display task stopped");
//     }
// }

// void DisplayManager::taskFunction(void* pvParameters) {
//     DisplayManager* instance = static_cast<DisplayManager*>(pvParameters);
//     while (true) {
//         instance->updateDisplay();
//         vTaskDelay((instance->screenOn ? 1000 : 5000) / portTICK_PERIOD_MS); // 1Hz khi bật, 0.2Hz khi tắt
//     }
// }

// void DisplayManager::updateData(bool wifiConnected, int stepCount, float distance, 
//                                int heartRate, int spo2, struct tm* timeinfo, bool timeInitialized) {
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//         wifiConnectedLocal = wifiConnected;
//         stepCountLocal = stepCount;
//         distanceLocal = distance;
//         heartRateLocal = heartRate;
//         spo2Local = spo2;
//         timeinfoLocal = *timeinfo;
//         timeInitializedLocal = timeInitialized;
//         xSemaphoreGive(dataMutex);
//     }
// }

// void DisplayManager::updateDisplay() {
//     if (!screenOn || !displayActive) {
//         if (displayActive) {
//             tft.writecommand(TFT_DISPOFF); // Tắt màn hình
//             displayActive = false;
//         }
//         return;
//     }

//     if (!displayActive) {
//         tft.writecommand(TFT_DISPON); // Bật màn hình
//         displayActive = true;
//     }

//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//         // Chỉ cập nhật vùng thay đổi
//         char buffer[20];

//         // Thời gian
//         if (timeInitializedLocal) {
//             sprintf(buffer, "%02d:%02d", timeinfoLocal.tm_hour, timeinfoLocal.tm_min);
//             partialUpdate(10, 10, 80, 32, buffer, TFT_WHITE);
//             sprintf(buffer, "%02d-%02d-%04d", timeinfoLocal.tm_mday, timeinfoLocal.tm_mon + 1, timeinfoLocal.tm_year + 1900);
//             partialUpdate(10, 40, 100, 16, buffer, TFT_WHITE);
//         } else {
//             partialUpdate(10, 10, 100, 32, "No time", TFT_RED);
//         }

//         // Bước chân
//         sprintf(buffer, "Steps: %d", stepCountLocal);
//         partialUpdate(10, 60, 100, 16, buffer, TFT_WHITE);

//         // Khoảng cách
//         sprintf(buffer, "Dist: %.1f m", distanceLocal);
//         partialUpdate(10, 80, 100, 16, buffer, TFT_WHITE);

//         // Nhịp tim
//         sprintf(buffer, "HR: %d BPM", heartRateLocal > 0 ? heartRateLocal : 0);
//         partialUpdate(10, 100, 100, 16, buffer, TFT_WHITE);

//         // SpO2
//         sprintf(buffer, "SpO2: %d %%", spo2Local > 0 ? spo2Local : 0);
//         partialUpdate(10, 120, 100, 16, buffer, TFT_WHITE);

//         // WiFi icon
//         drawIcons(wifiConnectedLocal);

//         xSemaphoreGive(dataMutex);
//     }
// }

// void DisplayManager::toggleScreen(bool screenOn) {
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//         this->screenOn = screenOn;
//         if (!screenOn && displayActive) {
//             tft.writecommand(TFT_DISPOFF); // Tắt màn hình
//             displayActive = false;
//         } else if (screenOn && !displayActive) {
//             tft.writecommand(TFT_DISPON); // Bật màn hình
//             displayActive = true;
//             tft.fillScreen(TFT_BLACK); // Xóa màn hình khi bật lại
//         }
//         xSemaphoreGive(dataMutex);
//     }
// }

// void DisplayManager::partialUpdate(int x, int y, int w, int h, const char* text, uint16_t color) {
//     tft.setTextColor(color, TFT_BLACK);
//     tft.fillRect(x, y, w, h, TFT_BLACK); // Xóa vùng cũ
//     tft.setCursor(x, y);
//     tft.print(text);
// }

// void DisplayManager::drawIcons(bool wifiConnected) {
//     if (wifiConnected) {
//         tft.setTextColor(TFT_GREEN, TFT_BLACK);
//         tft.setCursor(200, 10);
//         tft.print("WiFi");
//     } else {
//         tft.fillRect(200, 10, 40, 16, TFT_BLACK); // Xóa icon nếu không kết nối
//     }
// }


#include "DisplayManager.h"
#include "Config.h"

DisplayManager::DisplayManager() 
    : tft(), taskHandle(NULL), screenOn(true), displayActive(false), displayMode(0),
      wifiConnectedLocal(false), stepCountLocal(0), distanceLocal(0), 
      heartRateLocal(0), spo2Local(-999), timeInitializedLocal(false) {
    memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
    dataMutex = xSemaphoreCreateMutex();
}

void DisplayManager::begin() {
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    displayActive = true;
    Serial.println("Display initialized");
}

void DisplayManager::startTask() {
    xTaskCreate(
        taskFunction,       // Hàm task
        "DisplayTask",      // Tên task
        3072,               // Stack size
        this,               // Tham số
        1,                  // Độ ưu tiên
        &taskHandle         // Handle
    );
}

void DisplayManager::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        tft.writecommand(TFT_DISPOFF);
        displayActive = false;
        Serial.println("Display task stopped");
    }
}

void DisplayManager::taskFunction(void* pvParameters) {
    DisplayManager* instance = static_cast<DisplayManager*>(pvParameters);
    while (true) {
        instance->updateDisplay();
        vTaskDelay((instance->screenOn ? 2000 : 10000) / portTICK_PERIOD_MS); // 2s/10s
    }
}

// void DisplayManager::taskFunction(void* pvParameters) {
//     DisplayManager* instance = static_cast<DisplayManager*>(pvParameters);
//     while (true) {
//         instance->updateDisplay();
//         vTaskDelay((instance->screenOn ? 1000 : 5000) / portTICK_PERIOD_MS);
//     }
// }

void DisplayManager::updateData(bool wifiConnected, int stepCount, float distance, 
                               int heartRate, int spo2, struct tm* timeinfo, bool timeInitialized) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        wifiConnectedLocal = wifiConnected;
        stepCountLocal = stepCount;
        distanceLocal = distance;
        heartRateLocal = heartRate;
        spo2Local = spo2;
        timeinfoLocal = *timeinfo;
        timeInitializedLocal = timeInitialized;
        xSemaphoreGive(dataMutex);
    }
}

void DisplayManager::updateDisplay() {
    if (!screenOn || !displayActive) {
        if (displayActive) {
            tft.writecommand(TFT_DISPOFF);
            displayActive = false;
        }
        return;
    }

    if (!displayActive) {
        tft.writecommand(TFT_DISPON);
        displayActive = true;
        tft.fillScreen(TFT_BLACK);
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        char buffer[20];
        tft.fillScreen(TFT_BLACK); // Xóa màn hình trước khi vẽ chế độ mới

        switch (displayMode) {
            case 0: // Steps mode
                sprintf(buffer, "Steps: %d", stepCountLocal);
                partialUpdate(10, 60, 100, 16, buffer, TFT_WHITE);
                sprintf(buffer, "Dist: %.1f m", distanceLocal);
                partialUpdate(10, 80, 100, 16, buffer, TFT_WHITE);
                break;

            case 1: // HeartRate mode
                sprintf(buffer, "HR: %d BPM", heartRateLocal > 0 ? heartRateLocal : 0);
                partialUpdate(10, 100, 100, 16, buffer, TFT_WHITE);
                sprintf(buffer, "SpO2: %d %%", spo2Local > 0 ? spo2Local : 0);
                partialUpdate(10, 120, 100, 16, buffer, TFT_WHITE);
                break;

            case 2: // Time mode
                if (timeInitializedLocal) {
                    sprintf(buffer, "%02d:%02d", timeinfoLocal.tm_hour, timeinfoLocal.tm_min);
                    partialUpdate(10, 10, 80, 32, buffer, TFT_WHITE);
                    sprintf(buffer, "%02d-%02d-%04d", timeinfoLocal.tm_mday, timeinfoLocal.tm_mon + 1, timeinfoLocal.tm_year + 1900);
                    partialUpdate(10, 40, 100, 16, buffer, TFT_WHITE);
                } else {
                    partialUpdate(10, 10, 100, 32, "No time", TFT_RED);
                }
                break;
        }

        drawIcons(wifiConnectedLocal);
        xSemaphoreGive(dataMutex);
    }
}

void DisplayManager::toggleScreen(bool screenOn) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        this->screenOn = screenOn;
        if (!screenOn && displayActive) {
            tft.writecommand(TFT_DISPOFF);
            displayActive = false;
        } else if (screenOn && !displayActive) {
            tft.writecommand(TFT_DISPON);
            displayActive = true;
            tft.fillScreen(TFT_BLACK);
        }
        xSemaphoreGive(dataMutex);
    }
}

void DisplayManager::switchDisplayMode() {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        displayMode = (displayMode + 1) % 3; // Chuyển giữa 0, 1, 2
        Serial.print("Switched to display mode: ");
        Serial.println(displayMode);
        xSemaphoreGive(dataMutex);
    }
}

void DisplayManager::partialUpdate(int x, int y, int w, int h, const char* text, uint16_t color) {
    tft.setTextColor(color, TFT_BLACK);
    tft.fillRect(x, y, w, h, TFT_BLACK);
    tft.setCursor(x, y);
    tft.print(text);
}

void DisplayManager::drawIcons(bool wifiConnected) {
    if (wifiConnected) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(200, 10);
        tft.print("WiFi");
    } else {
        tft.fillRect(200, 10, 40, 16, TFT_BLACK);
    }
}