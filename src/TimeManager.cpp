#include "TimeManager.h"
#include "Config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <esp_sntp.h>

TimeManager::TimeManager() 
    : taskHandle(NULL), timeInitialized(false), lastSyncTime(0) {
    memset(&timeinfoLocal, 0, sizeof(timeinfoLocal));
    timeMutex = xSemaphoreCreateMutex();
}

void TimeManager::begin() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    syncNTP(); // Thử đồng bộ lần đầu
    Serial.println("TimeManager initialized");
}

void TimeManager::startTask() {
    xTaskCreate(
        taskFunction, "TimeTask", 1536, this, TASK_PRIORITY_TIME, &taskHandle
    );
}

void TimeManager::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        Serial.println("Time task stopped");
    }
}

void TimeManager::taskFunction(void* pvParameters) {
    TimeManager* instance = static_cast<TimeManager*>(pvParameters);
    while (true) {
        instance->updateTimeLocal();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TimeManager::updateTimeLocal() {
    if (!timeInitialized) {
        if (syncNTP()) {
            if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE) {
                timeInitialized = true;
                lastSyncTime = millis();
                xSemaphoreGive(timeMutex);
            }
        }
    } else {
        if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE) {
            timeinfoLocal.tm_sec++;
            if (timeinfoLocal.tm_sec >= 60) {
                timeinfoLocal.tm_sec = 0;
                timeinfoLocal.tm_min++;
                if (timeinfoLocal.tm_min >= 60) {
                    timeinfoLocal.tm_min = 0;
                    timeinfoLocal.tm_hour++;
                    if (timeinfoLocal.tm_hour >= 24) {
                        timeinfoLocal.tm_hour = 0;
                        timeinfoLocal.tm_mday++;
                        if (timeinfoLocal.tm_mday > daysInMonth(timeinfoLocal.tm_mon + 1, timeinfoLocal.tm_year + 1900)) {
                            timeinfoLocal.tm_mday = 1;
                            timeinfoLocal.tm_mon++;
                            if (timeinfoLocal.tm_mon >= 12) {
                                timeinfoLocal.tm_mon = 0;
                                timeinfoLocal.tm_year++;
                            }
                        }
                    }
                }
            }

            // Đồng bộ NTP mỗi 1 giờ nếu WiFi khả dụng và chưa nhận thời gian BLE
            if (millis() - lastSyncTime >= 3600000 && WiFi.status() == WL_CONNECTED) {
                if (syncNTP()) {
                    lastSyncTime = millis();
                }
            }
            xSemaphoreGive(timeMutex);
        }
    }
}

bool TimeManager::syncNTP() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    struct tm tempTime;
    if (!getLocalTime(&tempTime)) {
        return false;
    }

    if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE) {
        timeinfoLocal = tempTime;
        xSemaphoreGive(timeMutex);
    }
    return true;
}

int TimeManager::daysInMonth(int month, int year) {
    if (month == 2) {
        return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29 : 28;
    }
    return (month == 4 || month == 6 || month == 9 || month == 11) ? 30 : 31;
}

void TimeManager::getTime(struct tm& timeinfo, bool& initialized) {
    if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE) {
        timeinfo = timeinfoLocal;
        initialized = timeInitialized;
        xSemaphoreGive(timeMutex);
    }
}

void TimeManager::setTimeFromBLE(const struct tm& timeinfo) {
    if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE) {
        timeinfoLocal = timeinfo;
        timeInitialized = true;
        lastSyncTime = millis();
        Serial.printf("Time set from BLE: %04d-%02d-%02d %02d:%02d:%02d\n",
                      timeinfoLocal.tm_year + 1900, timeinfoLocal.tm_mon + 1, timeinfoLocal.tm_mday,
                      timeinfoLocal.tm_hour, timeinfoLocal.tm_min, timeinfoLocal.tm_sec);
        xSemaphoreGive(timeMutex);
    }
}