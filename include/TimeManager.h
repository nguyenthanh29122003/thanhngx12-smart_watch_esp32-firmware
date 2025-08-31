// include/TimeManager.h
#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class TimeManager {
public:
    TimeManager();
    void begin();
    void startTask();
    void stopTask();
    void getTime(struct tm& timeinfo, bool& timeInitialized);
    void setTimeFromBLE(const struct tm& timeinfo); // Thêm hàm nhận thời gian BLE

private:
    TaskHandle_t taskHandle;
    bool timeInitialized;
    struct tm timeinfoLocal;
    unsigned long lastSyncTime;
    static void taskFunction(void* pvParameters);
    void updateTimeLocal();
    bool syncNTP();
    int daysInMonth(int month, int year);
    SemaphoreHandle_t timeMutex;
};

#endif