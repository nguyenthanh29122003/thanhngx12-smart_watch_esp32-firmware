#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h> // Thêm include cho semaphore

class TimeManager {
public:
    TimeManager();
    void begin();
    void startTask();  // Khởi động task
    void stopTask();   // Dừng task
    void getTime(struct tm& timeinfo, bool& timeInitialized); // Lấy thời gian an toàn

private:
    TaskHandle_t taskHandle;
    bool timeInitialized;
    struct tm timeinfoLocal;
    unsigned long lastSyncTime;
    static void taskFunction(void* pvParameters); // Hàm task
    void updateTimeLocal(); // Cập nhật thời gian cục bộ
    bool syncNTP(); // Đồng bộ với NTP
    int daysInMonth(int month, int year);
    SemaphoreHandle_t timeMutex; // Sử dụng SemaphoreHandle_t với include đúng
};

#endif