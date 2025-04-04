#ifndef HEART_RATE_SPO2_H
#define HEART_RATE_SPO2_H

#include <MAX30105.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class HeartRateSpO2 {
public:
    HeartRateSpO2();
    void begin();
    void startTask();
    void stopTask();
    void getData(int& heartRate, int& spo2, long& irValue, long& redValue);

private:
    MAX30105 particleSensor;
    TaskHandle_t taskHandle;
    static void taskFunction(void* pvParameters);
    void updateSensor();
    float lowPassFilter(float input, float previous, float alpha);
    float calculateSpO2(long redValue, long irValue);
    byte rates[4];           // Mảng lưu nhịp tim (từ example)
    byte rateSpot;           // Vị trí trong mảng (từ example)
    unsigned long lastBeat;  // Thời điểm nhịp cuối (từ example)
    int heartRateLocal;      // Nhịp tim trung bình
    int spo2Local;           // SpO2
    long irValueLocal;       // Giá trị IR
    long redValueLocal;      // Giá trị Red
    SemaphoreHandle_t dataMutex;
};

#endif