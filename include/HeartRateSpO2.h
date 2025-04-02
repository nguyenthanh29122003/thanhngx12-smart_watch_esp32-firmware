#ifndef HEART_RATE_SPO2_H
#define HEART_RATE_SPO2_H

#include <MAX30105.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h> // Thêm include

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
    bool sensorActive;
    static void taskFunction(void* pvParameters);
    void updateSensor();
    float lowPassFilter(float input, float previous, float alpha);
    bool detectMotion(long irValue);
    float calculateSpO2(long redValue, long irValue);
    int16_t rates[4];
    byte rateSpot;
    unsigned long lastBeat;
    int heartRateLocal;
    int spo2Local;
    long irValueLocal;
    long redValueLocal;
    float irFiltered;
    SemaphoreHandle_t dataMutex;
};

#endif