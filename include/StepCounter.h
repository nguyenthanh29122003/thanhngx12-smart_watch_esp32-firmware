#ifndef STEP_COUNTER_H
#define STEP_COUNTER_H

#include <MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h> // Thêm include

class StepCounter {
public:
    StepCounter();
    void begin();
    void startTask();
    void stopTask();
    void getData(int& stepCount, float& distance, float& ax, float& ay, float& az);

private:
    MPU6050 mpu;
    TaskHandle_t taskHandle;
    bool stepDetected;
    bool sensorActive;
    unsigned long lastStepTime;
    static void taskFunction(void* pvParameters);
    void updateSensor();
    float lowPassFilter(float input, float previous, float alpha);
    bool detectStep(float accMagnitude, float gyroMagnitude);
    int stepCountLocal;
    float distanceLocal;
    float axLocal, ayLocal, azLocal;
    float accFiltered, gyroFiltered;
    SemaphoreHandle_t dataMutex;
};

#endif