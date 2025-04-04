#ifndef STEP_COUNTER_H
#define STEP_COUNTER_H

#include <MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class StepCounter {
public:
    StepCounter();
    void begin();
    void startTask();
    void stopTask();
    void updateSensor();
    void getData(int& stepCount, float& distance, float& ax, float& ay, float& az, 
                 float& gx, float& gy, float& gz);

private:
    MPU6050 mpu;
    TaskHandle_t taskHandle;
    bool stepDetected;
    bool sensorActive;
    unsigned long lastStepTime;
    int stepCountLocal;
    float distanceLocal;
    float axLocal, ayLocal, azLocal;
    float gxLocal, gyLocal, gzLocal;
    float accFiltered, gyroFiltered;
    SemaphoreHandle_t dataMutex;
    static void taskFunction(void* pvParameters);
    float lowPassFilter(float input, float previous, float alpha);
    bool detectStep(float accMagnitude, float gyroMagnitude);
};

#endif