#ifndef STEP_COUNTER_H
#define STEP_COUNTER_H

#include <Adafruit_MPU6050.h> // Thư viện Adafruit MPU6050
#include <Adafruit_Sensor.h>  // Cần cho sự kiện cảm biến
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class StepCounter {
public:
    StepCounter();
    bool begin();
    void startTask(UBaseType_t priority = 1);
    void stopTask();
    void updateSensor();
    void getData(int& stepCount, float& distance, float& ax, float& ay, float& az,
                 float& gx, float& gy, float& gz);
    void resetSteps();

private:
    Adafruit_MPU6050 mpu; // Đối tượng MPU6050

    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex;

    bool sensorReady;
    bool stepDetected;
    unsigned long lastStepTime;
    int stepCountLocal;
    float distanceLocal;
    int lastSavedStepCount;

    float axLocal, ayLocal, azLocal;
    float gxLocal, gyLocal, gzLocal;

    static void taskFunction(void* pvParameters);
    float lowPassFilter(float input, float previous, float alpha);
    bool detectStep(float accMagnitude, float gyroMagnitude);
};

#endif // STEP_COUNTER_H