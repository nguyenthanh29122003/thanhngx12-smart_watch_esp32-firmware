// src/StepCounter.cpp
#include "StepCounter.h"
#include "Config.h"
#include <EEPROM.h>

extern SemaphoreHandle_t i2cMutex;

#define EEPROM_SIZE 512
#define STEP_COUNT_ADDR 0

StepCounter::StepCounter() 
    : mpu(), taskHandle(NULL), stepDetected(false), lastStepTime(0),
      stepCountLocal(0), distanceLocal(0), axLocal(0), ayLocal(0), azLocal(0),
      accFiltered(0), gyroFiltered(0), gxLocal(0), gyLocal(0), gzLocal(0) {
    dataMutex = xSemaphoreCreateMutex();
}

void StepCounter::begin() {
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialize EEPROM");
        return;
    }

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        Serial.println("Initializing MPU6050...");
        mpu.initialize();
        if (!mpu.testConnection()) {
            Serial.println("MPU6050 connection failed");
        } else {
            Serial.println("MPU6050 initialized");
            mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
            mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("I2C mutex timeout in StepCounter begin");
    }

    stepCountLocal = EEPROM.readInt(STEP_COUNT_ADDR);
    if (stepCountLocal < 0 || stepCountLocal > 100000) {
        stepCountLocal = 0;
    }
    distanceLocal = stepCountLocal * STEP_LENGTH;
    Serial.printf("Restored step count: %d\n", stepCountLocal);
}

void StepCounter::startTask() {
    xTaskCreate(taskFunction, "StepCounterTask", 4096, this, 1, &taskHandle);
}

void StepCounter::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        Serial.println("StepCounter task stopped");
    }
}

void StepCounter::taskFunction(void* pvParameters) {
    StepCounter* instance = static_cast<StepCounter*>(pvParameters);
    while (true) {
        instance->updateSensor();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void StepCounter::updateSensor() {
    bool connectionValid = false;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        connectionValid = mpu.testConnection();
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("I2C mutex timeout in StepCounter (testConnection)");
        return;
    }

    if (!connectionValid) {
        Serial.println("MPU6050 not responding, attempting reinitialization...");
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            mpu.initialize();
            if (mpu.testConnection()) {
                Serial.println("MPU6050 reinitialized successfully");
                mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
                mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
            } else {
                Serial.println("MPU6050 reinitialization failed");
            }
            xSemaphoreGive(i2cMutex);
        }
        return;
    }

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    bool readSuccess = false;
    for (int attempts = 0; attempts < 3 && !readSuccess; attempts++) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
            mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
            xSemaphoreGive(i2cMutex);
            if (!(ax_raw == 0 && ay_raw == 0 && az_raw == 0 && gx_raw == 0 && gy_raw == 0 && gz_raw == 0)) {
                readSuccess = true;
            } else {
                Serial.println("MPU6050 returned all zeros, retrying...");
            }
        } else {
            Serial.println("I2C mutex timeout in StepCounter (read data)");
            return;
        }
    }

    if (!readSuccess) {
        Serial.println("MPU6050 read failed after retries");
        return;
    }

    float ax = ax_raw / 16384.0;
    float ay = ay_raw / 16384.0;
    float az = az_raw / 16384.0;
    float gx = gx_raw / 131.0;
    float gy = gy_raw / 131.0;
    float gz = gz_raw / 131.0;

    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        lastDebugTime = millis();
    }

    float accMagnitude = sqrt(ax * ax + ay * ay + az * az);
    float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

    // Debug ngưỡng
    static unsigned long lastMagnitudeDebug = 0;
    if (millis() - lastMagnitudeDebug > 1000) {
        lastMagnitudeDebug = millis();
    }

    if (accMagnitude > ACC_THRESHOLD || gyroMagnitude > GYRO_THRESHOLD) {
        accFiltered = lowPassFilter(accMagnitude, accFiltered, 0.9);
        gyroFiltered = lowPassFilter(gyroMagnitude, gyroFiltered, 0.9);

        if (detectStep(accFiltered, gyroFiltered)) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                stepCountLocal++;
                distanceLocal = stepCountLocal * STEP_LENGTH;
                lastStepTime = millis();
                stepDetected = true;

                if (stepCountLocal % SAVE_STEP_INTERVAL == 0) {
                    EEPROM.writeInt(STEP_COUNT_ADDR, stepCountLocal);
                    EEPROM.commit();
                    Serial.printf("Saved step count to EEPROM: %d\n", stepCountLocal);
                }

                Serial.printf("Step detected, total: %d\n", stepCountLocal);
                xSemaphoreGive(dataMutex);
            }
        }
        if (accFiltered < THRESHOLD - 0.5) {
            stepDetected = false;
        }
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        axLocal = ax;
        ayLocal = ay;
        azLocal = az;
        gxLocal = gx;
        gyLocal = gy;
        gzLocal = gz;
        xSemaphoreGive(dataMutex);
    }
}

float StepCounter::lowPassFilter(float input, float previous, float alpha) {
    return alpha * previous + (1 - alpha) * input;
}

bool StepCounter::detectStep(float accMagnitude, float gyroMagnitude) {
    if (!stepDetected && accMagnitude > THRESHOLD && gyroMagnitude < STEP_DETECT_MAX_GYRO) {
        if (millis() - lastStepTime > STEP_DELAY) {
            return true;
        }
    }
    return false;
}

void StepCounter::getData(int& stepCount, float& distance, float& ax, float& ay, float& az, 
                          float& gx, float& gy, float& gz) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        stepCount = stepCountLocal;
        distance = distanceLocal;
        ax = axLocal;
        ay = ayLocal;
        az = azLocal;
        gx = gxLocal;
        gy = gyLocal;
        gz = gzLocal;
        xSemaphoreGive(dataMutex);
    }
}