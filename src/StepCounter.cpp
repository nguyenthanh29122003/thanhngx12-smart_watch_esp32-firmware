#include "StepCounter.h"
#include "Config.h"
#include <EEPROM.h>

#define EEPROM_SIZE 512
#define STEP_COUNT_ADDR 0 // Địa chỉ lưu stepCount trong EEPROM

StepCounter::StepCounter() 
    : mpu(), taskHandle(NULL), stepDetected(false), sensorActive(false), lastStepTime(0),
      stepCountLocal(0), distanceLocal(0), axLocal(0), ayLocal(0), azLocal(0),
      accFiltered(0), gyroFiltered(0), gxLocal(0), gyLocal(0), gzLocal(0) {
    dataMutex = xSemaphoreCreateMutex();
}

void StepCounter::begin() {
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialize EEPROM");
        return;
    }

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
    } else {
        Serial.println("MPU6050 initialized");
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        mpu.setSleepEnabled(true);
        sensorActive = false;
    }

    // Khôi phục stepCount từ EEPROM khi khởi động
    stepCountLocal = EEPROM.readInt(STEP_COUNT_ADDR);
    if (stepCountLocal < 0 || stepCountLocal > 100000) { // Giới hạn hợp lý
        stepCountLocal = 0; // Reset nếu giá trị không hợp lệ
    }
    distanceLocal = stepCountLocal * STEP_LENGTH;
    Serial.printf("Restored step count: %d\n", stepCountLocal);
}

void StepCounter::startTask() {
    xTaskCreate(
        taskFunction,       // Hàm task
        "StepCounterTask",  // Tên task
        4096,               // Stack size
        this,               // Tham số
        1,                  // Độ ưu tiên
        &taskHandle         // Handle
    );
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
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 not responding, skipping update");
        return;
    }

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

    if (ax_raw == 0 && ay_raw == 0 && az_raw == 0 && gx_raw == 0 && gy_raw == 0 && gz_raw == 0) {
        Serial.println("MPU6050 returned all zeros, possible I2C error");
        return;
    }

    float ax = ax_raw / 16384.0; // Chuyển sang g
    float ay = ay_raw / 16384.0;
    float az = az_raw / 16384.0;
    float gx = gx_raw / 131.0;   // Chuyển sang độ/giây
    float gy = gy_raw / 131.0;
    float gz = gz_raw / 131.0;

    float accMagnitude = sqrt(ax * ax + ay * ay + az * az);
    float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

    accFiltered = lowPassFilter(accMagnitude, accFiltered, 0.9);
    gyroFiltered = lowPassFilter(gyroMagnitude, gyroFiltered, 0.9);

    if (accFiltered > 0.5 || gyroFiltered > 20) {
        if (!sensorActive) {
            mpu.setSleepEnabled(false);
            sensorActive = true;
            accFiltered = accMagnitude;
            gyroFiltered = gyroMagnitude;
        }

        if (detectStep(accFiltered, gyroFiltered)) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                stepCountLocal++;
                distanceLocal = stepCountLocal * STEP_LENGTH;
                lastStepTime = millis();
                stepDetected = true;
                // Lưu vào EEPROM mỗi khi bước chân tăng
                EEPROM.writeInt(STEP_COUNT_ADDR, stepCountLocal);
                EEPROM.commit();
                xSemaphoreGive(dataMutex);
            }
        }
        if (accFiltered < THRESHOLD - 0.5) {
            stepDetected = false;
        }
    } else {
        if (sensorActive) {
            mpu.setSleepEnabled(true);
            sensorActive = false;
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
    if (!stepDetected && accMagnitude > THRESHOLD && gyroMagnitude < 100) {
        if (millis() - lastStepTime > 200) {
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