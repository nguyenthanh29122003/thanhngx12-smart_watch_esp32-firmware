// #include "StepCounter.h"
// #include "Config.h"

// StepCounter::StepCounter() 
//     : mpu(), taskHandle(NULL), stepDetected(false), sensorActive(false), lastStepTime(0),
//       stepCountLocal(0), distanceLocal(0), axLocal(0), ayLocal(0), azLocal(0),
//       accFiltered(0), gyroFiltered(0) {
//     dataMutex = xSemaphoreCreateMutex();
// }

// void StepCounter::begin() {
//     mpu.initialize();
//     if (!mpu.testConnection()) {
//         Serial.println("MPU6050 connection failed! Check wiring or I2C address.");
//         // Không dùng while(1) để tránh treo chương trình
//     } else {
//         mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//         mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//         mpu.setSleepEnabled(true);
//         sensorActive = false;
//         Serial.println("MPU6050 initialized in low-power mode");
//     }
// }

// void StepCounter::startTask() {
//     xTaskCreate(
//         taskFunction,       // Hàm task
//         "StepCounterTask",  // Tên task
//         4096,               // Stack size tối ưu
//         this,               // Tham số
//         1,                  // Độ ưu tiên
//         &taskHandle         // Handle
//     );
// }

// void StepCounter::stopTask() {
//     if (taskHandle != NULL) {
//         vTaskDelete(taskHandle);
//         taskHandle = NULL;
//         mpu.setSleepEnabled(true); // Chuyển sang chế độ ngủ
//         sensorActive = false;
//         Serial.println("StepCounter task stopped");
//     }
// }

// void StepCounter::taskFunction(void* pvParameters) {
//     StepCounter* instance = static_cast<StepCounter*>(pvParameters);
//     while (true) {
//         instance->updateSensor();
//         vTaskDelay((instance->sensorActive ? 100 : 500) / portTICK_PERIOD_MS); // 10Hz khi đo, 2Hz khi chờ
//     }
// }

// void StepCounter::updateSensor() {
//     int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
//     mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
//     mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

//     // Chuyển đổi sang đơn vị thực tế
//     float ax = ax_raw / 16384.0; // ±2g
//     float ay = ay_raw / 16384.0;
//     float az = az_raw / 16384.0;
//     float gx = gx_raw / 131.0;   // ±250°/s
//     float gy = gy_raw / 131.0;
//     float gz = gz_raw / 131.0;

//     // Tính biên độ gia tốc và con quay
//     float accMagnitude = sqrt(ax * ax + ay * ay + az * az);
//     float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

//     // Lọc tín hiệu
//     accFiltered = lowPassFilter(accMagnitude, accFiltered, 0.9);
//     gyroFiltered = lowPassFilter(gyroMagnitude, gyroFiltered, 0.9);

//     // Phát hiện chuyển động
//     if (accFiltered > 0.5 || gyroFiltered > 20) { // Ngưỡng cơ bản để bật cảm biến
//         if (!sensorActive) {
//             mpu.setSleepEnabled(false); // Bật cảm biến
//             sensorActive = true;
//             accFiltered = accMagnitude; // Reset filter
//             gyroFiltered = gyroMagnitude;
//         }

//         // Phát hiện bước chân
//         if (detectStep(accFiltered, gyroFiltered)) {
//             if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//                 stepCountLocal++;
//                 distanceLocal = stepCountLocal * STEP_LENGTH;
//                 lastStepTime = millis();
//                 stepDetected = true;
//                 xSemaphoreGive(dataMutex);
//             }
//         }
//         if (accFiltered < THRESHOLD - 0.5) {
//             stepDetected = false;
//         }
//     } else { // Không có chuyển động
//         if (sensorActive) {
//             mpu.setSleepEnabled(true); // Chuyển sang chế độ ngủ
//             sensorActive = false;
//         }
//     }

//     // Lưu dữ liệu gia tốc
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//         axLocal = ax;
//         ayLocal = ay;
//         azLocal = az;
//         xSemaphoreGive(dataMutex);
//     }
// }

// float StepCounter::lowPassFilter(float input, float previous, float alpha) {
//     return alpha * previous + (1 - alpha) * input; // Bộ lọc Low-pass
// }

// bool StepCounter::detectStep(float accMagnitude, float gyroMagnitude) {
//     // Phát hiện bước dựa trên gia tốc và con quay
//     bool isStep = (accMagnitude > THRESHOLD && gyroMagnitude > 30 && !stepDetected && 
//                    (millis() - lastStepTime > STEP_DELAY));
//     return isStep;
// }

// void StepCounter::getData(int& stepCount, float& distance, float& ax, float& ay, float& az) {
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//         stepCount = stepCountLocal;
//         distance = distanceLocal;
//         ax = axLocal;
//         ay = ayLocal;
//         az = azLocal;
//         xSemaphoreGive(dataMutex);
//     }
// }

#include "StepCounter.h"
#include "Config.h"

StepCounter::StepCounter() 
    : mpu(), taskHandle(NULL), stepDetected(false), sensorActive(false), lastStepTime(0),
      stepCountLocal(0), distanceLocal(0), axLocal(0), ayLocal(0), azLocal(0),
      accFiltered(0), gyroFiltered(0) {
    dataMutex = xSemaphoreCreateMutex();
}

void StepCounter::begin() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed! Check wiring or I2C address.");
    } else {
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        mpu.setSleepEnabled(true);
        sensorActive = false;
        Serial.println("MPU6050 initialized in low-power mode");
    }
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
        mpu.setSleepEnabled(true);
        sensorActive = false;
        Serial.println("StepCounter task stopped");
    }
}

void StepCounter::taskFunction(void* pvParameters) {
    StepCounter* instance = static_cast<StepCounter*>(pvParameters);
    while (true) {
        instance->updateSensor();
        vTaskDelay((instance->sensorActive ? 100 : 500) / portTICK_PERIOD_MS);
    }
}

void StepCounter::updateSensor() {
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 not responding, skipping update");
        return; // Bỏ qua nếu cảm biến không phản hồi
    }

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

    // Kiểm tra xem dữ liệu có hợp lệ không (giả sử nếu tất cả bằng 0 thì lỗi)
    if (ax_raw == 0 && ay_raw == 0 && az_raw == 0 && gx_raw == 0 && gy_raw == 0 && gz_raw == 0) {
        Serial.println("MPU6050 returned all zeros, possible I2C error");
        return;
    }

    float ax = ax_raw / 16384.0;
    float ay = ay_raw / 16384.0;
    float az = az_raw / 16384.0;
    float gx = gx_raw / 131.0;
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
        xSemaphoreGive(dataMutex);
    }
}

float StepCounter::lowPassFilter(float input, float previous, float alpha) {
    return alpha * previous + (1 - alpha) * input;
}

bool StepCounter::detectStep(float accMagnitude, float gyroMagnitude) {
    bool isStep = (accMagnitude > THRESHOLD && gyroMagnitude > 30 && !stepDetected && 
                   (millis() - lastStepTime > STEP_DELAY));
    return isStep;
}

void StepCounter::getData(int& stepCount, float& distance, float& ax, float& ay, float& az) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        stepCount = stepCountLocal;
        distance = distanceLocal;
        ax = axLocal;
        ay = ayLocal;
        az = azLocal;
        xSemaphoreGive(dataMutex);
    }
}