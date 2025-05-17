#include "StepCounter.h"
#include "Config.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include <cmath>

// Khai báo Mutex I2C toàn cục (được định nghĩa trong main.cpp)
extern SemaphoreHandle_t i2cMutex;

// --- Chân I2C cho MPU6050 (Theo tài liệu board của bạn) ---
#ifndef MPU6050_SDA_PIN
#define MPU6050_SDA_PIN 42
#endif
#ifndef MPU6050_SCL_PIN
#define MPU6050_SCL_PIN 41
#endif

// --- EEPROM ---
#ifndef EEPROM_SIZE
#define EEPROM_SIZE 512
#endif
#ifndef STEP_COUNT_ADDR
#define STEP_COUNT_ADDR 0
#endif

// --- Constructor ---
StepCounter::StepCounter()
    : mpu(), // Khởi tạo đối tượng MPU6050
      taskHandle(NULL), dataMutex(NULL),
      sensorReady(false), stepDetected(false), lastStepTime(0),
      stepCountLocal(0), distanceLocal(0.0f), lastSavedStepCount(0),
      axLocal(0.0f), ayLocal(0.0f), azLocal(0.0f),
      gxLocal(0.0f), gyLocal(0.0f), gzLocal(0.0f)
{
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("CRITICAL: Failed to create StepCounter data mutex!");
    }
}

// --- Hàm begin() - Khởi tạo MPU6050 và EEPROM ---
bool StepCounter::begin() {
    sensorReady = false;

    // Khởi tạo EEPROM
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Warning: Failed to initialize EEPROM for StepCounter.");
    }

    // Lấy Mutex I2C để khởi tạo cảm biến an toàn
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS * 2)) == pdTRUE) {
        Serial.println("Initializing MPU6050...");

        // Khởi tạo MPU6050
        if (!mpu.begin(MPU6050_ADDRESS, &Wire)) {
            Serial.println("!!! ERROR: Failed to find/init MPU6050!");
            Serial.println("!!! Check wiring, I2C address, SDA/SCL pins.");
            Serial.println("!!! Step counting disabled.");
        } else {
            Serial.println("MPU6050 Found!");

            // Cấu hình Accelerometer và Gyroscope
            Serial.println("Configuring MPU6050 (Acc Range: ±4G, Gyro Range: ±250dps)...");
            mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // Dải ±4g
            mpu.setGyroRange(MPU6050_RANGE_250_DEG);      // Dải ±250dps
            mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Bộ lọc băng thông 21Hz

            sensorReady = true;
            Serial.println("MPU6050 Configured and Enabled.");
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("CRITICAL: Timeout waiting for I2C mutex during MPU6050 init!");
    }

    // Khôi phục bước chân từ EEPROM
    if (EEPROM.length() > 0) {
        stepCountLocal = EEPROM.readInt(STEP_COUNT_ADDR);
        if (stepCountLocal < 0 || stepCountLocal > 1000000) {
            Serial.printf("Warning: Invalid step count read from EEPROM (%d), resetting to 0.\n", stepCountLocal);
            stepCountLocal = 0;
        }
        lastSavedStepCount = stepCountLocal;
        distanceLocal = (float)stepCountLocal * STEP_LENGTH;
        Serial.printf("Restored step count from EEPROM: %d\n", stepCountLocal);
    } else {
        Serial.println("EEPROM not initialized, starting step count from 0.");
        stepCountLocal = 0;
        lastSavedStepCount = 0;
        distanceLocal = 0.0f;
    }

    return sensorReady;
}

// --- startTask ---
void StepCounter::startTask(UBaseType_t priority) {
    if (!sensorReady) {
        Serial.println("MPU6050 not ready, skipping StepCounter Task creation.");
        return;
    }
    xTaskCreate(taskFunction, "StepCounterTask", 5120, this, priority, &taskHandle);
    if (taskHandle == NULL) {
        Serial.println("CRITICAL: Error creating StepCounter Task!");
    } else {
        Serial.println("StepCounter Task started.");
    }
}

// --- stopTask ---
void StepCounter::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle;
        taskHandle = NULL;
        vTaskDelete(tempHandle);
        Serial.println("StepCounter task stopped.");
    }
}

// --- taskFunction ---
void StepCounter::taskFunction(void* pvParameters) {
    StepCounter* instance = static_cast<StepCounter*>(pvParameters);
    if (instance == nullptr) {
        Serial.println("CRITICAL: StepCounter task received null parameter!");
        vTaskDelete(NULL);
        return;
    }
    Serial.println("StepCounter Task running...");
    while (true) {
        instance->updateSensor();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- updateSensor ---
void StepCounter::updateSensor() {
    if (!sensorReady) {
        static int beginRetryCount = 0;
        if (beginRetryCount < 5) {
            Serial.println("StepCounter: Sensor not ready, attempting to re-initialize...");
            beginRetryCount++;
            begin();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        return;
    }

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    bool readSuccess = false;
    const int MAX_READ_ATTEMPTS = 3;

    // Lấy Mutex I2C
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; ++attempt) {
            sensors_event_t a, g, temp;
            if (mpu.getEvent(&a, &g, &temp)) {
                ax = a.acceleration.x / 9.81; // Chuyển m/s² sang g
                ay = a.acceleration.y / 9.81;
                az = a.acceleration.z / 9.81;
                gx = g.gyro.x * 57.2958;      // Chuyển rad/s sang deg/s
                gy = g.gyro.y * 57.2958;
                gz = g.gyro.z * 57.2958;

                if (!isnan(ax) && !isnan(ay) && !isnan(az) && !isnan(gx) && !isnan(gy) && !isnan(gz)) {
                    readSuccess = true;
                    break;
                } else {
                    Serial.println("Warning: MPU6050 read returned NAN values.");
                }
            } else {
                Serial.printf("Warning: MPU6050 read failed (attempt %d).\n", attempt + 1);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("Timeout waiting for I2C mutex in StepCounter updateSensor!");
        return;
    }

    if (!readSuccess) {
        Serial.println("ERROR: MPU6050 read failed after multiple attempts.");
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            axLocal = ayLocal = azLocal = gxLocal = gyLocal = gzLocal = 0.0f;
            xSemaphoreGive(dataMutex);
        }
        return;
    }

    // Tính độ lớn vector
    float accMagnitude = sqrt(ax * ax + ay * ay + az * az);
    float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

    // Logic đếm bước
    if (detectStep(accMagnitude, gyroMagnitude)) {
        int currentStepCount = 0;
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            stepCountLocal++;
            distanceLocal = (float)stepCountLocal * STEP_LENGTH;
            currentStepCount = stepCountLocal;
            lastStepTime = millis();
            stepDetected = true;

            if (currentStepCount > 0 && (currentStepCount - lastSavedStepCount >= SAVE_STEP_INTERVAL)) {
                if (EEPROM.readInt(STEP_COUNT_ADDR) != currentStepCount) {
                    Serial.printf("Saving steps: %d (Last saved: %d)\n", currentStepCount, lastSavedStepCount);
                    EEPROM.writeInt(STEP_COUNT_ADDR, currentStepCount);
                    if (EEPROM.commit()) {
                        Serial.printf("Step count saved to EEPROM: %d\n", currentStepCount);
                        lastSavedStepCount = currentStepCount;
                    } else {
                        Serial.println("EEPROM commit failed!");
                    }
                } else {
                    lastSavedStepCount = currentStepCount;
                }
            }
            xSemaphoreGive(dataMutex);
        } else {
            Serial.println("CRITICAL: Failed to take StepCounter dataMutex!");
        }
    }

    if (accMagnitude < STEP_RESET_THRESHOLD) {
        stepDetected = false;
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        axLocal = ax; ayLocal = ay; azLocal = az;
        gxLocal = gx; gyLocal = gy; gzLocal = gz;
        xSemaphoreGive(dataMutex);
    } else {
        Serial.println("CRITICAL: Failed to take StepCounter dataMutex for local update!");
    }

    static unsigned long lastIMUDebugTime = 0;
    if (millis() - lastIMUDebugTime > 5000) {
        Serial.printf("MPU6050 - Acc(g): %.2f,%.2f,%.2f | Gyr(dps): %.2f,%.2f,%.2f | Steps: %d\n",
                      axLocal, ayLocal, azLocal, gxLocal, gyLocal, gzLocal, stepCountLocal);
        lastIMUDebugTime = millis();
    }
}

// --- lowPassFilter ---
float StepCounter::lowPassFilter(float input, float previous, float alpha) {
    alpha = constrain(alpha, 0.0f, 1.0f);
    if (isnan(previous)) {
        return input;
    }
    return alpha * previous + (1.0f - alpha) * input;
}

// --- detectStep ---
bool StepCounter::detectStep(float accMagnitude, float gyroMagnitude) {
    if (!stepDetected &&
        accMagnitude > THRESHOLD &&
        gyroMagnitude < STEP_DETECT_MAX_GYRO &&
        (millis() - lastStepTime > STEP_DELAY)) {
        return true;
    }
    return false;
}

// --- getData ---
void StepCounter::getData(int& stepCount, float& distance, float& ax, float& ay, float& az,
                          float& gx, float& gy, float& gz) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        stepCount = stepCountLocal; distance = distanceLocal;
        ax = axLocal; ay = ayLocal; az = azLocal;
        gx = gxLocal; gy = gyLocal; gz = gzLocal;
        xSemaphoreGive(dataMutex);
    } else {
        stepCount = -1;
        distance = 0.0f;
        ax = ay = az = gx = gy = gz = 0.0f;
        Serial.println("CRITICAL: Failed to take dataMutex in getData!");
    }
}

// --- resetSteps ---
void StepCounter::resetSteps() {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        stepCountLocal = 0; distanceLocal = 0.0f; lastSavedStepCount = 0;
        Serial.println("StepCounter local data reset by resetSteps().");
        xSemaphoreGive(dataMutex);
        EEPROM.writeInt(STEP_COUNT_ADDR, 0);
        if (EEPROM.commit()) Serial.println("Step count reset to 0 in EEPROM (from resetSteps).");
        else Serial.println("Failed to commit EEPROM during step reset!");
    } else {
        Serial.println("CRITICAL: Failed to take dataMutex in resetSteps!");
    }
}