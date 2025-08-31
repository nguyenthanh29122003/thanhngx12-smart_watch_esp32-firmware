// src/StepCounter.cpp
#include "StepCounter.h"
#include "Config.h"   // Cho các hằng số cấu hình và địa chỉ I2C
#include <EEPROM.h>
#include <Arduino.h>  // Cho millis, Serial, vTaskDelay
#include <Wire.h>     // Cho TwoWire (mặc dù thư viện QMI có thể xử lý Wire.begin)
#include <cmath>      // Cho sqrt và isnan

// Khai báo Mutex I2C toàn cục (được định nghĩa trong main.cpp)
extern SemaphoreHandle_t i2cMutex;

// --- Chân I2C cho QMI8658 (Nếu thư viện không tự quản lý hoặc cần override) ---
// Các chân này nên được truyền vào qmi.begin()
#ifndef QMI8658_SDA_PIN // Đảm bảo định nghĩa nếu chưa có
#define QMI8658_SDA_PIN 42 // Giá trị mặc định từ tài liệu của bạn
#endif
#ifndef QMI8658_SCL_PIN
#define QMI8658_SCL_PIN 41 // Giá trị mặc định từ tài liệu của bạn
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
    : qmi(), // Khởi tạo đối tượng QMI8658C
      taskHandle(NULL), dataMutex(NULL),
      sensorReady(false), stepDetectedThisCycle(false), lastStepTimeMs(0),
      stepCountLocal(0), distanceLocal(0.0f), lastSavedStepCountEEPROM(0),
      axLocal(0.0f), ayLocal(0.0f), azLocal(0.0f),
      gxLocal(0.0f), gyLocal(0.0f), gzLocal(0.0f)
    // accMagnitudeFiltered(0.0f), gyroMagnitudeFiltered(0.0f) // Bỏ qua bộ lọc EMA ban đầu
{
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("CRITICAL: Failed to create StepCounter data mutex!");
    }
}

// --- Hàm begin() - Khởi tạo QMI8658 và EEPROM ---
bool StepCounter::begin() {
    sensorReady = false;

    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Warning: Failed to initialize EEPROM for StepCounter.");
    }

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS * 3)) == pdTRUE) { // Chờ lâu hơn cho init
        Serial.println("Initializing QMI8658...");
        // Khởi tạo Wire cho các chân cụ thể NẾU bus I2C mặc định không phải là 41/42
        // Wire.begin(QMI8658_SDA_PIN, QMI8658_SCL_PIN); // Dòng này có thể cần nếu đây là bus I2C phụ
        // Nếu Wire đã được begin() trong main.cpp cho bus mặc định, và QMI nối vào đó, thì không cần gọi lại.
        // Thư viện QMI của Lewis He nhận đối tượng Wire, SDA, SCL
        if (!qmi.begin(Wire, QMI8658_ADDRESS, QMI8658_SDA_PIN, QMI8658_SCL_PIN)) {
            Serial.println("!!! ERROR: Failed to find/init QMI8658!");
            Serial.println("!!! Check wiring, I2C address, SDA/SCL pins for QMI8658C.");
            Serial.println("!!! Step counting will be disabled.");
        } else {
            Serial.print("QMI8658 Found! Chip ID: 0x"); Serial.println(qmi.getChipID(), HEX);

            Serial.println("Configuring QMI8658 Accelerometer (Range: +/-4G, ODR: 125Hz)...");
            qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz);

            Serial.println("Configuring QMI8658 Gyroscope (Range: +/-256dps, ODR: ~112Hz)...");
            qmi.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS, SensorQMI8658::GYR_ODR_112_1Hz);

            qmi.enableAccelerometer();
            qmi.enableGyroscope();

            // (Tùy chọn) Thực hiện self-test một lần để kiểm tra
            // Serial.println("Performing QMI8658 Accel Self-Test...");
            // if (!qmi.selfTestAccel()) Serial.println("!!! QMI8658 Accel Self-Test FAILED!");
            // else Serial.println("QMI8658 Accel Self-Test PASSED.");

            Serial.println("QMI8658 Configured and Enabled.");
            sensorReady = true;
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("CRITICAL: Timeout waiting for I2C mutex during QMI8658 init!");
    }

    if (EEPROM.length() > 0) {
        stepCountLocal = EEPROM.readInt(STEP_COUNT_ADDR);
        if (stepCountLocal < 0 || stepCountLocal > 1000000) {
            Serial.printf("Warning: Invalid step count from EEPROM (%d), resetting.\n", stepCountLocal);
            stepCountLocal = 0;
        }
        lastSavedStepCountEEPROM = stepCountLocal;
        distanceLocal = (float)stepCountLocal * STEP_LENGTH;
        Serial.printf("Restored step count from EEPROM: %d\n", stepCountLocal);
    } else {
        Serial.println("EEPROM not available, starting steps from 0.");
        stepCountLocal = 0; lastSavedStepCountEEPROM = 0; distanceLocal = 0.0f;
    }
    return sensorReady;
}

// --- startTask ---
void StepCounter::startTask(UBaseType_t priority) {
    if (!sensorReady) {
        Serial.println("QMI8658 not ready, skipping StepCounter Task creation.");
        return;
    }
    xTaskCreate(taskFunction, "StepCounterTask", 5120, this, priority, &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating StepCounter Task!");
    else Serial.println("StepCounter Task started.");
}

// --- stopTask ---
void StepCounter::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle; taskHandle = NULL;
        vTaskDelete(tempHandle);
        Serial.println("StepCounter task stopped.");
        // if (sensorReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        //     qmi.disableAccelerometer(); qmi.disableGyroscope(); // If library supports
        //     xSemaphoreGive(i2cMutex);
        // }
    }
}

// --- taskFunction ---
void StepCounter::taskFunction(void* pvParameters) {
    StepCounter* instance = static_cast<StepCounter*>(pvParameters);
    if (instance == nullptr) { vTaskDelete(NULL); return; }
    Serial.println("StepCounter Task running...");
    while (true) {
        instance->updateSensorData();
        vTaskDelay(pdMS_TO_TICKS(100)); // ~10Hz
    }
}

// --- updateSensorData ---
void StepCounter::updateSensorData() {
    if (!sensorReady) {
        static int beginRetryCount = 0;
        if (beginRetryCount < 3) { // Giảm số lần thử lại
             Serial.println("StepCounter: QMI8658 not ready, attempting re-init...");
             beginRetryCount++;
             if (begin()) { // Gọi lại begin() để thử
                 beginRetryCount = 0; // Reset nếu thành công
             } else {
                 vTaskDelay(pdMS_TO_TICKS(2000)); // Chờ lâu hơn nếu thất bại
             }
        }
        return;
    }

    float ax_raw, ay_raw, az_raw; // Biến tạm để nhận giá trị từ thư viện
    float gx_raw, gy_raw, gz_raw;
    bool readSuccessful = false;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Thư viện QMI8658 của Lewis He trả về giá trị đã được chuyển đổi sang g và dps
        if (qmi.getAccelerometer(ax_raw, ay_raw, az_raw) && qmi.getGyroscope(gx_raw, gy_raw, gz_raw)) {
            // Kiểm tra NAN
            if (!isnan(ax_raw) && !isnan(ay_raw) && !isnan(az_raw) &&
                !isnan(gx_raw) && !isnan(gy_raw) && !isnan(gz_raw)) {
                readSuccessful = true;
            } else {
                Serial.println("Warning: QMI8658 read returned NAN values.");
            }
        } else {
            Serial.println("Warning: QMI8658 read failed (getAccel/Gyro returned false).");
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("Timeout waiting for I2C mutex in StepCounter updateSensorData!");
        return;
    }

    if (!readSuccessful) {
        // Nếu đọc lỗi, không cập nhật giá trị local để tránh gửi dữ liệu sai
        return;
    }

    // --- Dữ liệu đã ở đơn vị chuẩn (g và deg/s) từ thư viện ---
    // !!! NẾU GIÁ TRỊ VẪN SAI (VÍ DỤ AZ = 4.0G), VẤN ĐỀ LÀ Ở THƯ VIỆN HOẶC CẢM BIẾN !!!
    // KHÔNG CẦN CHIA Ở ĐÂY NỮA.

    float currentAccMagnitude = sqrt(ax_raw * ax_raw + ay_raw * ay_raw + az_raw * az_raw);
    float currentGyroMagnitude = sqrt(gx_raw * gx_raw + gy_raw * gy_raw + gz_raw * gz_raw);

    // --- Logic Đếm Bước ---
    if (checkForStep(currentAccMagnitude, currentGyroMagnitude)) {
        int currentLocalSteps = 0; // Để tránh truy cập stepCountLocal nhiều lần trong mutex
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            stepCountLocal++;
            distanceLocal = (float)stepCountLocal * STEP_LENGTH;
            currentLocalSteps = stepCountLocal;
            lastStepTimeMs = millis();
            stepDetectedThisCycle = true;

            // --- LOGIC LƯU EEPROM TỐI ƯU ---
            if (currentLocalSteps > 0 && (currentLocalSteps - lastSavedStepCountEEPROM >= SAVE_STEP_INTERVAL)) {
                if (EEPROM.readInt(STEP_COUNT_ADDR) != currentLocalSteps) {
                     Serial.printf("Saving steps to EEPROM: %d (Last saved: %d)\n", currentLocalSteps, lastSavedStepCountEEPROM);
                     EEPROM.writeInt(STEP_COUNT_ADDR, currentLocalSteps);
                     if (EEPROM.commit()) {
                         Serial.printf("Step count %d saved to EEPROM.\n", currentLocalSteps);
                         lastSavedStepCountEEPROM = currentLocalSteps;
                     } else {
                         Serial.println("EEPROM commit failed!");
                     }
                } else {
                     lastSavedStepCountEEPROM = currentLocalSteps; // EEPROM đã đúng, chỉ cập nhật biến last
                }
            }
            xSemaphoreGive(dataMutex);
            // Serial.printf("Step! Total: %d\n", currentLocalSteps); // Có thể quá nhiều log
        } else { Serial.println("CRITICAL: Failed to take StepCounter dataMutex for step increment!"); }
    }

    // Reset cờ phát hiện bước
    if (currentAccMagnitude < STEP_RESET_THRESHOLD) {
         stepDetectedThisCycle = false;
    }

    // --- Cập nhật giá trị local cuối cùng để gửi đi/hiển thị ---
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        axLocal = ax_raw; ayLocal = ay_raw; azLocal = az_raw;
        gxLocal = gx_raw; gyLocal = gy_raw; gzLocal = gz_raw;
        xSemaphoreGive(dataMutex);
    } else { Serial.println("CRITICAL: Failed to take StepCounter dataMutex for local data update!");}

    // Debug định kỳ
    static unsigned long lastIMUDebugLogTime = 0;
    if (millis() - lastIMUDebugLogTime > 5000) { // In mỗi 5 giây
        Serial.printf("QMI - Acc(g): %.2f,%.2f,%.2f | Gyr(dps): %.2f,%.2f,%.2f | Steps: %d\n",
                      axLocal, ayLocal, azLocal, gxLocal, gyLocal, gzLocal, stepCountLocal);
        lastIMUDebugLogTime = millis();
    }
}

// --- lowPassFilterEMA ---
float StepCounter::lowPassFilterEMA(float input, float previousValue, float alpha) {
    alpha = constrain(alpha, 0.0f, 1.0f);
    if (isnan(previousValue)) return input; // Khởi tạo bộ lọc bằng giá trị đầu tiên
    return alpha * input + (1.0f - alpha) * previousValue;
}

// --- checkForStep ---
bool StepCounter::checkForStep(float currentAccMagnitude, float currentGyroMagnitude) {
    if (!stepDetectedThisCycle &&
        currentAccMagnitude > THRESHOLD &&
        currentGyroMagnitude < STEP_DETECT_MAX_GYRO &&
        (millis() - lastStepTimeMs > STEP_DELAY)) {
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
        stepCount = -1; distance = 0.0f;
        ax = ay = az = gx = gy = gz = NAN; // Trả về NAN nếu không lấy được data
        Serial.println("CRITICAL: Failed to take StepCounter dataMutex in getData!");
    }
}

// --- resetSteps ---
void StepCounter::resetSteps() {
     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
         stepCountLocal = 0; distanceLocal = 0.0f; lastSavedStepCountEEPROM = 0;
         Serial.println("StepCounter: Steps and distance reset locally.");
         xSemaphoreGive(dataMutex);
         EEPROM.writeInt(STEP_COUNT_ADDR, 0);
         if (EEPROM.commit()) Serial.println("StepCounter: EEPROM reset to 0 steps.");
         else Serial.println("StepCounter: EEPROM commit failed during reset!");
     } else { Serial.println("CRITICAL: Failed to take StepCounter dataMutex in resetSteps!"); }
}