// src/StepCounter.cpp
#include "StepCounter.h"
#include "Config.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>     // Cần cho Wire
#include <cmath>      // Cần cho sqrt và isnan

// Khai báo Mutex I2C toàn cục (được định nghĩa trong main.cpp)
extern SemaphoreHandle_t i2cMutex;

// --- Chân I2C cho QMI8658 (Theo tài liệu board của bạn) ---
// Đảm bảo các chân này khớp với kết nối phần cứng thực tế
#ifndef QMI8658_SDA_PIN
#define QMI8658_SDA_PIN 42
#endif
#ifndef QMI8658_SCL_PIN
#define QMI8658_SCL_PIN 41
#endif

// --- EEPROM ---
#ifndef EEPROM_SIZE // Đảm bảo định nghĩa nếu chưa có trong Config.h
#define EEPROM_SIZE 512
#endif
#ifndef STEP_COUNT_ADDR
#define STEP_COUNT_ADDR 0
#endif

// --- Constructor ---
StepCounter::StepCounter()
    : qmi(), // Khởi tạo đối tượng qmi
      taskHandle(NULL), dataMutex(NULL),
      sensorReady(false), stepDetected(false), lastStepTime(0),
      stepCountLocal(0), distanceLocal(0.0f), lastSavedStepCount(0),
      axLocal(0.0f), ayLocal(0.0f), azLocal(0.0f),
      gxLocal(0.0f), gyLocal(0.0f), gzLocal(0.0f)
      // Không dùng bộ lọc EMA mặc định nữa
{
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("CRITICAL: Failed to create StepCounter data mutex!");
        // Nên có xử lý lỗi nghiêm trọng hơn, ví dụ: dừng hệ thống
    }
}

// --- Hàm begin() - Khởi tạo QMI8658 và EEPROM ---
bool StepCounter::begin() {
    sensorReady = false; // Reset trạng thái ban đầu

    // Khởi tạo EEPROM
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Warning: Failed to initialize EEPROM for StepCounter.");
        // Có thể vẫn tiếp tục mà không lưu bước
    }

    // Lấy Mutex I2C để khởi tạo cảm biến an toàn
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS * 2)) == pdTRUE) {
        Serial.println("Initializing QMI8658...");

        // Khởi tạo cảm biến QMI8658 với các chân và địa chỉ cụ thể
        // Wire phải được begin() trong main.cpp trước đó
        if (!qmi.begin(Wire, QMI8658_ADDRESS, QMI8658_SDA_PIN, QMI8658_SCL_PIN)) {
            Serial.println("!!! ERROR: Failed to find/init QMI8658!");
            Serial.println("!!! Check wiring, I2C address, SDA/SCL pins.");
            Serial.println("!!! Step counting disabled.");
        } else {
            Serial.print("QMI8658 Found! Chip ID: 0x"); Serial.println(qmi.getChipID(), HEX);

            // --- Cấu hình Accelerometer ---
            Serial.println("Configuring QMI8658 Accelerometer (Range: 4G, ODR: 125Hz)...");
            // Dải đo +/- 4g phù hợp cho hoạt động hàng ngày và bước chân
            // ODR 125Hz là đủ nhanh để bắt đỉnh gia tốc của bước chân
            qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz);
            // Có thể thử LPF_MODE_3 để lọc nhiễu nhiều hơn nếu tín hiệu ồn
            // qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz, SensorQMI8658::LPF_MODE_3);

            // --- Cấu hình Gyroscope ---
            Serial.println("Configuring QMI8658 Gyroscope (Range: 256dps, ODR: 112Hz)...");
            // Dải đo +/- 256dps thường đủ cho chuyển động tay thông thường
            qmi.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS, SensorQMI8658::GYR_ODR_112_1Hz);
            // Có thể thử LPF_MODE_3
            // qmi.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS, SensorQMI8658::GYR_ODR_112_1Hz, SensorQMI8658::LPF_MODE_3);

            // Bật cảm biến
            qmi.enableAccelerometer();
            qmi.enableGyroscope();

            Serial.println("QMI8658 Configured and Enabled.");
            sensorReady = true; // Đánh dấu cảm biến sẵn sàng
        }
        xSemaphoreGive(i2cMutex); // Luôn trả mutex sau khi xong
    } else {
        Serial.println("CRITICAL: Timeout waiting for I2C mutex during QMI8658 init!");
        // Không thể khởi tạo cảm biến, nên dừng lại hoặc báo lỗi rõ ràng
    }

    // Khôi phục bước chân từ EEPROM
    if (EEPROM.length() > 0) { // Kiểm tra EEPROM đã init chưa
        stepCountLocal = EEPROM.readInt(STEP_COUNT_ADDR);
        // Kiểm tra giá trị đọc được có hợp lý không
        if (stepCountLocal < 0 || stepCountLocal > 1000000) { // Giới hạn hợp lý
            Serial.printf("Warning: Invalid step count read from EEPROM (%d), resetting to 0.\n", stepCountLocal);
            stepCountLocal = 0;
        }
        lastSavedStepCount = stepCountLocal; // Đồng bộ giá trị đã lưu
        distanceLocal = (float)stepCountLocal * STEP_LENGTH; // Ép kiểu float
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
        Serial.println("QMI8658 not ready, skipping StepCounter Task creation.");
        return;
    }
    // Sử dụng Stack size lớn hơn một chút để đề phòng
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
        TaskHandle_t tempHandle = taskHandle; // Lưu lại handle trước khi xóa
        taskHandle = NULL; // Đặt NULL trước để tránh race condition nhỏ
        vTaskDelete(tempHandle);
        Serial.println("StepCounter task stopped.");

        // Đưa cảm biến về chế độ tiết kiệm pin khi dừng task (nếu hỗ trợ)
        // if (sensorReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        //     qmi.disableAccelerometer(); // Thư viện có thể không có hàm này
        //     qmi.disableGyroscope();
        //     xSemaphoreGive(i2cMutex);
        //     Serial.println("QMI8658 disabled.");
        // }
    }
}

// --- taskFunction ---
void StepCounter::taskFunction(void* pvParameters) {
    StepCounter* instance = static_cast<StepCounter*>(pvParameters);
    if (instance == nullptr) {
         Serial.println("CRITICAL: StepCounter task received null parameter!");
         vTaskDelete(NULL); // Tự hủy task
         return;
    }
    Serial.println("StepCounter Task running...");
    while (true) {
        instance->updateSensor();
        // Delay 100ms (10Hz) - Phù hợp với ODR 125Hz/112Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- updateSensor (Phiên bản nâng cấp) ---
void StepCounter::updateSensor() {
    if (!sensorReady) {
        // Thử khởi tạo lại nếu chưa sẵn sàng (chỉ thử vài lần)
        static int beginRetryCount = 0;
        if (beginRetryCount < 5) {
             Serial.println("StepCounter: Sensor not ready, attempting to re-initialize...");
             beginRetryCount++;
             begin(); // Gọi lại hàm begin để thử lại
             vTaskDelay(pdMS_TO_TICKS(1000)); // Chờ 1 giây trước khi thử lại
        } else {
             // Vẫn ở đây nếu không thể khởi tạo được
             // Serial.println("StepCounter: Sensor initialization failed permanently.");
        }
        return;
    }

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    bool readSuccess = false;
    const int MAX_READ_ATTEMPTS = 3; // Số lần thử đọc tối đa

    // Lấy Mutex I2C
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; ++attempt) {
            // Đọc dữ liệu Accel và Gyro
            if (qmi.getAccelerometer(ax, ay, az) && qmi.getGyroscope(gx, gy, gz)) {
                // Kiểm tra giá trị NAN (Not a Number) - phòng trường hợp thư viện trả về lỗi
                if (!isnan(ax) && !isnan(ay) && !isnan(az) && !isnan(gx) && !isnan(gy) && !isnan(gz)) {
                    readSuccess = true;
                    break; // Thoát vòng lặp nếu đọc thành công
                } else {
                    Serial.println("Warning: QMI8658 read returned NAN values.");
                }
            } else {
                Serial.printf("Warning: QMI8658 read failed (attempt %d).\n", attempt + 1);
            }
            // Chờ một chút trước khi thử lại
            if (attempt < MAX_READ_ATTEMPTS - 1) {
                vTaskDelay(pdMS_TO_TICKS(10)); // Delay ngắn giữa các lần thử
            }
        }
        xSemaphoreGive(i2cMutex); // Trả mutex
    } else {
        Serial.println("Timeout waiting for I2C mutex in StepCounter updateSensor!");
        return;
    }

    // Nếu đọc lỗi sau tất cả các lần thử
    if (!readSuccess) {
        Serial.println("ERROR: QMI8658 read failed after multiple attempts.");
        // Reset giá trị local để tránh gửi dữ liệu cũ/sai
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
             axLocal = ayLocal = azLocal = gxLocal = gyLocal = gzLocal = 0.0f;
             xSemaphoreGive(dataMutex);
        }
        // Có thể đặt sensorReady = false để thử re-init lần sau
        // sensorReady = false;
        return;
    }

    // --- Xử lý dữ liệu hợp lệ ---
    // Tính độ lớn vector
    float accMagnitude = sqrt(ax * ax + ay * ay + az * az);
    float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

    // --- Logic Đếm Bước (Sử dụng độ lớn trực tiếp) ---
    if (detectStep(accMagnitude, gyroMagnitude)) {
        int currentStepCount = 0;
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            stepCountLocal++;
            distanceLocal = (float)stepCountLocal * STEP_LENGTH;
            currentStepCount = stepCountLocal;
            lastStepTime = millis();
            stepDetected = true;

            // --- LOGIC LƯU EEPROM TỐI ƯU ---
            if (currentStepCount > 0 && (currentStepCount - lastSavedStepCount >= SAVE_STEP_INTERVAL)) {
                // Chỉ ghi khi thực sự cần
                if (EEPROM.readInt(STEP_COUNT_ADDR) != currentStepCount) { // Kiểm tra giá trị hiện tại trước khi ghi
                     Serial.printf("Saving steps: %d (Last saved: %d)\n", currentStepCount, lastSavedStepCount);
                     EEPROM.writeInt(STEP_COUNT_ADDR, currentStepCount);
                     if (EEPROM.commit()) {
                         Serial.printf("Step count saved to EEPROM: %d\n", currentStepCount);
                         lastSavedStepCount = currentStepCount;
                     } else {
                         Serial.println("EEPROM commit failed!");
                         // Nếu commit lỗi, không cập nhật lastSavedStepCount để thử lại lần sau
                     }
                } else {
                     // Giá trị trong EEPROM đã đúng, cập nhật lastSavedStepCount
                     lastSavedStepCount = currentStepCount;
                }
            }
            // Serial.printf("Step detected! Total: %d\n", currentStepCount);
            xSemaphoreGive(dataMutex);
        } else { Serial.println("CRITICAL: Failed to take StepCounter dataMutex!"); }
    }

    // Reset cờ phát hiện bước
    if (accMagnitude < STEP_RESET_THRESHOLD) {
         stepDetected = false;
    }

    // --- Cập nhật giá trị local cuối cùng ---
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        axLocal = ax; ayLocal = ay; azLocal = az;
        gxLocal = gx; gyLocal = gy; gzLocal = gz;
        xSemaphoreGive(dataMutex);
    } else { Serial.println("CRITICAL: Failed to take StepCounter dataMutex for local update!");}

    // Debug định kỳ (ít hơn)
    static unsigned long lastIMUDebugTime = 0;
    if (millis() - lastIMUDebugTime > 5000) { // In mỗi 5 giây
        Serial.printf("QMI - Acc(g): %.2f,%.2f,%.2f | Gyr(dps): %.2f,%.2f,%.2f | Steps: %d\n",
                      axLocal, ayLocal, azLocal, gxLocal, gyLocal, gzLocal, stepCountLocal);
        lastIMUDebugTime = millis();
        // In stack còn lại để theo dõi
        // UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL);
        // Serial.printf("StepCounter Task Stack High Water Mark: %u bytes\n", stackRemaining * sizeof(StackType_t));
    }
}

// --- lowPassFilter (Giữ nguyên) ---
float StepCounter::lowPassFilter(float input, float previous, float alpha) {
    // Đảm bảo alpha hợp lệ
    alpha = constrain(alpha, 0.0f, 1.0f);
    // Xử lý trường hợp đầu tiên hoặc previous là NAN
    if (isnan(previous)) {
         return input;
    }
    return alpha * previous + (1.0f - alpha) * input;
}

// --- detectStep (Giữ nguyên) ---
bool StepCounter::detectStep(float accMagnitude, float gyroMagnitude) {
    // Dùng các hằng số từ Config.h
    if (!stepDetected &&
        accMagnitude > THRESHOLD &&            // Gia tốc đủ lớn
        gyroMagnitude < STEP_DETECT_MAX_GYRO && // Không quay quá nhanh
        (millis() - lastStepTime > STEP_DELAY)) // Đủ thời gian trễ
    {
        return true;
    }
    return false;
}

// --- getData (Giữ nguyên) ---
void StepCounter::getData(int& stepCount, float& distance, float& ax, float& ay, float& az,
                          float& gx, float& gy, float& gz) {
     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        stepCount = stepCountLocal; distance = distanceLocal;
        ax = axLocal; ay = ayLocal; az = azLocal;
        gx = gxLocal; gy = gyLocal; gz = gzLocal;
        xSemaphoreGive(dataMutex);
    } else { stepCount = -1; /*...*/ }
}

// --- resetSteps (Giữ nguyên) ---
void StepCounter::resetSteps() {
     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
         stepCountLocal = 0; distanceLocal = 0.0f; lastSavedStepCount = 0;
         Serial.println("StepCounter local data reset by resetSteps().");
         xSemaphoreGive(dataMutex);
         EEPROM.writeInt(STEP_COUNT_ADDR, 0);
         if (EEPROM.commit()) Serial.println("Step count reset to 0 in EEPROM (from resetSteps).");
         else Serial.println("Failed to commit EEPROM during step reset!");
     } else { Serial.println("CRITICAL: Failed to take dataMutex in resetSteps!"); }
}