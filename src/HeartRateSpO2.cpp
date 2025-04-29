// src/HeartRateSpO2.cpp
#include "HeartRateSpO2.h"
#include "Config.h"
#include "heartRate.h" // Đảm bảo file này tồn tại và định nghĩa checkForBeat()
#include <Arduino.h>
#include <Wire.h>      // Cần cho Wire
#include <cmath>       // Cần cho isnan, abs, sqrt

// Khai báo Mutex I2C toàn cục
extern SemaphoreHandle_t i2cMutex;

// Constructor
HeartRateSpO2::HeartRateSpO2()
    : particleSensor(), // Khởi tạo đối tượng cảm biến
      _wire(nullptr),   // Khởi tạo con trỏ Wire
      taskHandle(NULL), dataMutex(NULL),
      sensorReady(false),
      heartRateStable(0), spo2Stable(-999), // Khởi tạo giá trị ổn định
      irValueLocal(0), redValueLocal(0),
      rateSpot(0), lastBeat(0),
      spo2_redDC(0.0f), spo2_irDC(0.0f), // Khởi tạo bộ lọc DC
      hrBufferIndex(0), spo2BufferIndex(0),
      hrValidCount(0), spo2ValidCount(0),
      hrIsStable(false), spo2IsStable(false) // Khởi tạo cờ ổn định
{
    memset(rates, 0, sizeof(rates));         // Xóa buffer rates HR
    memset(hrBuffer, 0, sizeof(hrBuffer));     // Xóa buffer ổn định HR
    memset(spo2Buffer, 0, sizeof(spo2Buffer)); // Xóa buffer ổn định SpO2
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("CRITICAL: Failed to create HeartRateSpO2 data mutex!");
    }
}

// --- Hàm begin() - Khởi tạo MAX3010x ---
bool HeartRateSpO2::begin(TwoWire &wireInstance) {
    sensorReady = false;
    _wire = &wireInstance; // Lưu đối tượng Wire

    // Lấy Mutex I2C
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS * 2)) == pdTRUE) {
        Serial.println("Initializing MAX3010x...");

        // Khởi tạo cảm biến với địa chỉ và bus I2C cụ thể
        // Sử dụng địa chỉ MAX30102_ADDRESS từ Config.h
        if (!particleSensor.begin(*_wire, I2C_SPEED_STANDARD, MAX30102_ADDRESS)) { // Dùng tốc độ chuẩn 100kHz
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.printf("MAX3010x NOT FOUND at address 0x%02X!\n", MAX30102_ADDRESS);
            Serial.println("Check wiring, I2C address.");
            Serial.println("HeartRate/SpO2 features disabled.");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        } else {
            Serial.println("MAX3010x Found and Initialized!");
            sensorReady = true;

            // --- Cấu hình chi tiết cảm biến ---
            Serial.println("Configuring MAX3010x...");
            // Giá trị ví dụ - CẦN TINH CHỈNH
            byte ledBrightness = 60;             // Độ sáng LED (~11mA)
            byte sampleAverage = 4;              // Trung bình 4 mẫu
            byte ledMode = 2;                    // Mode 2: Red + IR
            int sampleRate = SAMPLE_RATE_ACTIVE; // Tần số từ Config.h (ví dụ 50Hz)
            int pulseWidth = 411;                // Độ rộng xung (411us)
            int adcRange = 4096;                 // Dải ADC (4096 nA)

            particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
            particleSensor.setPulseAmplitudeRed(ledBrightness);
            particleSensor.setPulseAmplitudeIR(ledBrightness);
            particleSensor.setPulseAmplitudeGreen(0); // Tắt Green

            Serial.println("MAX3010x Configured.");
        }
        xSemaphoreGive(i2cMutex); // Trả mutex
    } else {
        Serial.println("CRITICAL: Timeout waiting for I2C mutex during MAX3010x init!");
    }
    return sensorReady;
}

// --- startTask ---
void HeartRateSpO2::startTask(UBaseType_t priority) {
    if (!sensorReady) {
        Serial.println("MAX3010x not ready, skipping HeartRate Task creation.");
        return;
    }
    // Tăng stack size nếu cần thiết, 4096 thường là đủ
    xTaskCreate(taskFunction, "HeartRateSpO2Task", 4096, this, priority, &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating HeartRate Task!");
    else Serial.println("HeartRate Task started.");
}

// --- stopTask ---
void HeartRateSpO2::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle;
        taskHandle = NULL;
        vTaskDelete(tempHandle);
        if (sensorReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            particleSensor.shutDown(); // Đưa cảm biến về chế độ nguồn thấp
            xSemaphoreGive(i2cMutex);
            Serial.println("MAX3010x shutdown.");
        }
        Serial.println("HeartRateSpO2 task stopped.");
    }
}

// --- taskFunction ---
void HeartRateSpO2::taskFunction(void* pvParameters) {
    HeartRateSpO2* instance = static_cast<HeartRateSpO2*>(pvParameters);
    if (instance == nullptr) { vTaskDelete(NULL); return; }

    Serial.println("HeartRate Task running...");
    // Tính toán delay dựa trên sample rate để khớp (nếu cần)
    // Ví dụ: 50Hz -> 1000ms / 50 = 20ms delay
    TickType_t taskDelay = pdMS_TO_TICKS(1000 / SAMPLE_RATE_ACTIVE);
    while (true) {
        instance->updateSensor();
        vTaskDelay(taskDelay); // Delay phù hợp với tần số lấy mẫu
    }
}

// --- updateSensor (Logic chính với ổn định hóa) ---
void HeartRateSpO2::updateSensor() {
    if (!sensorReady) return;

    long irValue = 0, redValue = 0;
    bool readSuccess = false;

    // Đọc I2C
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Có thể cần kiểm tra FIFO trước khi đọc nếu thư viện hỗ trợ
        // particleSensor.check(); // Ví dụ
        // while (particleSensor.available()) {
        //    irValue = particleSensor.getFIFOIR();
        //    redValue = particleSensor.getFIFORed();
        //    readSuccess = true; // Chỉ lấy mẫu cuối cùng từ FIFO hoặc xử lý tất cả
        // }
        // Nếu đọc trực tiếp:
        irValue = particleSensor.getIR();
        redValue = particleSensor.getRed();
        if (irValue > 0 || redValue > 0 || lastBeat == 0) readSuccess = true;
        else Serial.println("Warning: MAX3010x read zero values.");
        xSemaphoreGive(i2cMutex);
    } else { Serial.println("Timeout waiting for I2C mutex in HeartRateSpO2"); return; }

    if (!readSuccess) {
        // Nếu đọc lỗi, logic kiểm tra ổn định sẽ tự động xử lý
        checkHrStability(0); // Báo giá trị không hợp lệ
        checkSpo2Stability(-999);
        // Cập nhật giá trị thô về 0
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            irValueLocal = 0; redValueLocal = 0;
            xSemaphoreGive(dataMutex);
        }
        return;
    }

    // --- Tính toán giá trị thô ---
    int currentHr = 0;
    int currentSpo2 = -999;

    if (irValue > IR_THRESHOLD) { // Sử dụng ngưỡng từ Config.h
        // Tính HR
        if (checkForBeat(irValue)) {
            long delta = millis() - lastBeat;
            if (delta > 150 && delta < 3000) { // Thêm giới hạn delta max (20 BPM min)
                lastBeat = millis();
                float beatsPerMinute = 60000.0f / delta;
                if (beatsPerMinute < BPM_MAX && beatsPerMinute > BPM_MIN) { // Dùng hằng số Config
                    rates[rateSpot++] = (byte)beatsPerMinute;
                    rateSpot %= 4;
                    int sum = 0; byte validSamples = 0;
                    for (byte x = 0; x < 4; x++) {
                        if (rates[x] >= BPM_MIN && rates[x] < BPM_MAX) { sum += rates[x]; validSamples++; }
                    }
                    if (validSamples > 0) currentHr = sum / validSamples;
                }
            }
        }
        // Tính SpO2
        float spo2_calculated = calculateSpO2(redValue, irValue);
        // Không kiểm tra min/max ở đây nữa, để hàm ổn định làm
        currentSpo2 = (int)(spo2_calculated + 0.5f); // Làm tròn

    } else { // Không có ngón tay
        currentHr = 0;
        currentSpo2 = -999;
        // Reset trạng thái khi không có ngón tay
        if (hrValidCount > 0 || spo2ValidCount > 0) { // Chỉ reset nếu trước đó có dữ liệu
            memset(rates, 0, sizeof(rates)); rateSpot = 0; lastBeat = 0;
            spo2_redDC = 0.0f; spo2_irDC = 0.0f;
            // Reset cả buffer ổn định
             memset(hrBuffer, 0, sizeof(hrBuffer)); hrBufferIndex = 0; hrValidCount = 0;
             memset(spo2Buffer, 0, sizeof(spo2Buffer)); spo2BufferIndex = 0; spo2ValidCount = 0;
             hrIsStable = false; spo2IsStable = false;
             // Cập nhật giá trị stable về mặc định
             if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                  heartRateStable = 0; spo2Stable = -999;
                  xSemaphoreGive(dataMutex);
             }
        }
    }

    // --- Kiểm tra và cập nhật độ ổn định ---
    checkHrStability(currentHr);
    checkSpo2Stability(currentSpo2);

    // --- Cập nhật giá trị IR/Red thô ---
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        irValueLocal = irValue;
        redValueLocal = redValue;
        xSemaphoreGive(dataMutex);
    }

    // Debug (Giữ nguyên hoặc thay đổi tần suất)
    // static unsigned long lastHrDebugTime = 0;
    // if (millis() - lastHrDebugTime > 5000) { /* ... */ }
}

// --- Hàm kiểm tra ổn định HR ---
void HeartRateSpO2::checkHrStability(int currentHr) {
    bool wasStable = hrIsStable;
    hrIsStable = false; // Reset cờ

    // Chỉ thêm vào buffer nếu HR hợp lệ (theo min/max)
    if (currentHr >= BPM_MIN && currentHr < BPM_MAX) {
        hrBuffer[hrBufferIndex] = currentHr;
        hrBufferIndex = (hrBufferIndex + 1) % STABLE_BUFFER_SIZE;
        if (hrValidCount < STABLE_BUFFER_SIZE) hrValidCount++;
    } else {
        // Nếu giá trị không hợp lệ, coi như mất 1 mẫu hợp lệ trong buffer
        if (hrValidCount > 0) hrValidCount--;
    }

    // Kiểm tra ổn định nếu đủ mẫu
    if (hrValidCount >= MIN_VALID_FOR_STABLE) {
        int minVal = hrBuffer[0], maxVal = hrBuffer[0];
        int sum = 0; byte count = 0;

        // Tìm min/max và tính tổng các mẫu hợp lệ trong buffer
        // Cách hiệu quả hơn là chỉ xét hrValidCount phần tử cuối
        int startIndex = (hrBufferIndex - hrValidCount + STABLE_BUFFER_SIZE) % STABLE_BUFFER_SIZE;
        minVal = 300; // Đặt giá trị khởi tạo ngoài khoảng
        maxVal = 0;
        for (int i = 0; i < hrValidCount; ++i) {
            int index = (startIndex + i) % STABLE_BUFFER_SIZE;
            int val = hrBuffer[index];
             if (val >= BPM_MIN && val < BPM_MAX) { // Double check hợp lệ
                 sum += val;
                 if (val < minVal) minVal = val;
                 if (val > maxVal) maxVal = val;
                 count++;
             }
        }


        // Nếu đủ mẫu thực tế và độ chênh lệch nhỏ
        if (count >= MIN_VALID_FOR_STABLE && (maxVal - minVal) <= HR_STABILITY_THRESHOLD) {
            int stableValue = sum / count; // Giá trị ổn định là trung bình
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Chỉ cập nhật nếu giá trị mới khác giá trị cũ (tùy chọn)
                if (heartRateStable != stableValue) {
                    heartRateStable = stableValue;
                    // Serial.printf("HR Stable -> %d\n", heartRateStable); // Debug
                }
                hrIsStable = true; // Đặt cờ ổn định
                xSemaphoreGive(dataMutex);
            } else { Serial.println("Timeout taking dataMutex for stable HR update!"); }
        }
    }

    // Nếu không ổn định và trước đó là ổn định -> Reset giá trị stable
    if (!hrIsStable && wasStable) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            heartRateStable = 0; // Reset về 0
            // Serial.println("HR Unstable"); // Debug
            xSemaphoreGive(dataMutex);
        }
    }
}

// --- Hàm kiểm tra ổn định SpO2 ---
void HeartRateSpO2::checkSpo2Stability(int currentSpo2) {
    bool wasStable = spo2IsStable;
    spo2IsStable = false;

    // Chỉ thêm vào buffer nếu SpO2 hợp lệ (>= SPO2_MIN và <= SPO2_MAX)
    if (currentSpo2 >= SPO2_MIN && currentSpo2 <= SPO2_MAX) {
        spo2Buffer[spo2BufferIndex] = currentSpo2;
        spo2BufferIndex = (spo2BufferIndex + 1) % STABLE_BUFFER_SIZE;
        if (spo2ValidCount < STABLE_BUFFER_SIZE) spo2ValidCount++;
    } else {
        if (spo2ValidCount > 0) spo2ValidCount--;
    }

    // Kiểm tra ổn định
    if (spo2ValidCount >= MIN_VALID_FOR_STABLE) {
        int minVal = 101, maxVal = -1, sum = 0;
        byte count = 0;
        int startIndex = (spo2BufferIndex - spo2ValidCount + STABLE_BUFFER_SIZE) % STABLE_BUFFER_SIZE;

        for (int i = 0; i < spo2ValidCount; ++i) {
            int index = (startIndex + i) % STABLE_BUFFER_SIZE;
             int val = spo2Buffer[index];
             if (val >= SPO2_MIN && val <= SPO2_MAX) { // Kiểm tra lại
                 sum += val;
                 if (val < minVal) minVal = val;
                 if (val > maxVal) maxVal = val;
                 count++;
             }
        }

        if (count >= MIN_VALID_FOR_STABLE && (maxVal - minVal) <= SPO2_STABILITY_THRESHOLD) {
            int stableValue = (int)(((float)sum / count) + 0.5f); // Làm tròn
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (spo2Stable != stableValue) {
                     spo2Stable = stableValue;
                    // Serial.printf("SpO2 Stable -> %d\n", spo2Stable); // Debug
                }
                spo2IsStable = true;
                xSemaphoreGive(dataMutex);
            } else { Serial.println("Timeout taking dataMutex for stable SpO2 update!"); }
        }
    }

    // Reset nếu mất ổn định
    if (!spo2IsStable && wasStable) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            spo2Stable = -999; // Reset về không hợp lệ
            // Serial.println("SpO2 Unstable"); // Debug
            xSemaphoreGive(dataMutex);
        }
    }
}

// --- lowPassFilter (Giữ nguyên) ---
float HeartRateSpO2::lowPassFilter(float input, float previous, float alpha) {
    alpha = constrain(alpha, 0.0f, 1.0f);
    if (isnan(previous)) return input;
    return alpha * previous + (1.0f - alpha) * input;
}

// --- calculateSpO2 (Sử dụng biến thành viên và hằng số Config) ---
float HeartRateSpO2::calculateSpO2(long redValue, long irValue) {
    const float alpha = FILTER_ALPHA; // Dùng hằng số Config
    const float minDCLevel = 1000.0f; // Ngưỡng DC tối thiểu
    const float minR = 0.4f;          // Giới hạn R dưới
    const float maxR = 1.2f;          // Giới hạn R trên

    // Cập nhật DC estimates dùng biến thành viên
    spo2_redDC = lowPassFilter((float)redValue, spo2_redDC, alpha);
    spo2_irDC = lowPassFilter((float)irValue, spo2_irDC, alpha);

    // Ước tính AC
    float redAC = (float)redValue - spo2_redDC;
    float irAC = (float)irValue - spo2_irDC;

    // Kiểm tra điều kiện
    if (spo2_irDC < minDCLevel || spo2_redDC < minDCLevel || irAC == 0.0f || redAC == 0.0f) {
        return -999.0f;
    }

    // Tính R
    float R = (redAC / spo2_redDC) / (irAC / spo2_irDC);

    // Kiểm tra giới hạn R
    if (R < minR || R > maxR || isnan(R)) {
        return -999.0f;
    }

    // Công thức SpO2
    float spo2 = 104.0f - 17.0f * R;

    return spo2;
}

// --- getData (Trả về giá trị ổn định) ---
void HeartRateSpO2::getData(int& stableHeartRate, int& stableSpo2, long& lastIrValue, long& lastRedValue) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        stableHeartRate = hrIsStable ? heartRateStable : 0; // Trả về 0 nếu không ổn định
        stableSpo2 = spo2IsStable ? spo2Stable : -999;      // Trả về -999 nếu không ổn định
        lastIrValue = irValueLocal;
        lastRedValue = redValueLocal;
        xSemaphoreGive(dataMutex);
    } else { stableHeartRate = -1; stableSpo2 = -1; lastIrValue = -1; lastRedValue = -1; }
}