// src/HeartRateSpO2.cpp
#include "HeartRateSpO2.h"
// Config.h đã được include trong HeartRateSpO2.h
#include <Arduino.h>
#include <Wire.h>
#include <cmath> // Cho isnan, abs, sqrt

// Khai báo Mutex I2C toàn cục (được định nghĩa trong main.cpp)
extern SemaphoreHandle_t i2cMutex;

// Bảng tra cứu SpO2 (Từ nanoPulsePPG.ino)
static const uint8_t spo2LookupTable[184] PROGMEM = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
    97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
    90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
    80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
    66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
    28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
    3, 2, 1 };

// --- Constructor ---
HeartRateSpO2::HeartRateSpO2()
    : sensor(), pulseIR(), pulseRed(), bpmSmoother(), // Khởi tạo các đối tượng
      _wire(nullptr), taskHandle(NULL), dataMutex(NULL),
      sensorReady(false),
      heartRateFinal(0), spo2Final(-999),
      irValueRawLast(0), redValueRawLast(0),
      lastBeatTimestampMs(0)
      // hrIsStable(false), spo2IsStable(false) // Bỏ qua logic ổn định phức tạp ban đầu
{
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("CRITICAL: Failed to create HeartRateSpO2 data mutex!");
    }
}

// --- Hàm begin() ---
bool HeartRateSpO2::begin(TwoWire &wireInstance) {
    sensorReady = false;
    _wire = &wireInstance;

    Serial.println("Initializing HeartRateSpO2 Module (using Custom MAX30102 Driver)...");
    if (!sensor.begin(*_wire, MAX30102_ADDRESS)) { // Địa chỉ từ Config.h
        Serial.println("!!! ERROR: MAX30102_Custom Driver - Device NOT FOUND!");
        return false;
    }

    Serial.println("MAX30102_Custom Driver Found. Configuring device...");
    // Sử dụng các hằng số từ Config.h để cấu hình cảm biến
    sensor.setupDevice(
        MAX_LED_BRIGHTNESS,    // 1. ledBrightnessIR
        MAX_LED_BRIGHTNESS,    // 2. ledBrightnessRed (Hoặc MAX_RED_LED_BRIGHTNESS nếu khác)
        MAX_SAMPLE_AVERAGE,    // 3. sampleAverage
        MAX_LED_MODE,          // 4. ledModeConfig (Ví dụ: 0x03 cho SpO2, 0x07 cho Multi-LED)
        // --- THÊM 4 THAM SỐ CHO SLOT ---
        MAX_SLOT1_LEDS,        // 5. slot1ActiveLEDs (Ví dụ: LED IR)
        MAX_SLOT2_LEDS,        // 6. slot2ActiveLEDs (Ví dụ: LED Red)
        MAX_SLOT3_LEDS,        // 7. slot3ActiveLEDs (Ví dụ: 0x00 nếu không dùng)
        MAX_SLOT4_LEDS,        // 8. slot4ActiveLEDs (Ví dụ: 0x00 nếu không dùng)
        // -----------------------------
        MAX_SAMPLE_RATE,       // 9. sampleRateHz
        MAX_PULSE_WIDTH,       // 10. pulseWidthUs
        MAX_ADC_RANGE          // 11. adcRangeNA
    );
    sensor.printRevisionID();

    sensorReady = true;
    Serial.println("HeartRateSpO2 Module Initialized and Sensor Configured.");
    return true;
}

// --- startTask ---
void HeartRateSpO2::startTask(UBaseType_t priority) {
    if (!sensorReady) {
        Serial.println("MAX30102 not ready, skipping HeartRateSpO2 Task creation.");
        return;
    }
    xTaskCreate(taskFunction, "HeartRateSpO2Task", 4096, this, priority, &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating HeartRateSpO2 Task!");
    else Serial.println("HeartRateSpO2 Task started.");
}

// --- stopTask ---
void HeartRateSpO2::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle; taskHandle = NULL;
        vTaskDelete(tempHandle);
        if (sensorReady) sensor.shutDown(); // Tắt cảm biến khi dừng task
        Serial.println("HeartRateSpO2 task stopped and sensor shutdown.");
    }
}

// --- taskFunction ---
void HeartRateSpO2::taskFunction(void* pvParameters) {
    HeartRateSpO2* instance = static_cast<HeartRateSpO2*>(pvParameters);
    if (instance == nullptr) { vTaskDelete(NULL); return; }

    Serial.println("HeartRateSpO2 Task running...");
    // Tính toán delay dựa trên tần suất lấy mẫu của chip hoặc tần suất xử lý mong muốn
    // Nếu MAX_SAMPLE_RATE là 100Hz, đọc FIFO mỗi 20-50ms là hợp lý
    TickType_t taskDelay = pdMS_TO_TICKS(1000 / MAX_SAMPLE_RATE); // Khớp với SR chip
    if (taskDelay < pdMS_TO_TICKS(20)) taskDelay = pdMS_TO_TICKS(20); // Giới hạn tối thiểu 20ms (50Hz)

    while (true) {
        instance->updateSensorData();
        vTaskDelay(taskDelay);
    }
}

// --- updateSensorData (Logic chính) ---
void HeartRateSpO2::updateSensorData() {
    if (!sensorReady) return;

    // sensor.readFIFO() đã được bảo vệ bằng i2cMutex bên trong Max30102_Custom.cpp
    uint16_t newSamples = sensor.readFIFO();

    if (newSamples == 0 && sensor.available() == 0) {
        // Không có dữ liệu mới từ cảm biến
        return;
    }

    // Biến tạm để tính toán trong chu kỳ này
    int currentCalculatedHr = 0;
    int currentCalculatedSpo2 = -999;

    // Xử lý tất cả các mẫu có sẵn trong buffer của driver
    while (sensor.available()) {
        uint32_t ir = sensor.getIR();
        uint32_t red = sensor.getRed();
        sensor.nextSample(); // Quan trọng: Di chuyển con trỏ đọc của buffer driver

        // Cập nhật giá trị thô cuối cùng (để gửi qua BLE nếu cần)
        irValueRawLast = ir;
        redValueRawLast = red;

        // Kiểm tra có ngón tay không
        if (ir < IR_DETECT_THRESHOLD) { // Ngưỡng từ Config.h
             // Nếu tín hiệu IR quá thấp, không xử lý HR/SpO2
             // Reset giá trị nếu trước đó có (trong dataMutex)
             if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                 if (heartRateFinal != 0 || spo2Final != -999) {
                    // Serial.println("HR_SPO2: Finger off or low signal.");
                    heartRateFinal = 0;
                    spo2Final = -999;
                    lastBeatTimestampMs = 0; // Reset thời gian nhịp
                 }
                xSemaphoreGive(dataMutex);
            }
            continue; // Xử lý mẫu tiếp theo (nếu có)
        }

        // --- Xử lý tín hiệu IR cho Nhịp tim (HR) ---
        int16_t acIR = pulseIR.dc_filter(ir);    // Loại bỏ DC
        acIR = pulseIR.ma_filter(acIR);          // Làm mịn AC

        if (pulseIR.isBeat(acIR)) { // Phát hiện nhịp
            unsigned long currentTimeMs = millis();
            if (lastBeatTimestampMs != 0) { // Chỉ tính nếu đã có nhịp trước
                long beatIntervalMs = currentTimeMs - lastBeatTimestampMs;
                // Giới hạn khoảng thời gian giữa các nhịp (ví dụ: 200ms - 2000ms tương ứng 30-300 BPM)
                // (60000 / BPM_MAX) < beatIntervalMs < (60000 / BPM_MIN)
                if (beatIntervalMs > (60000 / BPM_MAX) && beatIntervalMs < (60000 / BPM_MIN)) {
                    int bpm = 60000 / beatIntervalMs;
                    currentCalculatedHr = bpmSmoother.filter(bpm); // Làm mịn giá trị BPM
                }
            }
            lastBeatTimestampMs = currentTimeMs; // Cập nhật thời điểm nhịp cuối
        }

        // --- Xử lý tín hiệu Red (cũng cần lọc để lấy AC/DC) ---
        int16_t acRed = pulseRed.dc_filter(red);
        // acRed = pulseRed.ma_filter(acRed); // Tùy chọn làm mịn Red AC
        pulseRed.isBeat(acRed); // Gọi isBeat cho Red để cập nhật avgAC/avgDC của nó

        // --- Tính toán SpO2 (chỉ khi có tín hiệu nhịp tim tốt) ---
        if (currentCalculatedHr > BPM_MIN - 5) { // Chỉ tính SpO2 nếu HR có vẻ hợp lý
            int32_t irDC_val = pulseIR.avgDC();
            int16_t irAC_val = pulseIR.avgAC();
            int32_t redDC_val = pulseRed.avgDC();
            int16_t redAC_val = pulseRed.avgAC();

            // Kiểm tra các giá trị AC/DC trước khi tính toán
            // Ngưỡng này cần tinh chỉnh kỹ lưỡng!
            const int32_t MIN_DC_FOR_SPO2 = 10000; // Ví dụ: DC phải đủ lớn
            const int16_t MIN_AC_FOR_SPO2 = PULSE_MIN_BEAT_AMPLITUDE / 2; // Ví dụ: AC phải đủ lớn

            if (abs(irDC_val) > MIN_DC_FOR_SPO2 && abs(redDC_val) > MIN_DC_FOR_SPO2 &&
                abs(irAC_val) > MIN_AC_FOR_SPO2 && abs(redAC_val) > MIN_AC_FOR_SPO2)
            {
                // Tính R ratio: (RedAC / RedDC) / (IRAC / IRDC)
                double R_numerator = (double)abs(redAC_val) * abs(irDC_val);
                double R_denominator = (double)abs(irAC_val) * abs(redDC_val);

                if (R_denominator != 0) {
                    double R = R_numerator / R_denominator;
                    int R_scaled_for_table = (int)(R * 100.0); // Nhân R với 100 để tra bảng

                    if (R_scaled_for_table >= 0 && R_scaled_for_table < 184) { // Giới hạn của bảng spo2LookupTable
                        currentCalculatedSpo2 = pgm_read_byte_near(&spo2LookupTable[R_scaled_for_table]);
                        // Kiểm tra lại với ngưỡng min/max từ Config
                        if (currentCalculatedSpo2 < SPO2_MIN_VALID || currentCalculatedSpo2 > SPO2_MAX_VALID) {
                            currentCalculatedSpo2 = -999;
                        }
                    } else {
                        currentCalculatedSpo2 = -999; // R ratio ngoài khoảng cho bảng
                    }
                } else { currentCalculatedSpo2 = -999; } // Mẫu số bằng 0
            } else { currentCalculatedSpo2 = -999; } // Tín hiệu AC/DC không đủ mạnh
        } // Kết thúc tính SpO2
    } // Kết thúc vòng lặp while(sensor.available())

    // --- Cập nhật giá trị cuối cùng (đã xử lý) vào biến thành viên ---
    // (Tạm thời chưa có logic ổn định hóa phức tạp, lấy giá trị cuối cùng tính được)
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (currentCalculatedHr > 0) { // Chỉ cập nhật HR nếu có giá trị mới
            heartRateFinal = currentCalculatedHr;
        }
        // Cập nhật SpO2 nếu có giá trị mới hợp lệ, nếu không giữ giá trị cũ (hoặc reset nếu không có ngón tay)
        if (currentCalculatedSpo2 >= SPO2_MIN_VALID) {
            spo2Final = currentCalculatedSpo2;
        } else if (irValueRawLast < IR_DETECT_THRESHOLD) { // Nếu không có ngón tay thì chắc chắn reset SpO2
            spo2Final = -999;
        }
        // Nếu currentCalculatedSpo2 là -999 nhưng đang có ngón tay, có thể giữ giá trị spo2Final cũ
        // để tránh hiển thị "--" liên tục khi tín hiệu hơi nhiễu.
        // Hoặc, nếu muốn nó phản ánh ngay lập tức:
        // else { spo2Final = -999; }


        xSemaphoreGive(dataMutex);
    } else { Serial.println("Timeout taking dataMutex for final HR/SpO2 update!"); }

    // Debug định kỳ
    static unsigned long lastDebugOutputTime = 0;
    if (millis() - lastDebugOutputTime > 3000) { // In mỗi 3 giây
        Serial.printf("HR_SPO2 Module - HR: %d, SpO2: %d, IR: %ld, Red: %ld\n",
                      heartRateFinal, spo2Final, irValueRawLast, redValueRawLast);
        lastDebugOutputTime = millis();
    }
}

// --- getData ---
void HeartRateSpO2::getData(int& heartRate, int& spo2, long& irValue, long& redValue) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        heartRate = heartRateFinal;
        spo2 = spo2Final;
        irValue = irValueRawLast;   // Trả về giá trị thô cuối cùng
        redValue = redValueRawLast;
        xSemaphoreGive(dataMutex);
    } else {
        // Lỗi nghiêm trọng nếu không lấy được mutex
        heartRate = -1; spo2 = -1; irValue = -1; redValue = -1;
        Serial.println("CRITICAL: Failed to take HRSpO2 dataMutex in getData!");
    }
}