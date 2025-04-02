#include "HeartRateSpO2.h"
#include "Config.h"
#include "heartRate.h"

HeartRateSpO2::HeartRateSpO2() 
    : particleSensor(), taskHandle(NULL), rateSpot(0), lastBeat(0), sensorActive(false),
      heartRateLocal(0), spo2Local(-999), irValueLocal(0), redValueLocal(0), irFiltered(0) {
    memset(rates, 0, sizeof(rates));
    dataMutex = xSemaphoreCreateMutex();
}

void HeartRateSpO2::begin() {
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 not found! Check wiring or I2C address.");
    } else {
        particleSensor.setup(0x0A, 4, 2, 100, 411, 16384);
        particleSensor.setPulseAmplitudeGreen(0);
        particleSensor.shutDown();
        sensorActive = false;
        Serial.println("MAX30105 initialized in low-power mode");
    }
}

void HeartRateSpO2::startTask() {
    xTaskCreate(
        taskFunction,           // Hàm task
        "HeartRateSpO2Task",    // Tên task
        4096,                   // Tăng stack size từ 1536 lên 4096
        this,                   // Tham số
        2,                      // Độ ưu tiên
        &taskHandle             // Handle
    );
}

void HeartRateSpO2::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        particleSensor.shutDown();
        sensorActive = false;
        Serial.println("HeartRateSpO2 task stopped");
    }
}

void HeartRateSpO2::taskFunction(void* pvParameters) {
    HeartRateSpO2* instance = static_cast<HeartRateSpO2*>(pvParameters);
    while (true) {
        instance->updateSensor();
        vTaskDelay((instance->sensorActive ? 20 : 100) / portTICK_PERIOD_MS);
    }
}

void HeartRateSpO2::updateSensor() {
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();
    if (irValue == 0 && redValue == 0) {
        Serial.println("MAX30105 read failed, check I2C connection");
        return; // Bỏ qua nếu đọc thất bại
    }

    irFiltered = lowPassFilter(irValue, irFiltered, 0.95);

    if (irValue > 50000) {
        if (!sensorActive) {
            particleSensor.wakeUp();
            sensorActive = true;
            irFiltered = irValue;
        }

        if (!detectMotion(irValue)) {
            if (checkForBeat(irValue)) {
                unsigned long currentTime = millis();
                long delta = currentTime - lastBeat;
                lastBeat = currentTime;
                int bpm = 60 / (delta / 1000.0);
                if (bpm > 20 && bpm < 255) {
                    rates[rateSpot++] = bpm;
                    rateSpot %= 4;
                    int avg = 0;
                    for (byte i = 0; i < 4; i++) {
                        avg += rates[i];
                    }
                    avg /= 4;

                    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                        heartRateLocal = avg;
                        xSemaphoreGive(dataMutex);
                    }
                }
            }

            float spo2 = calculateSpO2(redValue, irValue);
            if (spo2 >= 90 && spo2 <= 100) {
                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                    spo2Local = spo2;
                    irValueLocal = irValue;
                    redValueLocal = redValue;
                    xSemaphoreGive(dataMutex);
                }
            }
        }
    } else {
        if (sensorActive) {
            particleSensor.shutDown();
            sensorActive = false;
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                heartRateLocal = 0;
                spo2Local = -999;
                irValueLocal = 0;
                redValueLocal = 0;
                xSemaphoreGive(dataMutex);
            }
        }
    }
}

float HeartRateSpO2::lowPassFilter(float input, float previous, float alpha) {
    return alpha * previous + (1 - alpha) * input; // Bộ lọc Low-pass đơn giản
}

bool HeartRateSpO2::detectMotion(long irValue) {
    static long lastIr = 0;
    long delta = abs(irValue - lastIr);
    lastIr = irValue;
    return delta > 10000; // Ngưỡng phát hiện chuyển động (có thể điều chỉnh)
}

float HeartRateSpO2::calculateSpO2(long redValue, long irValue) {
    static long redDC = 0, irDC = 0, redAC = 0, irAC = 0;
    // Lọc tín hiệu DC
    redDC = lowPassFilter(redValue, redDC, 0.95);
    irDC = lowPassFilter(irValue, irDC, 0.95);
    redAC = redValue - redDC;
    irAC = irValue - irDC;

    if (irDC < 1000 || redDC < 1000 || irAC == 0 || redAC == 0) {
        return -999; // Dữ liệu không hợp lệ
    }

    float R = ((float)redAC / redDC) / ((float)irAC / irDC);
    if (R < 0.3 || R > 3) {
        return -999; // Giới hạn R chặt chẽ hơn
    }

    // Hiệu chỉnh SpO2 với hằng số thực nghiệm
    float spo2 = 104.0 - (17.0 * R); // Công thức cải tiến dựa trên dữ liệu thực nghiệm
    return spo2;
}

void HeartRateSpO2::getData(int& heartRate, int& spo2, long& irValue, long& redValue) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        heartRate = heartRateLocal;
        spo2 = spo2Local;
        irValue = irValueLocal;
        redValue = redValueLocal;
        xSemaphoreGive(dataMutex);
    }
}