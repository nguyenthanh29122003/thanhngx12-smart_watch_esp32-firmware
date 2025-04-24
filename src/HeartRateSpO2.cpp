// src/HeartRateSpO2.cpp
#include "HeartRateSpO2.h"
#include "Config.h"
#include "heartRate.h"

extern SemaphoreHandle_t i2cMutex;

HeartRateSpO2::HeartRateSpO2() 
    : particleSensor(), taskHandle(NULL), rateSpot(0), lastBeat(0),
      heartRateLocal(0), spo2Local(-999), irValueLocal(0), redValueLocal(0) {
    memset(rates, 0, sizeof(rates));
    dataMutex = xSemaphoreCreateMutex();
}

void HeartRateSpO2::begin() {
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 not found! Check wiring or I2C address.");
        return;
    } else {
        particleSensor.setup();
        particleSensor.setPulseAmplitudeRed(0x0A);
        particleSensor.setPulseAmplitudeGreen(0);
        Serial.println("MAX30105 initialized and active");
    }
}

void HeartRateSpO2::startTask() {
    xTaskCreate(taskFunction, "HeartRateSpO2Task", 4096, this, 2, &taskHandle);
}

void HeartRateSpO2::stopTask() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
        taskHandle = NULL;
        particleSensor.shutDown();
        Serial.println("HeartRateSpO2 task stopped");
    }
}

void HeartRateSpO2::taskFunction(void* pvParameters) {
    HeartRateSpO2* instance = static_cast<HeartRateSpO2*>(pvParameters);
    while (true) {
        instance->updateSensor();
        vTaskDelay(20 / portTICK_PERIOD_MS); // 50Hz
    }
}

void HeartRateSpO2::updateSensor() {
    long irValue = 0, redValue = 0;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Chỉ log mutex khi cần debug
        // Serial.println("I2C mutex taken by HeartRateSpO2");
        irValue = particleSensor.getIR();
        redValue = particleSensor.getRed();
        xSemaphoreGive(i2cMutex);
        // Serial.println("I2C mutex released by HeartRateSpO2");
    } else {
        Serial.println("I2C mutex timeout in HeartRateSpO2");
        return;
    }

    // Debug dữ liệu cảm biến
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) { // In mỗi giây
        lastDebugTime = millis();
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        irValueLocal = irValue;
        redValueLocal = redValue;

        if (checkForBeat(irValue) == true) {
            Serial.println("Beat detected");
            long delta = millis() - lastBeat;
            lastBeat = millis();

            float beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                rates[rateSpot++] = (byte)beatsPerMinute;
                rateSpot %= 4;

                int avg = 0;
                for (byte x = 0; x < 4; x++) {
                    avg += rates[x];
                }
                avg /= 4;
                heartRateLocal = avg;
                Serial.printf("HR: %d\n", heartRateLocal);
            }
        }

        if (irValue > 5000) {
            float spo2 = calculateSpO2(redValue, irValue);
            if (spo2 >= 90 && spo2 <= 100) {
                spo2Local = spo2;
                Serial.printf("SpO2: %.1f%%\n", spo2Local);
            }
        } else {
            heartRateLocal = 0;
            spo2Local = -999;
        }

        xSemaphoreGive(dataMutex);
    }
}

float HeartRateSpO2::lowPassFilter(float input, float previous, float alpha) {
    return alpha * previous + (1 - alpha) * input;
}

float HeartRateSpO2::calculateSpO2(long redValue, long irValue) {
    static long redDC = 0, irDC = 0, redAC = 0, irAC = 0;
    redDC = lowPassFilter(redValue, redDC, 0.95);
    irDC = lowPassFilter(irValue, irDC, 0.95);
    redAC = redValue - redDC;
    irAC = irValue - irDC;

    if (irDC < 100 || redDC < 100 || irAC == 0 || redAC == 0) {
        return -999;
    }

    float R = ((float)redAC / redDC) / ((float)irAC / irDC);
    if (R < 0.1 || R > 5) {
        return -999;
    }

    float spo2 = 104.0 - (17.0 * R);
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