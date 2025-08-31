// src/Max30102_Custom.cpp
#include "Max30102_Custom.h"
#include "Config.h"   // Cho I2C_MUTEX_TIMEOUT_MS và địa chỉ MAX30102_ADDRESS
#include <Arduino.h>

// Khai báo Mutex I2C toàn cục
extern SemaphoreHandle_t i2cMutex;

MAX30102_Custom::MAX30102_Custom()
    : _wireInstance(nullptr), _i2cAddressSensor(0), _isSensorInitialized(false) {
    fifoBuffer.head = 0;
    fifoBuffer.tail = 0;
    // Khởi tạo buffer bằng 0 (tùy chọn)
    // memset(fifoBuffer.red, 0, sizeof(fifoBuffer.red));
    // memset(fifoBuffer.ir, 0, sizeof(fifoBuffer.ir));
}

bool MAX30102_Custom::begin(TwoWire &wireInstance, uint8_t i2cAddress) {
    _wireInstance = &wireInstance;
    _i2cAddressSensor = i2cAddress;
    _isSensorInitialized = false;

    // Lấy Mutex I2C để đảm bảo thao tác begin là an toàn
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS * 2)) == pdTRUE) {
        _wireInstance->beginTransmission(_i2cAddressSensor);
        if (_wireInstance->endTransmission() != 0) {
            Serial.printf("MAX30102_Custom: No device found at I2C address 0x%02X\n", _i2cAddressSensor);
        } else {
            uint8_t partID = readRegister8_raw(MAX_REG_PART_ID); // Đọc Part ID (không cần mutex nữa vì đã giữ)
            if (partID != MAX30102_EXPECTED_PART_ID) {
                Serial.printf("MAX30102_Custom: Incorrect PART_ID. Expected 0x%02X, Got 0x%02X\n", MAX30102_EXPECTED_PART_ID, partID);
            } else {
                Serial.println("MAX30102_Custom: Device found and PART_ID verified.");
                _isSensorInitialized = true;
            }
        }
        xSemaphoreGive(i2cMutex); // Trả mutex
    } else {
        Serial.println("MAX30102_Custom: Timeout taking I2C mutex in begin().");
    }
    return _isSensorInitialized;
}

void MAX30102_Custom::setupDevice(uint8_t ledBrightnessIR, uint8_t ledBrightnessRed,
                                  uint8_t sampleAverageConfig, uint8_t ledModeValue,
                                  uint8_t slot1ActiveLEDs, uint8_t slot2ActiveLEDs,
                                  uint8_t slot3ActiveLEDs, uint8_t slot4ActiveLEDs,
                                  uint16_t sampleRateHz, uint16_t pulseWidthUs, uint16_t adcRangeNA) {
    if (!_isSensorInitialized) {
        Serial.println("MAX30102_Custom: Cannot setup, device not initialized.");
        return;
    }

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        // Soft Reset
        writeRegister8_raw(MAX_REG_MODE_CONFIG, 0x40);
        delay(100); // Đợi cho reset hoàn tất

        // FIFO Configuration
        // SMP_AVE (Sample Averaging): Bit 7:5
        uint8_t avgBits = 0;
        if (sampleAverageConfig >= 32) avgBits = 0b101;      // 32 samples
        else if (sampleAverageConfig >= 16) avgBits = 0b100; // 16 samples
        else if (sampleAverageConfig >= 8) avgBits = 0b011;  // 8 samples
        else if (sampleAverageConfig >= 4) avgBits = 0b010;  // 4 samples
        else if (sampleAverageConfig >= 2) avgBits = 0b001;  // 2 samples
        else avgBits = 0b000;                                // 1 sample (no averaging)
        // FIFO_ROLLOVER_EN: Bit 4 (1 = Rollover enabled)
        // FIFO_A_FULL: Bits 3:0 (Number of samples remaining to trigger interrupt, 0x0 to 0xF)
        // Ví dụ: 0x0F -> A_FULL = 15, ngắt khi còn 15 chỗ trống (17 mẫu đã vào)
        //        0x00 -> A_FULL = 0, ngắt khi FIFO đầy (32 mẫu đã vào)
        uint8_t fifoConfigValue = (avgBits << 5) | (1 << 4) | 0x00; // Rollover On, A_FULL khi đầy (32 mẫu)
        writeRegister8_raw(MAX_REG_FIFO_CONFIG, fifoConfigValue);

        // Mode Configuration (SHDN | RESET | LED_MODE[2:0])
        // ledModeValue: 0x02=HR only (Red LED active), 0x03=SpO2 mode (Red + IR active)
        //               0x07=Multi-LED mode (Red + IR + Green active)
        writeRegister8_raw(MAX_REG_MODE_CONFIG, ledModeValue);

        // SpO2 Configuration (SPO2_ADC_RGE[1:0] | SPO2_SR[2:0] | LED_PW[1:0])
        uint8_t spo2AdcRangeBits = 0;
        if (adcRangeNA <= 2048) spo2AdcRangeBits = 0x00;       // 7.81 pA/LSB
        else if (adcRangeNA <= 4096) spo2AdcRangeBits = 0x20;  // 15.63 pA/LSB
        else if (adcRangeNA <= 8192) spo2AdcRangeBits = 0x40;  // 31.25 pA/LSB
        else spo2AdcRangeBits = 0x60;                          // 16384nA, 62.5 pA/LSB

        uint8_t spo2SampleRateBits = 0;
        if (sampleRateHz <= 50) spo2SampleRateBits = 0x00;         // 50 SPS
        else if (sampleRateHz <= 100) spo2SampleRateBits = 0x04;   // 100 SPS
        else if (sampleRateHz <= 200) spo2SampleRateBits = 0x08;   // 200 SPS
        else if (sampleRateHz <= 400) spo2SampleRateBits = 0x0C;   // 400 SPS
        else if (sampleRateHz <= 800) spo2SampleRateBits = 0x10;   // 800 SPS
        else if (sampleRateHz <= 1000) spo2SampleRateBits = 0x14;  // 1000 SPS
        else if (sampleRateHz <= 1600) spo2SampleRateBits = 0x18;  // 1600 SPS
        else spo2SampleRateBits = 0x1C;                            // 3200 SPS

        uint8_t pulseWidthBits = 0;
        if (pulseWidthUs <= 69) pulseWidthBits = 0x00;      // 69us, 15-bit ADC resolution
        else if (pulseWidthUs <= 118) pulseWidthBits = 0x01; // 118us, 16-bit
        else if (pulseWidthUs <= 215) pulseWidthBits = 0x02; // 215us, 17-bit
        else pulseWidthBits = 0x03;                          // 411us, 18-bit
        writeRegister8_raw(MAX_REG_SPO2_CONFIG, spo2AdcRangeBits | spo2SampleRateBits | pulseWidthBits);

        // LED Pulse Amplitude Configuration
        writeRegister8_raw(MAX_REG_LED1_PA, ledBrightnessIR);   // LED1 (Thường là IR)
        writeRegister8_raw(MAX_REG_LED2_PA, ledBrightnessRed);  // LED2 (Thường là Red)
        // writeRegister8_raw(MAX_REG_LED3_PA, ledBrightnessGreen); // Nếu có LED3 (Green)

        // Multi-LED Mode Slot Configuration (nếu ledModeValue là 0x07)
        // SLOT1, SLOT2
        writeRegister8_raw(MAX_REG_MULTI_LED_CTRL1, (slot2ActiveLEDs << 4) | slot1ActiveLEDs);
        // SLOT3, SLOT4
        writeRegister8_raw(MAX_REG_MULTI_LED_CTRL2, (slot4ActiveLEDs << 4) | slot3ActiveLEDs);


        // Clear FIFO pointers
        writeRegister8_raw(MAX_REG_FIFO_WR_PTR, 0x00);
        writeRegister8_raw(MAX_REG_OVF_COUNTER, 0x00);
        writeRegister8_raw(MAX_REG_FIFO_RD_PTR, 0x00);

        xSemaphoreGive(i2cMutex);
        Serial.println("MAX30102_Custom: Device setup complete.");
    } else { Serial.println("MAX30102_Custom: Timeout taking I2C mutex in setupDevice()."); }
}

void MAX30102_Custom::shutDown() {
    if (!_isSensorInitialized) return;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        bitMask_raw(MAX_REG_MODE_CONFIG, 0x7F, 0x80); // Set SHDN bit
        xSemaphoreGive(i2cMutex);
    } else { Serial.println("MAX30102_Custom: Timeout taking I2C mutex in shutDown()."); }
}

void MAX30102_Custom::resume() {
    if (!_isSensorInitialized) return;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        bitMask_raw(MAX_REG_MODE_CONFIG, 0x7F, 0x00); // Clear SHDN bit
        xSemaphoreGive(i2cMutex);
    } else { Serial.println("MAX30102_Custom: Timeout taking I2C mutex in resume()."); }
}

uint8_t MAX30102_Custom::available(void) const {
    int8_t numberOfSamples = fifoBuffer.head - fifoBuffer.tail;
    if (numberOfSamples < 0) numberOfSamples += MAX_DRIVER_LOCAL_FIFO_SIZE;
    return (uint8_t)numberOfSamples;
}

uint32_t MAX30102_Custom::getRed(void) const {
    if (available() == 0) return 0; // Trả về 0 nếu buffer rỗng
    return fifoBuffer.red[fifoBuffer.tail];
}

uint32_t MAX30102_Custom::getIR(void) const {
    if (available() == 0) return 0; // Trả về 0 nếu buffer rỗng
    return fifoBuffer.ir[fifoBuffer.tail];
}

void MAX30102_Custom::nextSample(void) {
    if (available()) {
        fifoBuffer.tail = (fifoBuffer.tail + 1) % MAX_DRIVER_LOCAL_FIFO_SIZE;
    }
}

uint16_t MAX30102_Custom::readFIFO(void) {
    if (!_isSensorInitialized) return 0;

    uint8_t readPointer = 0;
    uint8_t writePointer = 0;
    int samplesInChipFIFO = 0;
    uint16_t samplesSuccessfullyReadToDriver = 0;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        readPointer = readRegister8_raw(MAX_REG_FIFO_RD_PTR);
        writePointer = readRegister8_raw(MAX_REG_FIFO_WR_PTR);

        if (readPointer != writePointer) {
            samplesInChipFIFO = writePointer - readPointer;
            if (samplesInChipFIFO < 0) samplesInChipFIFO += 32; // FIFO của chip MAX30102 là 32 mẫu

            // Đọc từng mẫu một để đơn giản hóa và kiểm soát buffer cục bộ
            for (int i = 0; i < samplesInChipFIFO; i++) {
                // Kiểm tra buffer cục bộ có đầy không
                if (((fifoBuffer.head + 1) % MAX_DRIVER_LOCAL_FIFO_SIZE) == fifoBuffer.tail) {
                    // Buffer đầy, bỏ qua mẫu cũ nhất trong buffer cục bộ
                    fifoBuffer.tail = (fifoBuffer.tail + 1) % MAX_DRIVER_LOCAL_FIFO_SIZE;
                    Serial.println("MAX30102_Custom: Driver FIFO buffer overflow!");
                }

                uint8_t temp[6]; // 3 byte cho IR, 3 byte cho Red
                _wireInstance->beginTransmission(_i2cAddressSensor);
                _wireInstance->write(MAX_REG_FIFO_DATA);
                if (_wireInstance->endTransmission(false) != 0) { // Giữ kết nối
                    Serial.println("MAX30102_Custom: Error ending Tx before FIFO read.");
                    break; // Lỗi, thoát vòng lặp
                }

                if (_wireInstance->requestFrom(_i2cAddressSensor, (uint8_t)6) == 6) {
                    for(int j=0; j<6; j++) temp[j] = _wireInstance->read();

                    fifoBuffer.head = (fifoBuffer.head + 1) % MAX_DRIVER_LOCAL_FIFO_SIZE;
                    // Theo datasheet, nếu MODE là SpO2 (Red và IR):
                    // FIFO sẽ chứa [IR_MSB, IR_MID, IR_LSB, RED_MSB, RED_MID, RED_LSB] cho mỗi mẫu
                    // Mỗi giá trị là 18-bit
                    fifoBuffer.ir[fifoBuffer.head]  = ((uint32_t)temp[0] << 16 | (uint32_t)temp[1] << 8 | (uint32_t)temp[2]) & 0x03FFFF;
                    fifoBuffer.red[fifoBuffer.head] = ((uint32_t)temp[3] << 16 | (uint32_t)temp[4] << 8 | (uint32_t)temp[5]) & 0x03FFFF;
                    samplesSuccessfullyReadToDriver++;
                } else {
                    Serial.println("MAX30102_Custom: Failed to read 6 bytes from FIFO.");
                    break; // Lỗi đọc, thoát
                }
            } // Kết thúc for loop đọc từng mẫu
        } // Kết thúc if (readPointer != writePointer)
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("MAX30102_Custom: Timeout taking I2C mutex in readFIFO().");
    }
    return samplesSuccessfullyReadToDriver;
}

// --- Hàm đọc/ghi thanh ghi private (raw, không có mutex riêng) ---
uint8_t MAX30102_Custom::readRegister8_raw(uint8_t regAddress) {
    _wireInstance->beginTransmission(_i2cAddressSensor);
    _wireInstance->write(regAddress);
    if (_wireInstance->endTransmission(false) != 0) {
        Serial.printf("MAX30102_Custom (raw): NACK on reg write 0x%02X\n", regAddress);
        return 0;
    }
    if (_wireInstance->requestFrom(_i2cAddressSensor, (uint8_t)1) != 1) {
        Serial.printf("MAX30102_Custom (raw): No data on reg read 0x%02X\n", regAddress);
        return 0;
    }
    return _wireInstance->read();
}

void MAX30102_Custom::writeRegister8_raw(uint8_t regAddress, uint8_t value) {
    _wireInstance->beginTransmission(_i2cAddressSensor);
    _wireInstance->write(regAddress);
    _wireInstance->write(value);
    if (_wireInstance->endTransmission() != 0) {
        Serial.printf("MAX30102_Custom (raw): NACK on reg write 0x%02X with value 0x%02X\n", regAddress, value);
    }
}

void MAX30102_Custom::bitMask_raw(uint8_t regAddress, uint8_t maskToClearComplement, uint8_t bitsToSet) {
    uint8_t originalContents = readRegister8_raw(regAddress);
    originalContents = originalContents & maskToClearComplement;
    writeRegister8_raw(regAddress, originalContents | bitsToSet);
}

void MAX30102_Custom::printRevisionID() {
    if (!_isSensorInitialized) { Serial.println("MAX30102 not initialized, cannot read ID."); return; }
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        uint8_t revID = readRegister8_raw(MAX_REG_REV_ID);
        uint8_t partID = readRegister8_raw(MAX_REG_PART_ID);
        Serial.print("MAX30102_Custom - Revision ID: 0x"); Serial.print(revID, HEX);
        Serial.print(", Part ID: 0x"); Serial.println(partID, HEX);
        xSemaphoreGive(i2cMutex);
     } else {Serial.println("MAX30102_Custom: Timeout (printRevisionID)");}
}