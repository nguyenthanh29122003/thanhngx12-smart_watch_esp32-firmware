// include/Max30102_Custom.h
#ifndef MAX30102_CUSTOM_H
#define MAX30102_CUSTOM_H

#include <Arduino.h>
#include <Wire.h> // Cần Wire.h

// Register addresses (Giữ nguyên từ code gốc của bạn)
#define MAX_REG_INTR_STATUS_1   0x00
#define MAX_REG_INTR_STATUS_2   0x01
#define MAX_REG_INTR_ENABLE_1   0x02
#define MAX_REG_INTR_ENABLE_2   0x03
#define MAX_REG_FIFO_WR_PTR     0x04
#define MAX_REG_OVF_COUNTER     0x05
#define MAX_REG_FIFO_RD_PTR     0x06
#define MAX_REG_FIFO_DATA       0x07
#define MAX_REG_FIFO_CONFIG     0x08
#define MAX_REG_MODE_CONFIG     0x09
#define MAX_REG_SPO2_CONFIG     0x0A
#define MAX_REG_LED1_PA         0x0C // IR
#define MAX_REG_LED2_PA         0x0D // Red
#define MAX_REG_LED3_PA         0x0E // Green (MAX30102 không có, MAX30105 có)
#define MAX_REG_PILOT_PA        0x10
#define MAX_REG_MULTI_LED_CTRL1 0x11 // Slot 1, 2
#define MAX_REG_MULTI_LED_CTRL2 0x12 // Slot 3, 4
#define MAX_REG_TEMP_INTR       0x1F
#define MAX_REG_TEMP_FRAC       0x20
#define MAX_REG_TEMP_CONFIG     0x21
#define MAX_REG_PROX_INT_THRESH 0x30
#define MAX_REG_REV_ID          0xFE
#define MAX_REG_PART_ID         0xFF

#define MAX30102_EXPECTED_PART_ID 0x15 // Part ID cho MAX30102

// Kích thước buffer FIFO cục bộ trong driver.
// FIFO của chip MAX30102 có thể lưu 32 mẫu (mỗi mẫu gồm các kênh LED được kích hoạt)
#define MAX_DRIVER_LOCAL_FIFO_SIZE 32 // Lưu trữ tối đa 32 mẫu (IR và Red)

class MAX30102_Custom {
public:
    MAX30102_Custom(void);

    // Khởi tạo cảm biến với bus I2C và địa chỉ cụ thể.
    // Trả về true nếu thành công, false nếu thất bại.
    bool begin(TwoWire &wireInstance, uint8_t i2cAddress);

    // Cấu hình chi tiết các thông số hoạt động của cảm biến.
    // ledMode: Bit 0=EN_SLOT1_RED, Bit 1=EN_SLOT2_IR (ví dụ)
    // Xem datasheet để biết chi tiết về các slot và ledMode.
    // Với SpO2, thường dùng Red và IR.
    void setupDevice(uint8_t ledBrightnessIR, uint8_t ledBrightnessRed,
                     uint8_t sampleAverage, uint8_t ledModeConfig, // ledModeConfig là giá trị cho REG_MODE_CONFIG
                     uint8_t slot1ActiveLEDs, uint8_t slot2ActiveLEDs, // Giá trị cho REG_MULTI_LED_CTRL1
                     uint8_t slot3ActiveLEDs, uint8_t slot4ActiveLEDs, // Giá trị cho REG_MULTI_LED_CTRL2
                     uint16_t sampleRateHz, uint16_t pulseWidthUs, uint16_t adcRangeNA);

    void shutDown(); // Đưa cảm biến vào chế độ nguồn thấp (SHDN = 1)
    void resume();   // Khởi động lại cảm biến từ chế độ nguồn thấp (SHDN = 0)

    // Đọc dữ liệu từ FIFO của chip vào buffer cục bộ của driver.
    // Trả về số lượng mẫu MỚI (mỗi mẫu gồm IR và Red) đã đọc thành công.
    uint16_t readFIFO(void);

    // Kiểm tra số lượng mẫu có sẵn trong buffer cục bộ của driver.
    uint8_t available(void) const;

    // Di chuyển con trỏ đọc (tail) của buffer cục bộ đến mẫu tiếp theo.
    void nextSample(void);

    // Lấy giá trị Red và IR của mẫu hiện tại (tại vị trí tail).
    uint32_t getRed(void) const; // Giá trị Red 18-bit
    uint32_t getIR(void) const;  // Giá trị IR 18-bit

    // Đọc và in Revision ID và Part ID của chip (hữu ích khi debug).
    void printRevisionID();

private:
    TwoWire* _wireInstance; // Con trỏ tới đối tượng I2C bus (Wire, Wire1, ...)
    uint8_t _i2cAddressSensor;   // Địa chỉ I2C của cảm biến
    bool _isSensorInitialized; // Cờ báo cảm biến đã được khởi tạo thành công

    // Buffer cục bộ để lưu dữ liệu đọc từ FIFO của chip
    struct {
        uint32_t red[MAX_DRIVER_LOCAL_FIFO_SIZE];
        uint32_t ir[MAX_DRIVER_LOCAL_FIFO_SIZE]; // Đổi tên thành ir cho nhất quán
        volatile byte head; // Index để ghi mẫu mới
        volatile byte tail; // Index để đọc mẫu cũ
    } fifoBuffer;

    // Hàm tiện ích đọc/ghi thanh ghi (KHÔNG có mutex, chỉ dùng nội bộ trong các hàm public đã có mutex)
    uint8_t readRegister8_raw(uint8_t regAddress);
    void writeRegister8_raw(uint8_t regAddress, uint8_t value);
    // Hàm tiện ích để set/clear các bit trong một thanh ghi
    void bitMask_raw(uint8_t regAddress, uint8_t maskToClear, uint8_t bitsToSet);
};

#endif // MAX30102_CUSTOM_H