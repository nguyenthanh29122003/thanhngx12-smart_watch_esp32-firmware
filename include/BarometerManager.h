// include/BarometerManager.h
#ifndef BAROMETER_MANAGER_H
#define BAROMETER_MANAGER_H

#include <Adafruit_BMP280.h> // Sử dụng thư viện Adafruit BMP280
#include <Adafruit_Sensor.h> // Thư viện Sensor cơ sở của Adafruit
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Địa chỉ I2C mặc định của BMP280 thường là 0x77 hoặc 0x76
// Đảm bảo BMP280_ADDRESS được định nghĩa trong Config.h và đúng với kết quả scan
// #define BMP280_ADDRESS 0x77 // Hoặc 0x76

class BarometerManager {
public:
    BarometerManager();
    bool begin(TwoWire &wireInstance = Wire); // Cho phép chọn bus I2C, mặc định là Wire
    void startTask(UBaseType_t priority = 1); // Ưu tiên thấp hơn cho cảm biến môi trường
    void stopTask();
    // Cung cấp dữ liệu nhiệt độ (Celsius) và áp suất (Pascals)
    void getData(float& temperature, float& pressure); // Sửa kiểu trả về áp suất thành float

private:
    Adafruit_BMP280 bmp; // Đối tượng cảm biến BMP280 từ thư viện Adafruit
    TwoWire *_wire;      // Con trỏ tới đối tượng Wire (I2C bus)

    TaskHandle_t taskHandle;
    SemaphoreHandle_t dataMutex; // Mutex bảo vệ dữ liệu cục bộ

    // Biến lưu trữ dữ liệu cục bộ
    float temperatureLocal; // Nhiệt độ (Celsius)
    float pressureLocal;    // Áp suất (Pascals)
    bool sensorReady;       // Cờ báo cảm biến đã sẵn sàng

    // Hàm task chạy nền
    static void taskFunction(void* pvParameters);
    void updateSensor();    // Hàm đọc dữ liệu trong task
};

#endif // BAROMETER_MANAGER_H