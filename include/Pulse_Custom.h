// include/Pulse_Custom.h
#ifndef PULSE_CUSTOM_H
#define PULSE_CUSTOM_H

#include "Arduino.h" // Cần cho int16_t, uint8_t, int32_t (mặc dù Arduino.h bao gồm nhiều thứ)
                     // Hoặc dùng <cstdint> cho các kiểu dữ liệu chuẩn

// --- Các hằng số cấu hình cho bộ lọc và phát hiện nhịp ---
// Bạn NÊN di chuyển các #define này vào file Config.h để dễ quản lý và tinh chỉnh
// Nếu để ở đây, chúng sẽ chỉ có phạm vi trong file này và các file include nó.

#ifndef PULSE_DC_FILTER_NSAMPLE
#define PULSE_DC_FILTER_NSAMPLE 24  // Số mẫu cho bộ lọc DC (alpha = 1/N)
                                    // Giá trị lớn hơn -> lọc mạnh hơn, đáp ứng chậm hơn
#endif

#ifndef PULSE_MA_FILTER_NSLOT
#define PULSE_MA_FILTER_NSLOT   4   // Số mẫu cho bộ lọc trung bình trượt làm mịn tín hiệu AC
                                    // Giá trị nhỏ -> ít làm mịn, nhạy hơn với thay đổi nhanh
#endif

#ifndef PULSE_MIN_BEAT_AMPLITUDE
#define PULSE_MIN_BEAT_AMPLITUDE 50 // Ngưỡng biên độ AC tối thiểu của nhịp đập (đơn vị của tín hiệu sau DC filter)
                                    // Cần tinh chỉnh kỹ lưỡng dựa trên tín hiệu thực tế!
#endif

#ifndef PULSE_MAX_BEAT_AMPLITUDE
#define PULSE_MAX_BEAT_AMPLITUDE 2000 // Ngưỡng biên độ AC tối đa (tránh nhiễu lớn)
                                     // Giá trị này cũng cần tinh chỉnh
#endif
// ---------------------------------------------------------


class MAFilter_Custom {
public:
    MAFilter_Custom(); // Constructor để khởi tạo buffer
    int16_t filter(int16_t value); // Áp dụng bộ lọc
private:
    int16_t buffer[PULSE_MA_FILTER_NSLOT]; // Buffer cho các mẫu
    uint8_t nextslot;                      // Vị trí ghi tiếp theo trong buffer
};

class DCFilter_Custom {
public:
    DCFilter_Custom(void); // Constructor để khởi tạo trạng thái
    int16_t filter(int32_t sample); // Áp dụng bộ lọc, trả về tín hiệu AC
    int32_t avgDC() const;          // Trả về giá trị DC ước tính hiện tại
private:
    // Sử dụng bộ lọc IIR bậc 1 để ước tính thành phần DC:
    // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    // Trong đó y[n] là ước tính DC, x[n] là mẫu đầu vào.
    // Tín hiệu AC sẽ là x[n] - y[n].
    float dcEstimate; // Biến lưu trữ giá trị DC ước tính (dùng float để chính xác hơn)
    // Alpha cho bộ lọc DC, tính từ PULSE_DC_FILTER_NSAMPLE
    // Alpha càng nhỏ, bộ lọc càng mạnh (đáp ứng chậm hơn với thay đổi DC)
    static constexpr float dcFilterAlpha = 1.0f / (float)PULSE_DC_FILTER_NSAMPLE;
};

class Pulse_Custom {
public:
    Pulse_Custom(void); // Constructor

    // Áp dụng bộ lọc DC và trả về tín hiệu AC
    int16_t dc_filter(int32_t sample);
    // Áp dụng bộ lọc MA (làm mịn) cho tín hiệu AC
    int16_t ma_filter(int16_t sample);
    // Phát hiện nhịp đập từ tín hiệu AC đã được làm mịn
    bool isBeat(int16_t filteredSignal);
    // Trả về giá trị DC trung bình ước tính từ DCFilter
    int32_t avgDC() const;
    // Trả về biên độ AC trung bình ước tính (sau khi lọc EMA)
    int16_t avgAC() const;

private:
    DCFilter_Custom dcFilter;   // Đối tượng bộ lọc DC
    MAFilter_Custom maFilter;   // Đối tượng bộ lọc MA

    // Biến cho thuật toán phát hiện nhịp (isBeat)
    int16_t cycle_max_amplitude; // Giá trị dương lớn nhất trong nửa chu kỳ hiện tại
    int16_t cycle_min_amplitude; // Giá trị âm nhỏ nhất trong nửa chu kỳ hiện tại
    bool lookingForMax;          // Cờ: true = đang tìm đỉnh (sườn lên), false = đang tìm đáy (sườn xuống)
    int16_t previous_signal;     // Giá trị tín hiệu của mẫu trước đó

    // Biến cho tính toán biên độ AC trung bình (avgAC)
    float acAmplitudeRunningAvg; // Dùng float để tính EMA cho biên độ AC
    // Alpha cho bộ lọc EMA của biên độ AC (ví dụ: 1/4 = 0.25)
    // Giá trị nhỏ hơn -> làm mịn nhiều hơn, đáp ứng chậm hơn
    static constexpr float AC_EMA_ALPHA = 0.25f;
};

#endif // PULSE_CUSTOM_H