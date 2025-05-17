// src/Pulse_Custom.cpp
#include "Pulse_Custom.h"
#include <cmath> // Cho abs() và isnan() nếu cần

// --- MAFilter_Custom Implementation ---
MAFilter_Custom::MAFilter_Custom() : nextslot(0) {
    for (int i = 0; i < PULSE_MA_FILTER_NSLOT; ++i) {
        buffer[i] = 0;
    }
}

int16_t MAFilter_Custom::filter(int16_t value) {
    buffer[nextslot] = value;
    nextslot = (nextslot + 1) % PULSE_MA_FILTER_NSLOT;
    int32_t total = 0; // Sử dụng int32_t để tránh tràn số khi cộng
    for (int i = 0; i < PULSE_MA_FILTER_NSLOT; ++i) {
        total += buffer[i];
    }
    return (int16_t)(total / PULSE_MA_FILTER_NSLOT);
}

// --- DCFilter_Custom Implementation ---
DCFilter_Custom::DCFilter_Custom(void) : dcEstimate(0.0f) {
    // Khởi tạo dcEstimate là 0.0f hoặc bằng mẫu đầu tiên (nếu biết trước)
}

// Lọc IIR bậc 1 để ước tính và loại bỏ DC:
// dcEstimate[n] = alpha * sample[n] + (1 - alpha) * dcEstimate[n-1]
// acSignal[n] = sample[n] - dcEstimate[n]
int16_t DCFilter_Custom::filter(int32_t sample) {
    // Cập nhật ước tính DC
    dcEstimate = dcFilterAlpha * (float)sample + (1.0f - dcFilterAlpha) * dcEstimate;
    // Trả về thành phần AC
    return (int16_t)((float)sample - dcEstimate);
}

int32_t DCFilter_Custom::avgDC() const {
    return (int32_t)dcEstimate; // Trả về ước tính DC hiện tại
}


// --- Pulse_Custom Implementation ---
Pulse_Custom::Pulse_Custom(void)
    : cycle_max_amplitude(0),   // Khởi tạo bằng 0 hoặc một giá trị rất nhỏ
      cycle_min_amplitude(0),   // Khởi tạo bằng 0 hoặc một giá trị rất lớn
      lookingForMax(true),     // Bắt đầu bằng việc tìm đỉnh (sườn lên)
      previous_signal(0),
      acAmplitudeRunningAvg(PULSE_MIN_BEAT_AMPLITUDE) // Khởi tạo với giá trị hợp lý
{
    // Không cần làm gì thêm ở đây nếu các đối tượng filter đã có constructor mặc định
}

// Áp dụng bộ lọc DC và trả về tín hiệu AC
int16_t Pulse_Custom::dc_filter(int32_t sample) {
    return dcFilter.filter(sample);
}

// Áp dụng bộ lọc MA (làm mịn) cho tín hiệu AC
int16_t Pulse_Custom::ma_filter(int16_t sample) {
    return maFilter.filter(sample);
}

// Phát hiện nhịp đập từ tín hiệu AC đã được làm mịn
bool Pulse_Custom::isBeat(int16_t filteredSignal) {
    bool beatDetected = false;

    if (lookingForMax) { // Đang ở sườn lên, tìm đỉnh (max)
        if (filteredSignal > previous_signal) {
            cycle_max_amplitude = filteredSignal; // Tín hiệu đang tăng, cập nhật max
        } else if (filteredSignal < previous_signal) {
            // Tín hiệu bắt đầu giảm -> vừa qua đỉnh
            // Biên độ của nửa chu kỳ dương này là cycle_max_amplitude
            // (Vì đáy trước đó có thể là cycle_min_amplitude < 0)
            // Biên độ thực sự của nhịp là (cycle_max_amplitude - cycle_min_amplitude_của_nhịp_trước)
            // Ở đây chúng ta chỉ phát hiện đỉnh, việc tính biên độ chính xác cho SpO2 phức tạp hơn.

            // Kiểm tra xem đỉnh có đủ "nổi bật" không (so với giá trị trung bình của biên độ)
            // và có nằm trong khoảng cho phép không
            // Ngưỡng đơn giản: nếu cycle_max_amplitude đủ lớn
            if (cycle_max_amplitude > (PULSE_MIN_BEAT_AMPLITUDE / 2) && cycle_max_amplitude < (PULSE_MAX_BEAT_AMPLITUDE /2) ) { // Điều chỉnh ngưỡng
                // Để tính biên độ chính xác hơn, chúng ta cần lưu lại cycle_min_amplitude của nhịp trước.
                // Tạm thời, chúng ta giả định biên độ là cycle_max_amplitude - cycle_min_amplitude (của chu kỳ này)
                // hoặc chỉ cần cycle_max_amplitude đủ lớn.
                int currentPulseAmplitude = cycle_max_amplitude - cycle_min_amplitude; // Biên độ của chu kỳ này
                if(currentPulseAmplitude > PULSE_MIN_BEAT_AMPLITUDE && currentPulseAmplitude < PULSE_MAX_BEAT_AMPLITUDE){
                    beatDetected = true;
                    // Cập nhật EMA của biên độ AC
                    acAmplitudeRunningAvg += AC_EMA_ALPHA * ((float)currentPulseAmplitude - acAmplitudeRunningAvg);
                }
            }
            lookingForMax = false; // Chuyển sang tìm đáy
            cycle_min_amplitude = filteredSignal; // Đỉnh vừa rồi là điểm bắt đầu của sườn xuống
        }
    } else { // Đang ở sườn xuống, tìm đáy (min)
        if (filteredSignal < previous_signal) {
            cycle_min_amplitude = filteredSignal; // Tín hiệu đang giảm, cập nhật min
        } else if (filteredSignal > previous_signal) {
            // Tín hiệu bắt đầu tăng -> vừa qua đáy
            // Biên độ của nửa chu kỳ âm này là cycle_min_amplitude
            lookingForMax = true; // Chuyển sang tìm đỉnh
            cycle_max_amplitude = filteredSignal; // Đáy vừa rồi là điểm bắt đầu của sườn lên
        }
    }
    previous_signal = filteredSignal;
    return beatDetected;
}

// Trả về giá trị DC trung bình ước tính từ DCFilter
int32_t Pulse_Custom::avgDC() const {
    return dcFilter.avgDC();
}

// Trả về biên độ AC trung bình ước tính (sau khi lọc EMA)
int16_t Pulse_Custom::avgAC() const {
    return (int16_t)acAmplitudeRunningAvg;
}