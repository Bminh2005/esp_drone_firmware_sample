#ifndef ALTITUDE_FILTER_H
#define ALTITUDE_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "esp_err.h"

    /**
     * @brief Cấu hình ma trận nhiễu cho bộ lọc EKF Altitude (Q, R)
     */
    typedef struct
    {
        float q_accel;   // Nhiễu quá trình của Gia tốc (Process Noise Q cho az_world) [m^2/s^4]
        float r_tof;     // Nhiễu đo lường của cảm biến VL53L0X (Measurement Noise R) [m^2]
        float p_init_z;  // Hiệp phương sai sai số ban đầu cho z
        float p_init_vz; // Hiệp phương sai sai số ban đầu cho vz
    } altitude_filter_config_t;

    /**
     * @brief Cấu trúc chứa kết quả ước lượng trạng thái độ cao
     */
    typedef struct
    {
        float z;  // Cao độ ước lượng (mét)
        float vz; // Vận tốc đứng ước lượng (m/s)
    } altitude_estimate_t;

    /**
     * @brief Khởi tạo bộ lọc độ cao với thông số nhiễu mặc định
     *
     * @param initial_z Cao độ ban đầu (thường là 0.0f mét)
     * @return esp_err_t ESP_OK nếu khởi tạo bộ nhớ EKF thành công
     */
    esp_err_t altitude_filter_init(float initial_z);

    /**
     * @brief Khởi tạo bộ lọc độ cao với cấu hình nhiễu tùy chỉnh (Q, R)
     *
     * @param initial_z Cao độ ban đầu (mét)
     * @param config Con trỏ tới struct chứa các tham số Q, R, P
     * @return esp_err_t ESP_OK nếu thành công
     */
    esp_err_t altitude_filter_init_with_config(float initial_z, const altitude_filter_config_t *config);

    /**
     * @brief Bước Predict (Dự báo): Gọi ở chu kỳ IMU cố định (VD: 100Hz trong monitor_task)
     *
     * @param az_world Gia tốc đứng trong hệ quy chiếu Trái Đất (m/s^2), đã trừ gia tốc trọng trường g
     * @param dt Thời gian chu kỳ lấy mẫu (giây, VD: 0.01f cho 100Hz)
     */
    void altitude_filter_predict(float az_world, float dt);

    /**
     * @brief Bước Update (Cập nhật): Gọi mỗi khi nhận được dữ liệu ToF hợp lệ từ VL53L0X
     *
     * @param z_meas Khoảng cách từ VL53L0X đã bù góc nghiêng Roll/Pitch (mét)
     */
    void altitude_filter_update(float z_meas);

    /**
     * @brief Lấy cao độ ước lượng z hiện tại
     * @return float Cao độ (mét)
     */
    float altitude_filter_get_z(void);

    /**
     * @brief Lấy vận tốc đứng ước lượng vz hiện tại
     * @return float Vận tốc đứng (m/s)
     */
    float altitude_filter_get_vz(void);

    /**
     * @brief Lấy đồng thời cả cao độ z và vận tốc đứng vz
     * @param estimate Con trỏ lưu struct độ cao & vận tốc kết quả
     */
    void altitude_filter_get_estimate(altitude_estimate_t *estimate);

    /**
     * @brief Reset lại trạng thái bộ lọc EKF (khi bắt đầu lượt bay mới hoặc re-arm)
     * @param z_reset Cao độ mốc thiết lập lại (mét)
     */
    void altitude_filter_reset(float z_reset);

#ifdef __cplusplus
}
#endif

#endif // ALTITUDE_FILTER_H