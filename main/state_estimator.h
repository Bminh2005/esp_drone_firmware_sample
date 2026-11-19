#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // --- STRUCT CHỨA TOÀN BỘ TRẠNG THÁI UAV (DÙNG CHO PID & LOGGING) ---
    typedef struct
    {
        // 1. Tư thế Attitude (Radian)
        float roll;  // Roll (rad)
        float pitch; // Pitch (rad)
        float yaw;   // Yaw (rad)

        // 2. Tư thế Attitude (Degree) - Dùng cho telemetry & visualizer
        float roll_deg;  // Roll (độ)
        float pitch_deg; // Pitch (độ)
        float yaw_deg;   // Yaw (độ)

        // 3. Tốc độ góc Body Frame (rad/s) - BẮT BUỘC CHO VÒNG LẶP RATE PID
        float gyro_x; // Roll rate (rad/s)
        float gyro_y; // Pitch rate (rad/s)
        float gyro_z; // Yaw rate (rad/s)

        // 4. Cao độ EKF 1D & ToF
        float z;          // Cao độ thực tế (m)
        float vz;         // Vận tốc đứng (m/s)
        uint16_t dist_mm; // Khoảng cách thô ToF (mm)

        // 5. Vị trí & Vận tốc EKF 2D (Optical Flow)
        float x;       // Tọa độ X (m)
        float y;       // Tọa độ Y (m)
        float vx;      // Vận tốc ngang Vx (m/s)
        float vy;      // Vận tốc ngang Vy (m/s)
        uint8_t squal; // Chất lượng điểm ảnh Optical Flow (0 - 255)

        // 6. Trạng thái hệ thống
        bool is_initialized;
    } uav_state_t;

    /**
     * @brief Khởi tạo toàn bộ Bus (SPI, I2C), cảm biến và kích hoạt các FreeRTOS Task EKF.
     * @return esp_err_t ESP_OK nếu thành công.
     */
    esp_err_t state_estimator_init(void);

    /**
     * @brief Trích xuất trạng thái ước lượng mới nhất của UAV (Thread-safe qua Mutex).
     * @param out_state Con trỏ trỏ tới struct uav_state_t nhận dữ liệu.
     * @return true nếu lấy dữ liệu thành công.
     */
    bool state_estimator_get_state(uav_state_t *out_state);

    esp_err_t state_estimator_calibrate_gyro(void);
    /**
     * @brief In chuỗi Telemetry định dạng Visualizer ra UART.
     * @param state Con trỏ dữ liệu state.
     */
    void state_estimator_stream_telemetry(const uav_state_t *state);

#ifdef __cplusplus
}
#endif

#endif // STATE_ESTIMATOR_H