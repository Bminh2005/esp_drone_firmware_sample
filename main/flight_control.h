#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "state_estimator.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // --- CHẾ ĐỘ BAY (FLIGHT MODES) ---
    typedef enum
    {
        FLIGHT_MODE_DISARMED = 0, // Khóa động cơ (PWM = 0)
        FLIGHT_MODE_RATE,         // Điều khiển Tốc độ góc (Acro Mode - Vòng trong cùng)
        FLIGHT_MODE_STABILIZE,    // Điều khiển Góc nghiêng (Angle Mode - Cân bằng tự động)
        FLIGHT_MODE_ALT_HOLD,     // Điều khiển Cao độ (Tự giữ Z)
        FLIGHT_MODE_POS_HOLD      // Điều khiển Vị trí 2D (Tự đứng yên 1 chỗ X, Y, Z)
    } flight_mode_t;

    // --- DỮ LIỆU MỤC TIÊU ĐIỀU KHIỂN (SETPOINT) ---
    typedef struct
    {
        float roll_deg;       // Góc Roll mục tiêu (độ)
        float pitch_deg;      // Góc Pitch mục tiêu (độ)
        float yaw_rate_dps;   // Tốc độ xoay Yaw mục tiêu (độ/s)
        float alt_z_m;        // Cao độ mục tiêu (m)
        float pos_x_m;        // Vị trí X mục tiêu (m)
        float pos_y_m;        // Vị trí Y mục tiêu (m)
        float hover_throttle; // Mức Ga lơ lửng tĩnh (0.0 - 1.0, ví dụ 0.45 = 45% PWM)
    } uav_setpoint_t;

    // --- CẤU HÌNH BỘ THAM SỐ PID ---
    typedef struct
    {
        float kp;
        float ki;
        float kd;
        float i_term;
        float i_max;   // Anti-windup limit
        float out_max; // Giới hạn đầu ra cực đại
        float prev_error;
    } pid_controller_t;

    /**
     * @brief Khởi tạo ngoại vi LEDC PWM cho 4 động cơ và reset tham số PID.
     * @return esp_err_t ESP_OK nếu khởi tạo thành công.
     */
    esp_err_t flight_control_init(void);

    /**
     * @brief Chuyển đổi trạng thái ARM / DISARM bảo vệ an toàn.
     */
    void flight_control_arm(void);
    void flight_control_disarm(void);
    bool flight_control_is_armed(void);

    /**
     * @brief Thiết lập Chế độ bay và Điểm mục tiêu (Setpoint).
     */
    void flight_control_set_mode(flight_mode_t mode);
    void flight_control_set_setpoint(const uav_setpoint_t *setpoint);

    /**
     * @brief TÍNH TOÁN VÒNG LẶP PID PHÂN TẦNG (Gọi liên tục ở tần số 100Hz).
     * @param state Trạng thái ước lượng mới nhất từ state_estimator.
     * @param dt Khoảng thời gian thực tế giữa 2 lần tính toán (giây).
     */
    void flight_control_update(const uav_state_t *state, float dt);
    typedef enum
    {
        DISARM_REASON_NONE = 0,
        DISARM_REASON_USER_CMD,
        DISARM_REASON_LOW_BATTERY, // Sụt áp < 3.0V
        DISARM_REASON_INVERTED,    // Nghiêng > 60 độ
        DISARM_REASON_WIFI_LOST    // Mất Wi-Fi Timeout
    } disarm_reason_t;

    float flight_control_get_vbat(void);
    disarm_reason_t flight_control_get_disarm_reason(void);
    const char *flight_control_get_disarm_reason_str(void);
#ifdef __cplusplus
}
#endif

#endif // FLIGHT_CONTROL_H