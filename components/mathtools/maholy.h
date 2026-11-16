#ifndef MAHONY_H
#define MAHONY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Hệ số cấu hình mặc định bộ lọc Mahony
 */
#define MAHONY_DEFAULT_KP 2.0f   // Hệ số P (Tín nhiệm Accel)
#define MAHONY_DEFAULT_KI 0.005f // Hệ số I (Bù Gyro Drift)

    /**
     * @brief Cấu trúc dữ liệu lưu trữ trạng thái tư thế và gia tốc Trái Đất
     */
    typedef struct
    {
        float roll;     // Góc Roll (rad)
        float pitch;    // Góc Pitch (rad)
        float yaw;      // Góc Yaw (rad, trôi tự do ở hệ 6-DOF)
        float az_world; // Gia tốc trục Z hệ quy chiếu Trái Đất (m/s^2) đã trừ 1G (Dùng cho Bước 3 EKF)
    } ahrs_state_t;

    /**
     * @brief Khởi tạo/Đặt lại trạng thái bộ lọc Mahony (Quaternion = [1, 0, 0, 0], xóa tích phân lỗi)
     */
    void mahony_init(void);

    /**
     * @brief Đặt lại bộ tích phân lỗi tích lũy (dùng khi ngắt/reset thuật toán)
     */
    void mahony_reset_integral(void);

    /**
     * @brief Cập nhật trạng thái AHRS bằng bộ lọc Mahony 6-DOF
     *
     * @param gx Tốc độ góc trục X (rad/s)
     * @param gy Tốc độ góc trục Y (rad/s)
     * @param gz Tốc độ góc trục Z (rad/s)
     * @param ax Gia tốc trục X (g hoặc m/s^2, thuật toán tự chuẩn hóa)
     * @param ay Gia tốc trục Y (g hoặc m/s^2, thuật toán tự chuẩn hóa)
     * @param az Gia tốc trục Z (g hoặc m/s^2, thuật toán tự chuẩn hóa)
     * @param dt Khoảng thời gian lấy mẫu (s) - Ví dụ: 0.002f tương ứng 500Hz
     * @param output Con trỏ ghi kết quả góc và az_world
     */
    void mahony_ahrs_update_6dof(float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt, ahrs_state_t *output);

    /**
     * @brief Trích xuất Quaternion trạng thái hiện tại [q0, q1, q2, q3]
     */
    void mahony_get_quaternion(float *q0, float *q1, float *q2, float *q3);

#ifdef __cplusplus
}
#endif

#endif // MAHONY_H