#ifndef FLOW_FILTER_H
#define FLOW_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"

    typedef struct
    {
        float x;  // Vị trí X (mét)
        float vx; // Vận tốc X (m/s)
        float y;  // Vị trí Y (mét)
        float vy; // Vận tốc Y (m/s)
    } pos2d_estimate_t;

    /**
     * @brief Khởi tạo bộ lọc 2D Position/Velocity EKF
     */
    esp_err_t flow_filter_init(void);

    /**
     * @brief Bước Predict EKF 2D theo gia tốc ngang hệ Trái Đất (100Hz)
     * @param ax_world Gia tốc trục X trong hệ Trái Đất (m/s^2)
     * @param ay_world Gia tốc trục Y trong hệ Trái Đất (m/s^2)
     * @param dt Thời gian chu kỳ (s)
     */
    void flow_filter_predict(float ax_world, float ay_world, float dt);

    /**
     * @brief Bước Update EKF 2D khi có mẫu Optical Flow đã De-rotate (20Hz - 50Hz)
     * @param vx_flow Vận tốc X tính từ Optical Flow (m/s)
     * @param vy_flow Vận tốc Y tính từ Optical Flow (m/s)
     * @param qual Chất lượng điểm ảnh (squal từ PMW3901)
     */
    void flow_filter_update(float vx_flow, float vy_flow, uint8_t qual);

    /**
     * @brief Lấy ước lượng vị trí và vận tốc 2D hiện tại
     */
    void flow_filter_get_estimate(pos2d_estimate_t *est);
    void flow_filter_reset(void);
    void flow_filter_zero_velocity(void);
#ifdef __cplusplus
}
#endif

#endif // FLOW_FILTER_H