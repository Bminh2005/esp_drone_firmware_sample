#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "state_estimator.h"

static const char *TAG = "MAIN_APP";

void app_main(void)
{
    // 1. Khởi tạo toàn bộ Module Ước lượng Trạng thái
    if (state_estimator_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Sensor system initialization failed! System halted.");
        return;
    }

    uav_state_t uav_state = {0};

    // 2. Vòng lặp chính (Hoặc sau này tạo Task Motor Control riêng)
    while (1)
    {
        // Lấy trạng thái UAV mới nhất
        if (state_estimator_get_state(&uav_state))
        {
            // Stream telemetry ra Web/Serial Visualizer
            state_estimator_stream_telemetry(&uav_state);

            /* -------------------------------------------------------------
             * CHỖ NÀY SẼ LÀ NƠI TRUYỀN DỮ LIỆU VÀO VÒNG LẶP PID ĐỘNG CƠ:
             * - Rate PID     <-- uav_state.gyro_x, gyro_y, gyro_z
             * - Angle PID    <-- uav_state.roll, pitch, yaw
             * - Alt PID      <-- uav_state.z, vz
             * - Pos 2D PID   <-- uav_state.x, y, vx, vy
             * ------------------------------------------------------------- */
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz Stream Log
    }
}