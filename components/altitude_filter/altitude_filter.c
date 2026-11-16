#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "ekf.h" // Header module Kalman của esp-dsp

static const char *TAG = "ALTITUDE_EKF";

// Cấu hình Kích thước: State = 2 (z, vz), Measurement = 1 (z_tof), Input = 1 (az)
#define N_STATES 2
#define N_MEAS 1
#define N_INPUTS 1

// Đơn vị và Hằng số
#define GRAVITY 9.80665f

// Khai báo đối tượng EKF của esp-dsp
static ekf_t altitude_ekf;

// Bộ nhớ lưu trữ ma trận nội bộ cho EKF (esp-dsp yêu cầu cấp phát bộ nhớ tĩnh hoặc động)
static float H_data[N_MEAS * N_STATES] = {1.0f, 0.0f}; // z_tof chỉ đo z
static float Q_data[N_STATES * N_STATES] = {
    0.001f, 0.0f,
    0.0f, 0.01f};
static float R_data[N_MEAS * N_MEAS] = {0.005f}; // Nhiễu đo lường của VL53L0X (~5mm)

/**
 * @brief Khởi tạo 1D Altitude EKF
 */
void altitude_ekf_init(float initial_altitude)
{
    // 1. Khởi tạo cấu trúc EKF
    ekf_init(&altitude_ekf, N_STATES, N_MEAS);

    // 2. Thiết lập trạng thái ban đầu x = [z_0, 0]^T
    altitude_ekf.x[0] = initial_altitude;
    altitude_ekf.x[1] = 0.0f;

    // 3. Khởi tạo ma trận hiệp phương sai sai số P (Initial Uncertainty)
    altitude_ekf.P[0] = 0.01f; // P[0][0]
    altitude_ekf.P[1] = 0.00f;
    altitude_ekf.P[2] = 0.00f;
    altitude_ekf.P[3] = 0.1f; // P[1][1]

    ESP_LOGI(TAG, "Altitude EKF Vector & Matrix initialized.");
}

/**
 * @brief Bước Predict (Dự báo dựa vào az_world và dt)
 *
 * @param az_world Gia tốc trục Z trong hệ thế giới (m/s^2), đã trừ gravity
 * @param dt Thời gian lấy mẫu (giây)
 */
void altitude_ekf_predict(float az_world, float dt)
{
    float dt2 = 0.5f * dt * dt;

    // 1. Cập nhật phương trình phi tuyến State Prediction: x = F*x + B*u
    altitude_ekf.x[0] += altitude_ekf.x[1] * dt + az_world * dt2;
    altitude_ekf.x[1] += az_world * dt;

    // 2. Cập nhật Ma trận Chuyển trạng thái Jacobi F
    // F = [1, dt]
    //     [0,  1]
    float F[N_STATES * N_STATES] = {
        1.0f, dt,
        0.0f, 1.0f};

    // 3. Dự báo Ma trận Hiệp phương sai P = F*P*F^T + Q
    // esp-dsp hỗ trợ hàm ekf_predict_cov hoặc tự nhân ma trận qua dspm
    ekf_predict_cov(&altitude_ekf, F, Q_data);
}

/**
 * @brief Bước Update (Cập nhật dựa trên khoảng cách đo từ VL53L0X)
 *
 * @param z_meas Khoảng cách đo được từ VL53L0X (mét)
 */
void altitude_ekf_update(float z_meas)
{
    // Hàm h(x) dự đoán giá trị đo lường từ trạng thái hiện tại: z_pred = x[0]
    float z_pred = altitude_ekf.x[0];

    // Innovation (Residual) y = z_meas - h(x)
    float y = z_meas - z_pred;

    // Thực hiện Update bước lọc EKF với ma trận H và nhiễu R
    ekf_update(&altitude_ekf, H_data, &y, R_data);
}

// --- Ví dụ luồng chạy Task chính ---
void altitude_filter_task(void *pvParameters)
{
    float dt = 0.01f; // Loop 100Hz (10ms)
    altitude_ekf_init(0.0f);

    while (1)
    {
        // 1. Đọc IMU và lấy az_world (m/s^2)
        // Giả sử az_world đã lọc tilt compensation từ Quaternion/Euler
        float az_world = 0.12f; // Ví dụ: Drone bắt đầu tăng tốc nhẹ lên trên

        // 2. Thực hiện Predict liên tục theo chu kỳ IMU (100Hz)
        altitude_ekf_predict(az_world, dt);

        // 3. Nếu có dữ liệu mới từ VL53L0X (thường chạy chậm hơn, ví dụ 33Hz ~ 30ms)
        bool tof_data_ready = true;
        if (tof_data_ready)
        {
            float z_tof = 0.45f; // Đọc 45cm từ VL53L0X
            altitude_ekf_update(z_tof);
        }

        // Lấy kết quả Độ cao và Vận tốc đứng đã qua lọc
        float estimated_altitude = altitude_ekf.x[0];
        float estimated_velocity = altitude_ekf.x[1];

        ESP_LOGD(TAG, "Alt: %.3fm | Vz: %.3fm/s", estimated_altitude, estimated_velocity);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}