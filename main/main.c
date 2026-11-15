#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver_mpu6500_basic.h"

#define RAD_TO_DEG (180.0f / M_PI)
#define ALPHA 0.96f /* Hệ số lọc bù (96% Gyro, 4% Accel) */
#define DT 0.01f    /* Chu kỳ lấy mẫu 10ms = 0.01s (100Hz) */

static const char *TAG = "main";

/* Cấu trúc lưu giá trị Offset Gyro */
typedef struct
{
    float x;
    float y;
    float z;
    bool is_calibrated;
} gyro_bias_t;

static gyro_bias_t s_gyro_bias = {0.0f, 0.0f, 0.0f, false};

/* Cấu trúc lưu góc Euler (Roll, Pitch, Yaw) */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} euler_angles_t;

static euler_angles_t s_angles = {0.0f, 0.0f, 0.0f};

/**
 * @brief  Hiệu chỉnh Gyroscope tĩnh lúc vừa bật nguồn (1000 mẫu)
 */
uint8_t mpu6500_gyro_calibrate(uint16_t sample_count)
{
    float accel_g[3];
    float gyro_dps[3];
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint16_t valid_samples = 0;

    ESP_LOGI(TAG, "=== Starting Gyro Calibration ===");
    vTaskDelay(pdMS_TO_TICKS(500));

    for (uint16_t i = 0; i < sample_count; i++)
    {
        if (mpu6500_basic_read(accel_g, gyro_dps) == 0)
        {
            sum_x += gyro_dps[0];
            sum_y += gyro_dps[1];
            sum_z += gyro_dps[2];
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (valid_samples < (sample_count * 0.8f))
    {
        ESP_LOGE(TAG, "Gyro calibration failed!");
        s_gyro_bias.is_calibrated = false;
        return 1;
    }

    s_gyro_bias.x = (float)(sum_x / valid_samples);
    s_gyro_bias.y = (float)(sum_y / valid_samples);
    s_gyro_bias.z = (float)(sum_z / valid_samples);
    s_gyro_bias.is_calibrated = true;

    ESP_LOGI(TAG, "Gyro Bias -> X: %.4f, Y: %.4f, Z: %.4f",
             s_gyro_bias.x, s_gyro_bias.y, s_gyro_bias.z);

    return 0;
}

/**
 * @brief Task đọc dữ liệu IMU & tính toán góc Roll, Pitch, Yaw qua Complementary Filter
 */
void mpu6500_read_task(void *pvParameters)
{
    float accel[3]; /* Ax, Ay, Az (g) */
    float gyro[3];  /* Gx, Gy, Gz (dps) */
    uint8_t res;

    /* 1. Khởi tạo MPU6500 SPI */
    res = mpu6500_basic_init(MPU6500_INTERFACE_SPI, MPU6500_ADDRESS_AD0_LOW);
    if (res != 0)
    {
        ESP_LOGE(TAG, "MPU6500 SPI init failed!");
        vTaskDelete(NULL);
        return;
    }

    /* 2. Hiệu chỉnh Gyro lúc boot */
    if (mpu6500_gyro_calibrate(1000) != 0)
    {
        vTaskDelete(NULL);
        return;
    }

    /* Đọc mẫu ban đầu để thiết lập góc khởi tạo từ Accel */
    if (mpu6500_basic_read(accel, gyro) == 0)
    {
        s_angles.roll = atan2f(accel[1], accel[2]) * RAD_TO_DEG;
        s_angles.pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) * RAD_TO_DEG;
        s_angles.yaw = 0.0f;
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* 3. Vòng lặp tính toán góc (Chu kỳ 10ms = 100Hz) */
    while (1)
    {
        res = mpu6500_basic_read(accel, gyro);
        if (res == 0)
        {
            /* Trừ offset cho Gyro */
            float gx = gyro[0] - s_gyro_bias.x;
            float gy = gyro[1] - s_gyro_bias.y;
            float gz = gyro[2] - s_gyro_bias.z;

            /* Tính Roll và Pitch từ Accelerometer */
            float accel_roll = atan2f(accel[1], accel[2]) * RAD_TO_DEG;
            float accel_pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) * RAD_TO_DEG;

            /* Bộ lọc bù (Complementary Filter) cho Roll & Pitch */
            s_angles.roll = ALPHA * (s_angles.roll + gx * DT) + (1.0f - ALPHA) * accel_roll;
            s_angles.pitch = ALPHA * (s_angles.pitch + gy * DT) + (1.0f - ALPHA) * accel_pitch;

            /* Yaw chỉ tích phân từ Gyro Z (MPU6500 không có Từ kế Magnetometer) */
            s_angles.yaw += gz * DT;
        }

        /* Đảm bảo vòng lặp chạy đúng định thời 10ms (100Hz) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }

    mpu6500_basic_deinit();
    vTaskDelete(NULL);
}

/**
 * @brief Task xuất chuỗi dữ liệu Roll, Pitch, Yaw định dạng CSV qua Serial
 */
void serial_output_task(void *pvParameters)
{
    /* Đợi 3 giây để quá trình Calibrate hoàn tất */
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1)
    {
        /* In dữ liệu Roll, Pitch, Yaw ra Serial cho script Python */
        printf("%.2f,%.2f,%.2f\n", s_angles.roll, s_angles.pitch, s_angles.yaw);

        /* Gửi với tần số 50Hz (20ms) */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    xTaskCreate(mpu6500_read_task, "mpu6500_read_task", 4096, NULL, 10, NULL);
    xTaskCreate(serial_output_task, "serial_output_task", 2048, NULL, 5, NULL);
}