#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

// Sensor Drivers, Math Tools & Altitude EKF Filter
#include "pmw3901.h"
#include "vl53l0x.h"
#include "driver_mpu6500_basic.h"
#include "maholy.h"
#include "altitude_filter.h"
#include "flow_filter.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
#endif

static const char *TAG_CALIB = "IMU_CALIB_180";
static const char *TAG_LOGGER = "IMU_LOGGER";

typedef struct
{
    float ax_offset; // Bias gia tốc X (g)
    float ay_offset; // Bias gia tốc Y (g)
    float az_offset; // Bias gia tốc Z (g)
    float gx_offset; // Bias con quay Gyro X (rad/s)
    float gy_offset; // Bias con quay Gyro Y (rad/s)
    float gz_offset; // Bias con quay Gyro Z (rad/s)
} imu_bias_t;

// Biến lưu Bias toàn cục
static imu_bias_t g_imu_bias = {0};

/**
 * @brief Hàm Calibrate IMU 180 độ dùng driver mpu6500_basic
 */
void calibrate_imu_tilted_table(imu_bias_t *out_bias)
{
    const int num_samples = 200;
    float accel[3], gyro[3];
    float ax1 = 0, ay1 = 0, az1 = 0;
    float ax2 = 0, ay2 = 0, az2 = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    ESP_LOGW(TAG_CALIB, "==================================================");
    ESP_LOGW(TAG_CALIB, "BẮT ĐẦU HIỆU CHỈNH IMU TRÊN BÀN NGHIÊNG (180 DEG)");
    ESP_LOGW(TAG_CALIB, "==================================================");

    // --- BƯỚC 1: ĐO HƯỚNG 0 ĐỘ ---
    ESP_LOGI(TAG_CALIB, "[BƯỚC 1/2] Đặt Drone cố định HƯỚNG VỀ PHÍA TRƯỚC (0 deg).");
    ESP_LOGI(TAG_CALIB, "Bắt đầu đo sau 3 giây... Hãy giữ yên drone!");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG_CALIB, "Đang thu thập dữ liệu Hướng 1...");
    for (int i = 0; i < num_samples; i++)
    {
        if (mpu6500_basic_read(accel, gyro) == 0)
        {
            ax1 += accel[0]; // Đơn vị: g
            ay1 += accel[1];
            az1 += accel[2];

            // Quy đổi Gyro dps (deg/s) -> rad/s
            gx_sum += gyro[0] * DEG_TO_RAD;
            gy_sum += gyro[1] * DEG_TO_RAD;
            gz_sum += gyro[2] * DEG_TO_RAD;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ax1 /= num_samples;
    ay1 /= num_samples;
    az1 /= num_samples;

    // --- BƯỚC 2: XOAY 180 ĐỘ ---
    ESP_LOGW(TAG_CALIB, "--------------------------------------------------");
    ESP_LOGW(TAG_CALIB, "[BƯỚC 2/2] CHỜ 7 GIÂY: Hãy xoay drone 180 DEG!");
    ESP_LOGW(TAG_CALIB, "Giữ ĐÚNG VỊ TRÍ CŨ trên bàn, chỉ quay ngược đuôi lại!");
    ESP_LOGW(TAG_CALIB, "--------------------------------------------------");

    for (int countdown = 12; countdown > 0; countdown--)
    {
        ESP_LOGI(TAG_CALIB, "Đếm ngược: %d giây...", countdown);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG_CALIB, "Đang thu thập dữ liệu Hướng 2 (180 deg)... Hãy giữ yên!");
    for (int i = 0; i < num_samples; i++)
    {
        if (mpu6500_basic_read(accel, gyro) == 0)
        {
            ax2 += accel[0];
            ay2 += accel[1];
            az2 += accel[2];

            gx_sum += gyro[0] * DEG_TO_RAD;
            gy_sum += gyro[1] * DEG_TO_RAD;
            gz_sum += gyro[2] * DEG_TO_RAD;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ax2 /= num_samples;
    ay2 /= num_samples;
    az2 /= num_samples;
    ESP_LOGI(TAG_CALIB, "(%+.4f, %+.4f, %+.4f) g && (%+.4f, %+.4f, %+.4f) g", ax1, ay1, az1, ax2, ay2, az2);
    // --- BƯỚC 3: TÍNH TOÁN BIAS TRIỆT TIÊU GÓC NGHIÊNG BÀN ---
    out_bias->ax_offset = (ax1 + ax2) / 2.0f;
    out_bias->ay_offset = (ay1 + ay2) / 2.0f;
    out_bias->az_offset = ((az1 + az2) / 2.0f) - 1.0f; // Trục Z chuẩn 1.0g

    out_bias->gx_offset = gx_sum / (num_samples * 2);
    out_bias->gy_offset = gy_sum / (num_samples * 2);
    out_bias->gz_offset = gz_sum / (num_samples * 2);

    ESP_LOGW(TAG_CALIB, "==================================================");
    ESP_LOGI(TAG_CALIB, "CALIBRATE THÀNH CÔNG! KẾT QUẢ BIAS NỘI TẠI:");
    ESP_LOGI(TAG_CALIB, "Accel Offsets (g)    : Ax = %+.4f | Ay = %+.4f | Az = %+.4f",
             out_bias->ax_offset, out_bias->ay_offset, out_bias->az_offset);
    ESP_LOGI(TAG_CALIB, "Gyro Offsets (rad/s) : Gx = %+.4f | Gy = %+.4f | Gz = %+.4f",
             out_bias->gx_offset, out_bias->gy_offset, out_bias->gz_offset);
    ESP_LOGW(TAG_CALIB, "==================================================");
}

/**
 * @brief Task in dữ liệu RAW và CALIB của IMU 20 lần/giây (20Hz)
 */
void imu_logger_task(void *pvParameters)
{
    // Đợi 4 giây đầu để hoàn tất quá trình Calibrate trước khi logger chạy
    vTaskDelay(pdMS_TO_TICKS(4000));

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 500ms = 2Hz

    float accel[3];
    float gyro[3];
    uint8_t res;
    res = mpu6500_basic_init(MPU6500_INTERFACE_SPI, MPU6500_ADDRESS_AD0_LOW);
    if (res != 0)
    {
        ESP_LOGE(TAG_LOGGER, "MPU6500 SPI init failed!");
        vTaskDelete(NULL);
        return;
    }
    calibrate_imu_tilted_table(&g_imu_bias);

    ESP_LOGI(TAG_LOGGER, "Bắt đầu task IMU Logger (20Hz)...");

    while (1)
    {
        if (mpu6500_basic_read(accel, gyro) == 0)
        {
            // 1. GIÁ TRỊ GỐC (RAW) TỪ DRIVER (accel: g, gyro: deg/s)
            float ax_raw_g = accel[0];
            float ay_raw_g = accel[1];
            float az_raw_g = accel[2];

            float gx_raw_deg = gyro[0];
            float gy_raw_deg = gyro[1];
            float gz_raw_deg = gyro[2];

            // 2. GIÁ TRỊ SAU HIỆU CHỈNH (CALIBRATED)
            float ax_cal_g = ax_raw_g - g_imu_bias.ax_offset;
            float ay_cal_g = ay_raw_g - g_imu_bias.ay_offset;
            float az_cal_g = az_raw_g - g_imu_bias.az_offset;

            // Quy đổi Bias (rad/s) về (deg/s) để trừ trực tiếp với gyro_raw (deg/s)
            float gx_cal_deg = gx_raw_deg - (g_imu_bias.gx_offset * RAD_TO_DEG);
            float gy_cal_deg = gy_raw_deg - (g_imu_bias.gy_offset * RAD_TO_DEG);
            float gz_cal_deg = gz_raw_deg - (g_imu_bias.gz_offset * RAD_TO_DEG);

            // 3. IN SO SÁNH TRỰC QUAN LOG
            ESP_LOGI(TAG_LOGGER, "ACC RAW [g]   | X: %6.3f | Y: %6.3f | Z: %6.3f", ax_raw_g, ay_raw_g, az_raw_g);
            ESP_LOGI(TAG_LOGGER, "ACC CAL [g]   | X: %6.3f | Y: %6.3f | Z: %6.3f", ax_cal_g, ay_cal_g, az_cal_g);
            ESP_LOGI(TAG_LOGGER, "GYR RAW [°/s] | X: %6.2f | Y: %6.2f | Z: %6.2f", gx_raw_deg, gy_raw_deg, gz_raw_deg);
            ESP_LOGI(TAG_LOGGER, "GYR CAL [°/s] | X: %6.2f | Y: %6.2f | Z: %6.2f", gx_cal_deg, gy_cal_deg, gz_cal_deg);
            printf("--------------------------------------------------------------------\n");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    // ESP_LOGI(TAG, "Initializing System Hardware...");

    // // 0. Tạo Mutex bảo vệ dữ liệu AHRS giữa 2 Tasks
    // g_ahrs_mutex = xSemaphoreCreateMutex();
    // if (g_ahrs_mutex == NULL)
    // {
    //     ESP_LOGE(TAG, "Khong the tao Mutex AHRS!");
    //     return;
    // }

    // // Khởi tạo 1D Altitude EKF với cao độ mốc ban đầu 0.0m
    // altitude_filter_init(0.0f);

    // // 1. Khởi tạo Buses
    // app_spi_init();

    // // 2. Chờ 50ms cho nguồn cảm biến ổn định hẳn
    // vTaskDelay(pdMS_TO_TICKS(50));

    // // 3. Khởi tạo PMW3901
    // if (pmw3901Init(PMW3901_CS_PIN))
    // {
    //     ESP_LOGI(TAG, "PMW3901 SPI Init OK!");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "PMW3901 SPI Init FAIL!");
    // }

    // // 4. Khởi tạo VL53L0X
    // vl53l0x_hard_reset();
    // init_i2c_bus();
    // if (vl53l0xInit(&vl53_dev, vl53_dev_handle, true))
    // {
    //     ESP_LOGI(TAG, "VL53L0X I2C Init OK!");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "VL53L0X I2C Init FAIL!");
    // }

    // 5. Tạo Tasks
    xTaskCreatePinnedToCore(imu_logger_task, "mpu6500_task", 4096, NULL, 10, NULL, 1);
    // xTaskCreatePinnedToCore(sensor_monitor_task, "monitor_task", 4096, NULL, 5, NULL, 0);
}