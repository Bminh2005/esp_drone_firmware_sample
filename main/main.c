#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

// Sensor Drivers
#include "pmw3901.h"
#include "vl53l0x.h"
#include "driver_mpu6500_basic.h"

static const char *TAG = "MAIN_SYSTEM";

// --- CẤU HÌNH BỘ LỌC BÙ & TẦN SỐ MPU6500 ---
#define ALPHA 0.96f
#define DT 0.01f // 10ms (100Hz)
#define RAD_TO_DEG (180.0f / M_PI)

// --- CẤU HÌNH CHÂN NỐI PHẦN CỨNG (GPIO PINOUT) ---
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define PMW3901_CS_PIN GPIO_NUM_26

// --- STRUCT LƯU TRẠNG THÁI MPU6500 ---
typedef struct
{
    float x;
    float y;
    float z;
    bool is_calibrated;
} gyro_bias_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} euler_angles_t;

static gyro_bias_t s_gyro_bias = {0};
static euler_angles_t s_angles = {0};

// --- HANDLE THIẾT BỊ BUS ---
i2c_master_bus_handle_t i2c_bus_handle = NULL;
i2c_master_dev_handle_t vl53_dev_handle = NULL;
spi_device_handle_t pmw3901_spi_handle = NULL;

static VL53L0xDev vl53_dev;

// --- KHỞI TẠO HARDWARE SPI DÀNH RIÊNG CHO PMW3901 (SPI3_HOST) ---
static void app_spi_init(void)
{
    esp_err_t ret;

    // KÊNH SPI 3 (SPI3_HOST) - DÀNH CHO PMW3901
    spi_bus_config_t buscfg_pmw = {
        .miso_io_num = GPIO_NUM_33,
        .mosi_io_num = GPIO_NUM_32,
        .sclk_io_num = GPIO_NUM_25,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ret = spi_bus_initialize(SPI3_HOST, &buscfg_pmw, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Cấu hình CS điều khiển bằng tay (Software GPIO) cho PMW3901
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PMW3901_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PMW3901_CS_PIN, 1); // Trạng thái nghỉ ban đầu của CS là HIGH

    spi_device_interface_config_t devcfg_pmw = {
        .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz cho PMW3901
        .mode = 3,                         // SPI Mode 3
        .spics_io_num = -1,                // Quản lý CS thủ công qua GPIO
        .queue_size = 7,
    };
    ret = spi_bus_add_device(SPI3_HOST, &devcfg_pmw, &pmw3901_spi_handle);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Khoi tao SPI3_HOST cho PMW3901 thanh cong");

    // Delay ngắn để chân CS và Bus SPI3 ổn định mức điện thế
    vTaskDelay(pdMS_TO_TICKS(50));
}

// --- KHỞI TẠO HARDWARE I2C DÀNH CHO VL53L0X ---
static void init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    // Đăng ký VL53L0X với tốc độ 100kHz (an toàn hơn khi dùng pull-up nội)
    i2c_device_config_t vl53_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_DEFAULT_ADDRESS, // 0x29
        .scl_speed_hz = 100000,                    // 100 kHz
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &vl53_cfg, &vl53_dev_handle));
}

// --- HÀM HIỆU CHỈNH GYRO (OFFSETS) ---
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

// --- TASK ĐỌC VÀ TÍNH TOÁN GÓC IMU (100Hz) ---
void mpu6500_read_task(void *pvParameters)
{
    float accel[3]; /* Ax, Ay, Az (g) */
    float gyro[3];  /* Gx, Gy, Gz (dps) */
    uint8_t res;

    /* 1. Khởi tạo MPU6500 SPI (LibDriver sẽ tự gọi mpu6500_interface_spi_init khởi tạo SPI2_HOST) */
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

            /* Yaw chỉ tích phân từ Gyro Z */
            s_angles.yaw += gz * DT;
        }

        /* Đảm bảo vòng lặp chạy đúng định thời 10ms (100Hz) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }

    mpu6500_basic_deinit();
    vTaskDelete(NULL);
}

// --- TASK ĐỌC PHỤ & MONITOR CẢM BIẾN (PMW3901 & VL53L0X) ---
void sensor_monitor_task(void *pvParameters)
{
    motionBurst_t pmw_motion = {0};
    uint16_t distance_mm = 0;

    vl53l0xStartContinuous(&vl53_dev, 0);

    while (1)
    {
        // Đọc PMW3901 Optical Flow
        pmw3901ReadMotion(PMW3901_CS_PIN, &pmw_motion);

        // Đọc VL53L0X ToF Distance
        distance_mm = vl53l0xReadRangeContinuousMillimeters(&vl53_dev);

        // In tổng hợp dữ liệu toàn bộ hệ thống cảm biến
        ESP_LOGI(TAG, "ATTITUDE | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f",
                 s_angles.roll, s_angles.pitch, s_angles.yaw);
        ESP_LOGI(TAG, "OPT_FLOW | Motion: %d | DeltaX: %4d | DeltaY: %4d | SQual: %3d",
                 pmw_motion.motionOccured,
                 pmw_motion.deltaX,
                 pmw_motion.deltaY,
                 pmw_motion.squal);
        ESP_LOGI(TAG, "ALTITUDE | Distance: %4d mm", distance_mm);
        printf("--------------------------------------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(50)); // In dữ liệu ở chu kỳ 20Hz (50ms)
    }
}

// --- MAIN ENTRY POINT ---
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing System Hardware...");

    // 1. Khởi tạo Buses
    app_spi_init();
    init_i2c_bus();

    // 2. Chờ 50ms cho nguồn cảm biến ổn định hẳn
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Khởi tạo PMW3901
    if (pmw3901Init(PMW3901_CS_PIN))
    {
        ESP_LOGI(TAG, "PMW3901 SPI Init OK!");
    }
    else
    {
        ESP_LOGE(TAG, "PMW3901 SPI Init FAIL!");
    }

    // 4. Khởi tạo VL53L0X
    if (vl53l0xInit(&vl53_dev, vl53_dev_handle, true))
    {
        ESP_LOGI(TAG, "VL53L0X I2C Init OK!");
    }
    else
    {
        ESP_LOGE(TAG, "VL53L0X I2C Init FAIL!");
    }

    // 5. Tạo Tasks
    xTaskCreatePinnedToCore(mpu6500_read_task, "mpu6500_task", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(sensor_monitor_task, "monitor_task", 4096, NULL, 5, NULL, 0);
}