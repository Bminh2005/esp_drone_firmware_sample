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

// Sensor Drivers & Math Tools
#include "pmw3901.h"
#include "vl53l0x.h"
#include "driver_mpu6500_basic.h"
#include "maholy.h"

static const char *TAG = "MAIN_SYSTEM";

// --- CHUYỂN ĐỔI ĐƠN VỊ ---
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

// --- CẤU HÌNH CHÂN NỐI PHẦN CỨNG (GPIO PINOUT) ---
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define PMW3901_CS_PIN GPIO_NUM_26
#define VL53L0X_XSHUT_PIN GPIO_NUM_17

// --- STRUCT LƯU TRẠNG THÁI GYRO BIAS ---
typedef struct
{
    float x;
    float y;
    float z;
    bool is_calibrated;
} gyro_bias_t;

static gyro_bias_t s_gyro_bias = {0};

// --- QUẢN LÝ DỮ LIỆU MAHONY AHRS TOÀN CỤC & MUTEX ---
static ahrs_state_t g_ahrs_state = {0};
static SemaphoreHandle_t g_ahrs_mutex = NULL;

// --- HANDLE THIẾT BỊ BUS ---
i2c_master_bus_handle_t i2c_bus_handle = NULL;
i2c_master_dev_handle_t vl53_dev_handle = NULL;
spi_device_handle_t pmw3901_spi_handle = NULL;

static VL53L0xDev vl53_dev;

// --- KHỞI TẠO HARDWARE SPI DÀNH RIÊNG CHO PMW3901 (SPI3_HOST) ---
static void app_spi_init(void)
{
    esp_err_t ret;

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

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PMW3901_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PMW3901_CS_PIN, 1);

    spi_device_interface_config_t devcfg_pmw = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 7,
    };
    ret = spi_bus_add_device(SPI3_HOST, &devcfg_pmw, &pmw3901_spi_handle);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Khoi tao SPI3_HOST cho PMW3901 thanh cong");

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

    i2c_device_config_t vl53_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_DEFAULT_ADDRESS,
        .scl_speed_hz = 100000,
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

// --- TASK ĐỌC VÀ TÍNH TOÁN GÓC MAHONY AHRS (100Hz) ---
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

    /* 3. Khởi tạo thuật toán Mahony AHRS */
    mahony_init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t last_time = esp_timer_get_time();

    /* 4. Vòng lặp tính toán góc Mahony AHRS (Chu kỳ 10ms = 100Hz) */
    while (1)
    {
        res = mpu6500_basic_read(accel, gyro);
        if (res == 0)
        {
            // Tính khoảng dt thực tế chính xác bằng esp_timer
            int64_t now = esp_timer_get_time();
            float dt = (float)(now - last_time) / 1000000.0f;
            last_time = now;

            if (dt <= 0.0f || dt > 0.1f)
            {
                dt = 0.01f; // Chu kỳ mặc định 10ms nếu dt thất thường
            }

            // Trừ offset và đổi Gyro từ dps -> rad/s (bắt buộc cho Mahony)
            float gx_raw = (gyro[0] - s_gyro_bias.x) * DEG_TO_RAD;
            float gy_raw = (gyro[1] - s_gyro_bias.y) * DEG_TO_RAD;
            float gz_raw = (gyro[2] - s_gyro_bias.z) * DEG_TO_RAD;

            // 1. Remap Accel (Giữ chuẩn Right-Handed Coordinate System)
            float ax_body = accel[1];  // Trục Y IMU làm hướng tiến (+X Body)
            float ay_body = -accel[0]; // Trục X IMU làm hướng ngang (+Y Body, chú ý dấu ÂM)
            float az_body = accel[2];  // Trục Z IMU hướng xuống (+Z Body)

            // 2. Remap Gyro đồng bộ 1:1 theo trục Accel
            float gx_body = gy_raw;  // Gyro Roll theo trục X body (từ Gyro Y IMU)
            float gy_body = -gx_raw; // Gyro Pitch theo trục Y body (từ Gyro X IMU, đảo dấu)
            float gz_body = -gz_raw; // Gyro Yaw (Xoay phải -> Yaw tăng)

            // 3. Truyền vào Mahony AHRS
            ahrs_state_t temp_ahrs;
            mahony_ahrs_update_6dof(gx_body, gy_body, gz_body,
                                    ax_body, ay_body, az_body,
                                    dt, &temp_ahrs);
            // Ghi dữ liệu an toàn vào biến toàn cục cho task monitor
            if (xSemaphoreTake(g_ahrs_mutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                g_ahrs_state = temp_ahrs;
                xSemaphoreGive(g_ahrs_mutex);
            }
        }

        /* Đảm bảo vòng lặp định thời 10ms (100Hz) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }

    mpu6500_basic_deinit();
    vTaskDelete(NULL);
}

// --- TASK ĐỌC PHỤ & MONITOR CẢM BIẾN (PMW3901, VL53L0X & MAHONY AHRS) ---
void sensor_monitor_task(void *pvParameters)
{
    motionBurst_t pmw_motion = {0};
    uint16_t distance_mm = 0;
    ahrs_state_t local_ahrs = {0};

    vl53l0xStartContinuous(&vl53_dev, 0);

    while (1)
    {
        // 1. Đọc PMW3901 Optical Flow
        pmw3901ReadMotion(PMW3901_CS_PIN, &pmw_motion);

        // 2. Đọc VL53L0X ToF Distance
        distance_mm = vl53l0xReadRangeContinuousMillimeters(&vl53_dev);

        // 3. Đọc dữ liệu tư thế an toàn từ Mutex
        if (xSemaphoreTake(g_ahrs_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            local_ahrs = g_ahrs_state;
            xSemaphoreGive(g_ahrs_mutex);
        }

        // Chuyển đổi góc từ Radian -> Độ (Degree)
        float roll_deg = local_ahrs.roll * RAD_TO_DEG;
        float pitch_deg = local_ahrs.pitch * RAD_TO_DEG;
        float yaw_deg = local_ahrs.yaw * RAD_TO_DEG;

        // 4. In tổng hợp dữ liệu toàn bộ hệ thống cảm biến (GIỮ NGUYÊN FORMAT GỐC FOR VISUALIZATION)
        ESP_LOGI(TAG, "ATTITUDE | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f",
                 roll_deg, pitch_deg, yaw_deg);
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

static void vl53l0x_hard_reset(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VL53L0X_XSHUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_direction(VL53L0X_XSHUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(VL53L0X_XSHUT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(VL53L0X_XSHUT_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

// --- MAIN ENTRY POINT ---
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing System Hardware...");

    // 0. Tạo Mutex bảo vệ dữ liệu AHRS giữa 2 Tasks
    g_ahrs_mutex = xSemaphoreCreateMutex();
    if (g_ahrs_mutex == NULL)
    {
        ESP_LOGE(TAG, "Khong the tao Mutex AHRS!");
        return;
    }

    // 1. Khởi tạo Buses
    app_spi_init();

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
    vl53l0x_hard_reset();
    init_i2c_bus();
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