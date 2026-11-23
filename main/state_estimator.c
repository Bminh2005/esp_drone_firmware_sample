#include "state_estimator.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

// Hardware Drivers & Filters
#include "pmw3901.h"
#include "vl53l0x.h"
#include "driver_mpu6500_basic.h"
#include "maholy.h"
#include "altitude_filter.h"
#include "flow_filter.h"

static const char *TAG = "STATE_ESTIMATOR";

// --- HẰNG SỐ ĐƠN VỊ & NGƯỠNG KHỬ NHIỄU ---
#define RAD_TO_DEG (180.0f / 3.1415926535f)
#define DEG_TO_RAD (3.1415926535f / 180.0f)
#define GRAVITY_MSS 9.80665f
#define GYRO_DEADBAND_RAD (0.12f * DEG_TO_RAD)

// --- BIAS GIA TỐC CỐ ĐỊNH ---
#define ACCEL_BIAS_X (+0.0108f)
#define ACCEL_BIAS_Y (+0.0058f)
#define ACCEL_BIAS_Z (+0.0586f)

// --- BIAS GYRO ĐỘNG ---
static float s_gyro_bias_x = -0.0009f;
static float s_gyro_bias_y = +0.0138f;
static float s_gyro_bias_z = +0.0043f;

// --- BIẾN ĐỒNG BỘ CALIB GYRO ---
static volatile bool s_calib_requested = false;
static volatile bool s_calib_done = false;

// --- PINOUT PHẦN CỨNG ---
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define PMW3901_CS_PIN GPIO_NUM_26
#define VL53L0X_XSHUT_PIN GPIO_NUM_17

// --- BIẾN NỘI BỘ MODULE ---
static uav_state_t s_uav_state = {0};
static SemaphoreHandle_t s_state_mutex = NULL;

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_vl53_dev_handle = NULL;
spi_device_handle_t pmw3901_spi_handle = NULL;
static VL53L0xDev s_vl53_dev;

static void app_spi_init(void)
{
    spi_bus_config_t buscfg_pmw = {
        .miso_io_num = GPIO_NUM_33,
        .mosi_io_num = GPIO_NUM_32,
        .sclk_io_num = GPIO_NUM_25,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg_pmw, SPI_DMA_CH_AUTO));

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

    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg_pmw, &pmw3901_spi_handle));
}

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
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &s_i2c_bus_handle));

    i2c_device_config_t vl53_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_DEFAULT_ADDRESS,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus_handle, &vl53_cfg, &s_vl53_dev_handle));
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

// --- TASK ĐỌC IMU (CORE 1 - NÂNG LÊN 250Hz / MAHONY AHRS + EKF PREDICT) ---
static void mpu6500_read_task(void *pvParameters)
{
    float accel[3], gyro[3];
    uint8_t res = mpu6500_basic_init(MPU6500_INTERFACE_SPI, MPU6500_ADDRESS_AD0_LOW);
    if (res != 0)
    {
        ESP_LOGE(TAG, "MPU6500 SPI init failed!");
        vTaskDelete(NULL);
        return;
    }

    mahony_init();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t last_time = esp_timer_get_time();

    float accum_gx = 0.0f, accum_gy = 0.0f, accum_gz = 0.0f;
    int calib_count = 0;

    // Biến tích lũy gia tốc để hạ tần số EKF xuống 50Hz (nhẹ CPU)
    static uint8_t ekf_divider = 0;
    static float az_accum = 0.0f, ax_accum = 0.0f, ay_accum = 0.0f;

    while (1)
    {
        if (mpu6500_basic_read(accel, gyro) == 0)
        {
            int64_t now = esp_timer_get_time();
            float dt = (float)(now - last_time) / 1000000.0f;
            last_time = now;
            if (dt <= 0.0f || dt > 0.05f)
                dt = 0.004f; // 250Hz

            // CALIB GYRO ĐỘNG (250 MẪU ~ 1.0 GIÂY)
            if (s_calib_requested)
            {
                accum_gx += gyro[0] * DEG_TO_RAD;
                accum_gy += gyro[1] * DEG_TO_RAD;
                accum_gz += gyro[2] * DEG_TO_RAD;
                calib_count++;

                if (calib_count >= 250)
                {
                    s_gyro_bias_x = accum_gx / 250.0f;
                    s_gyro_bias_y = accum_gy / 250.0f;
                    s_gyro_bias_z = accum_gz / 250.0f;

                    accum_gx = accum_gy = accum_gz = 0.0f;
                    calib_count = 0;

                    mahony_init();
                    altitude_filter_init(0.0f);
                    flow_filter_init();

                    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
                    {
                        bool init_flag = s_uav_state.is_initialized;
                        memset(&s_uav_state, 0, sizeof(uav_state_t));
                        s_uav_state.is_initialized = init_flag;
                        xSemaphoreGive(s_state_mutex);
                    }

                    s_calib_requested = false;
                    s_calib_done = true;
                }

                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
                continue;
            }

            // BÙ BIAS THÔ VÀ REMAP SANG BODY FRAME
            float ax_imu_cal = accel[0] - ACCEL_BIAS_X;
            float ay_imu_cal = accel[1] - ACCEL_BIAS_Y;
            float az_imu_cal = accel[2] - ACCEL_BIAS_Z;

            float gx_raw = (gyro[0] * DEG_TO_RAD) - s_gyro_bias_x;
            float gy_raw = (gyro[1] * DEG_TO_RAD) - s_gyro_bias_y;
            float gz_raw = (gyro[2] * DEG_TO_RAD) - s_gyro_bias_z;

            float ax_body = ay_imu_cal;
            float ay_body = -ax_imu_cal;
            float az_body = az_imu_cal;

            float gx_body = gy_raw;
            float gy_body = -gx_raw;
            float gz_body = -gz_raw;

            float gx_mahony = (fabsf(gx_body) < GYRO_DEADBAND_RAD) ? 0.0f : gx_body;
            float gy_mahony = (fabsf(gy_body) < GYRO_DEADBAND_RAD) ? 0.0f : gy_body;
            float gz_mahony = (fabsf(gz_body) < GYRO_DEADBAND_RAD) ? 0.0f : gz_body;

            // 1. MAHONY AHRS UPDATE (CHẠY ĐỦ 250Hz ĐỂ ĐỦ GÓC NGHIÊNG MỚI NHẤT CHO PID)
            ahrs_state_t temp_ahrs;
            mahony_ahrs_update_6dof(gx_mahony, gy_mahony, gz_mahony,
                                    ax_body, ay_body, az_body,
                                    dt, &temp_ahrs);

            // ACCELERATION TO WORLD FRAME
            float sin_r = sinf(temp_ahrs.roll), cos_r = cosf(temp_ahrs.roll);
            float sin_p = sinf(temp_ahrs.pitch), cos_p = cosf(temp_ahrs.pitch);
            float sin_y = sinf(temp_ahrs.yaw), cos_y = cosf(temp_ahrs.yaw);

            float az_world_g = -ax_body * sin_p + ay_body * sin_r * cos_p + az_body * cos_r * cos_p - 1.0f;
            float az_world = az_world_g * GRAVITY_MSS;

            float ax_w_g = ax_body * cos_p * cos_y +
                           ay_body * (sin_r * sin_p * cos_y - cos_r * sin_y) +
                           az_body * (cos_r * sin_p * cos_y + sin_r * sin_y);

            float ay_w_g = ax_body * cos_p * sin_y +
                           ay_body * (sin_r * sin_p * sin_y + cos_r * cos_y) +
                           az_body * (cos_r * sin_p * sin_y - sin_r * cos_y);

            float ax_world = ax_w_g * GRAVITY_MSS;
            float ay_world = ay_w_g * GRAVITY_MSS;

            if (fabsf(ax_world) < 0.15f)
                ax_world = 0.0f;
            if (fabsf(ay_world) < 0.15f)
                ay_world = 0.0f;

            // 2. TÍCH LŨY GIA TỐC VÀ DOWNSAMPLE EKF PREDICT XUỐNG 50Hz (5 VÒNG LẶP = 20ms)
            az_accum += az_world;
            ax_accum += ax_world;
            ay_accum += ay_world;

            if (++ekf_divider >= 5)
            {
                float dt_ekf = dt * 5.0f; // dt 20ms
                altitude_filter_predict(az_accum / 5.0f, dt_ekf);
                flow_filter_predict(ax_accum / 5.0f, ay_accum / 5.0f, dt_ekf);

                az_accum = 0.0f;
                ax_accum = 0.0f;
                ay_accum = 0.0f;
                ekf_divider = 0;
            }

            // GHI DỮ LIỆU ĐÃ CẬP NHẬT
            if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
            {
                s_uav_state.roll = temp_ahrs.roll;
                s_uav_state.pitch = temp_ahrs.pitch;
                s_uav_state.yaw = temp_ahrs.yaw;

                s_uav_state.roll_deg = temp_ahrs.roll * RAD_TO_DEG;
                s_uav_state.pitch_deg = temp_ahrs.pitch * RAD_TO_DEG;
                s_uav_state.yaw_deg = temp_ahrs.yaw * RAD_TO_DEG;

                s_uav_state.gyro_x = gx_body;
                s_uav_state.gyro_y = gy_body;
                s_uav_state.gyro_z = gz_body;
                xSemaphoreGive(s_state_mutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4)); // 250Hz (4ms)
    }
}
// --- TASK MONITOR SENSOR (CORE 0 - 20Hz / OPTICAL FLOW + TOF + EKF UPDATE) ---
static void sensor_monitor_task(void *pvParameters)
{
    motionBurst_t pmw_motion = {0};
    uint16_t distance_mm = 0;

    flow_filter_init();
    vl53l0xStartContinuous(&s_vl53_dev, 0);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t last_time = esp_timer_get_time();

    while (1)
    {
        int64_t now = esp_timer_get_time();
        float dt_monitor = (float)(now - last_time) / 1000000.0f;
        last_time = now;
        if (dt_monitor <= 0.0f || dt_monitor > 0.2f)
            dt_monitor = 0.05f;

        pmw3901ReadMotion(PMW3901_CS_PIN, &pmw_motion);
        distance_mm = vl53l0xReadRangeContinuousMillimeters(&s_vl53_dev);

        float current_roll = 0.0f, current_pitch = 0.0f;
        float current_gx = 0.0f, current_gy = 0.0f;

        if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(3)) == pdTRUE)
        {
            current_roll = s_uav_state.roll;
            current_pitch = s_uav_state.pitch;
            current_gx = s_uav_state.gyro_x;
            current_gy = s_uav_state.gyro_y;
            xSemaphoreGive(s_state_mutex);
        }

        if (distance_mm > 20 && distance_mm < 2000)
        {
            float z_raw_m = (float)distance_mm / 1000.0f;
            float z_meas = z_raw_m * cosf(current_roll) * cosf(current_pitch);
            altitude_filter_update(z_meas);
        }

        float est_z = altitude_filter_get_z();
        float est_vz = altitude_filter_get_vz();
        float z_for_flow = (est_z < 0.05f) ? 0.05f : est_z;

        if (pmw_motion.motionOccured)
        {
            if (abs(pmw_motion.deltaX) < 300 && abs(pmw_motion.deltaY) < 300)
            {
                float flow_rad_x = ((float)pmw_motion.deltaX * 0.0024f) / dt_monitor;
                float flow_rad_y = ((float)pmw_motion.deltaY * 0.0024f) / dt_monitor;

                float flow_trans_x = flow_rad_x - current_gy;
                float flow_trans_y = flow_rad_y + current_gx;

                float vx_flow = flow_trans_x * z_for_flow;
                float vy_flow = flow_trans_y * z_for_flow;

                flow_filter_update(vx_flow, vy_flow, pmw_motion.squal);
            }
        }

        if (est_z < 0.07f)
        {
            flow_filter_zero_velocity();
        }

        pos2d_estimate_t pos2d = {0};
        flow_filter_get_estimate(&pos2d);

        if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(3)) == pdTRUE)
        {
            s_uav_state.z = est_z;
            s_uav_state.vz = est_vz;
            s_uav_state.dist_mm = distance_mm;

            s_uav_state.x = pos2d.x;
            s_uav_state.y = pos2d.y;
            s_uav_state.vx = pos2d.vx;
            s_uav_state.vy = pos2d.vy;
            s_uav_state.squal = pmw_motion.squal;
            xSemaphoreGive(s_state_mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// --- PUBLIC APIs ---

esp_err_t state_estimator_init(void)
{
    ESP_LOGI(TAG, "Initializing State Estimator Module...");

    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create State Mutex!");
        return ESP_FAIL;
    }

    altitude_filter_init(0.0f);
    app_spi_init();
    vTaskDelay(pdMS_TO_TICKS(50));

    if (!pmw3901Init(PMW3901_CS_PIN))
    {
        ESP_LOGE(TAG, "PMW3901 Init FAIL!");
        return ESP_FAIL;
    }

    vl53l0x_hard_reset();
    init_i2c_bus();
    if (!vl53l0xInit(&s_vl53_dev, s_vl53_dev_handle, true))
    {
        ESP_LOGE(TAG, "VL53L0X Init FAIL!");
        return ESP_FAIL;
    }

    // Pinned IMU Task -> CORE 1 (Priority 6), Flow Task -> CORE 0 (Priority 5)
    xTaskCreatePinnedToCore(mpu6500_read_task, "imu_task", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(sensor_monitor_task, "flow_task", 4096, NULL, 5, NULL, 0);

    s_uav_state.is_initialized = true;
    ESP_LOGI(TAG, "State Estimator Module Ready!");

    state_estimator_calibrate_gyro();

    return ESP_OK;
}

esp_err_t state_estimator_calibrate_gyro(void)
{
    ESP_LOGI(TAG, "Requesting Dynamic Gyro Calibration... Keep UAV stationary!");
    s_calib_done = false;
    s_calib_requested = true;

    int timeout_ms = 2000;
    while (!s_calib_done && timeout_ms > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        timeout_ms -= 20;
    }

    if (!s_calib_done)
    {
        ESP_LOGE(TAG, "Gyro Calibration Timeout!");
        s_calib_requested = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Gyro Calibration OK! Bias (rad/s) -> X:%.5f, Y:%.5f, Z:%.5f",
             s_gyro_bias_x, s_gyro_bias_y, s_gyro_bias_z);
    return ESP_OK;
}

bool state_estimator_get_state(uav_state_t *out_state)
{
    if (out_state == NULL || !s_uav_state.is_initialized)
        return false;

    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
    {
        *out_state = s_uav_state;
        xSemaphoreGive(s_state_mutex);
        return true;
    }
    return false;
}

void state_estimator_stream_telemetry(const uav_state_t *state)
{
    if (state == NULL)
        return;
    printf("/*%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d*/\n",
           state->roll_deg, state->pitch_deg, state->yaw_deg,
           state->x, state->y, state->z,
           state->vx, state->vy, state->vz,
           state->squal);
}