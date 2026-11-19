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
#include "altitude_filter.h" // Bộ lọc cao độ EKF 1D tích hợp esp-dsp
#include "flow_filter.h"     // Bộ lọc Optical Flow EKF 2D tích hợp esp-dsp
static const char *TAG = "MAIN_SYSTEM";

// --- CHUYỂN ĐỔI ĐƠN VỊ & HẰNG SỐ ---
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)
#define GRAVITY_MSS 9.80665f

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
typedef struct
{
    ahrs_state_t ahrs;
    float gyro_x; // Tốc độ góc Roll (rad/s)
    float gyro_y; // Tốc độ góc Pitch (rad/s)
    float gyro_z; // Tốc độ góc Yaw (rad/s)
} shared_ahrs_t;

static shared_ahrs_t g_shared_ahrs = {0};
static SemaphoreHandle_t g_ahrs_mutex = NULL;
static gyro_bias_t s_gyro_bias = {0};

// --- QUẢN LÝ DỮ LIỆU MAHONY AHRS TOÀN CỤC & MUTEX ---
static ahrs_state_t g_ahrs_state = {0};

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

// --- TASK ĐỌC IMU, TÍNH GÓC MAHONY AHRS VÀ DỰ BÁO EKF ALTITUDE (100Hz) ---
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

    /* 4. Vòng lặp tính toán góc Mahony AHRS & EKF Predict (Chu kỳ 10ms = 100Hz) */
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

            // 4. Tính toán gia tốc đứng trong hệ Trái Đất az_world (m/s^2)
            float roll = temp_ahrs.roll;
            float pitch = temp_ahrs.pitch;
            float yaw = temp_ahrs.yaw;

            // Tính gia tốc phẳng ax_world, ay_world trong hệ Trái Đất (m/s^2)
            float sin_r = sinf(roll), cos_r = cosf(roll);
            float sin_p = sinf(pitch), cos_p = cosf(pitch);
            float sin_y = sinf(yaw), cos_y = cosf(yaw);

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
            // 5. Thực thi bước PREDICT cho 1D Altitude EKF tại tần số IMU 100Hz
            altitude_filter_predict(az_world, dt);
            flow_filter_predict(ax_world, ay_world, dt);
            // Ghi dữ liệu tư thế an toàn vào biến toàn cục cho task monitor
            // ... (Đoạn tính toán gx_body, gy_body, gz_body và gọi mahony_ahrs_update_6dof phía trên giữ nguyên) ...

            // Ghi dữ liệu tư thế và tốc độ góc an toàn vào biến toàn cục cho task monitor
            if (xSemaphoreTake(g_ahrs_mutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                g_shared_ahrs.ahrs = temp_ahrs;
                g_shared_ahrs.gyro_x = gx_body;
                g_shared_ahrs.gyro_y = gy_body;
                g_shared_ahrs.gyro_z = gz_body;
                xSemaphoreGive(g_ahrs_mutex);
            }
        }

        /* Đảm bảo vòng lặp định thời 10ms (100Hz) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }

    mpu6500_basic_deinit();
    vTaskDelete(NULL);
}

// --- TASK MONITOR CẢM BIẾN (PMW3901, VL53L0X, MAHONY AHRS & EKF UPDATE) ---
void sensor_monitor_task(void *pvParameters)
{
    motionBurst_t pmw_motion = {0};
    uint16_t distance_mm = 0;
    shared_ahrs_t local_shared = {0};

    // 1. Khởi tạo bộ lọc 2D Position EKF (Bước 4)
    flow_filter_init();
    vl53l0xStartContinuous(&vl53_dev, 0);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float dt_monitor = 0.05f; // Loop 20Hz (50ms)

    while (1)
    {
        // 2. Đọc cảm biến PMW3901 Optical Flow & VL53L0X ToF
        pmw3901ReadMotion(PMW3901_CS_PIN, &pmw_motion);
        distance_mm = vl53l0xReadRangeContinuousMillimeters(&vl53_dev);

        // 3. Đọc dữ liệu tư thế & tốc độ góc an toàn từ Mutex
        if (xSemaphoreTake(g_ahrs_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            local_shared = g_shared_ahrs;
            xSemaphoreGive(g_ahrs_mutex);
        }

        ahrs_state_t local_ahrs = local_shared.ahrs;

        // Chuyển đổi góc từ Radian -> Độ (Degree)
        float roll_deg = local_ahrs.roll * RAD_TO_DEG;
        float pitch_deg = local_ahrs.pitch * RAD_TO_DEG;
        float yaw_deg = local_ahrs.yaw * RAD_TO_DEG;

        // 4. Cập nhật EKF Altitude (Bước 3) khi có khoảng cách ToF hợp lệ
        if (distance_mm > 20 && distance_mm < 2000)
        {
            float z_raw_m = (float)distance_mm / 1000.0f;
            float z_meas = z_raw_m * cosf(local_ahrs.roll) * cosf(local_ahrs.pitch);

            altitude_filter_update(z_meas);
        }

        float est_z = altitude_filter_get_z();
        float est_vz = altitude_filter_get_vz();
        float z_for_flow = (est_z < 0.05f) ? 0.05f : est_z;

        // 5. THỰC HIỆN DE-ROTATION & UPDATE 2D OPTICAL FLOW EKF (Bước 4)
        if (pmw_motion.motionOccured)
        {
            // [BẢO VỆ 1]: Outlier Rejection - Bỏ qua xung rác SPI nổ số
            if (abs(pmw_motion.deltaX) < 300 && abs(pmw_motion.deltaY) < 300)
            {
                // Tốc độ góc pixel -> rad/s
                float flow_rad_x = ((float)pmw_motion.deltaX * 0.0024f) / dt_monitor;
                float flow_rad_y = ((float)pmw_motion.deltaY * 0.0024f) / dt_monitor;

                // De-rotation: Trừ đi tốc độ góc gyro (rad/s)
                float flow_trans_x = flow_rad_x - local_shared.gyro_y;
                float flow_trans_y = flow_rad_y + local_shared.gyro_x;

                // Quy đổi sang vận tốc thực m/s (v = omega * z)
                float vx_flow = flow_trans_x * z_for_flow;
                float vy_flow = flow_trans_y * z_for_flow;

                // Update bước đo lường vào EKF 2D
                flow_filter_update(vx_flow, vy_flow, pmw_motion.squal);
            }
            else
            {
                ESP_LOGW(TAG, "OPT_FLOW SPI Spike Detected! dX: %d, dY: %d (Ignored)",
                         pmw_motion.deltaX, pmw_motion.deltaY);
            }
        }
        if (est_z < 0.06f)
        {
            // Reset EKF 2D về 0 khi nằm yên dưới đất để tránh tích phân trôi khi chuẩn bị cất cánh
            flow_filter_reset();
        }

        // 6. Trích xuất vị trí 2D (x, y) và vận tốc 2D (vx, vy)
        pos2d_estimate_t pos2d = {0};
        flow_filter_get_estimate(&pos2d);

        // 7. IN LOG TỔNG HỢP VÀ KHUNG DỮ LIỆU BẢNG ĐIỀU KHIỂN
        ESP_LOGI(TAG, "ATTITUDE | Roll: %6.2f° | Pitch: %6.2f° | Yaw: %6.2f°", roll_deg, pitch_deg, yaw_deg);
        ESP_LOGI(TAG, "2D_POS   | X: %6.2fm | Y: %6.2fm | Vx: %6.2fm/s | Vy: %6.2fm/s", pos2d.x, pos2d.y, pos2d.vx, pos2d.vy);
        ESP_LOGI(TAG, "ALTITUDE | Dist: %4dmm | Z: %6.3fm | Vz: %6.3fm/s", distance_mm, est_z, est_vz);
        ESP_LOGI(TAG, "OPT_FLOW | Motion: %d | dX: %4d | dY: %4d | SQual: %3d",
                 pmw_motion.motionOccured, pmw_motion.deltaX, pmw_motion.deltaY, pmw_motion.squal);

        // Stream dữ liệu định dạng chuẩn Visualizer:
        printf("/*%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d*/\n",
               roll_deg, pitch_deg, yaw_deg,
               pos2d.x, pos2d.y, est_z,
               pos2d.vx, pos2d.vy, est_vz,
               pmw_motion.squal);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
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

    // Khởi tạo 1D Altitude EKF với cao độ mốc ban đầu 0.0m
    altitude_filter_init(0.0f);

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