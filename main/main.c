#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "state_estimator.h"
#include "flight_control.h"

static const char *TAG = "MAIN_SYSTEM";

// === BIẾN TRẠNG THÁI TOÀN CỤC ===
static uav_state_t s_uav_state = {0};
static drone_setpoint_t s_drone_setpoint = {
    .vx_target = 0.0f,
    .vy_target = 0.0f,
    .yaw_rate_target = 0.0f,
    .z_target = 0.0f,
    .armed = false,
    .last_packet_ms = 0};

// === BIẾN ĐỊA CHỈ MÁY TÍNH NHẬN TELEMETRY UDP ===
static struct sockaddr_in s_client_telemetry_addr = {
    .sin_family = AF_INET,
    .sin_port = htons(8889),
    .sin_addr.s_addr = 0 // Tự động cập nhật khi nhận gói điều khiển
};
static volatile bool s_client_connected = false;

// === KHỞI TẠO WI-FI ACCESS POINT (SOFT-AP) ===
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32-DRONE-NET",
            .ssid_len = strlen("ESP32-DRONE-NET"),
            .channel = 1,
            .password = "", // Không dùng mật khẩu để tối ưu latency
            .max_connection = 2,
            .authmode = WIFI_AUTH_OPEN},
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP Initialized. SSID: ESP32-DRONE-NET (IP: 192.168.4.1)");
}

// === CORE 0: UDP COMMAND RECEIVER & FAILSAFE WATCHDOG (50Hz) ===
static void udp_rx_task(void *pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create UDP RX socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct timeval timeout = {.tv_sec = 0, .tv_usec = 50000}; // Timeout 50ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(8888),
        .sin_addr.s_addr = htonl(INADDR_ANY)};

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    uint8_t rx_buf[64];
    ESP_LOGI(TAG, "UDP Command Receiver active on Port 8888 (Core 0)");

    while (1)
    {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // Gói 18-byte: 4 floats (16B) + armed (1B) + calib_req (1B)
        if (len >= 18)
        {
            float *fp = (float *)rx_buf;
            s_drone_setpoint.vx_target = fp[0];       // m/s
            s_drone_setpoint.vy_target = fp[1];       // m/s
            s_drone_setpoint.yaw_rate_target = fp[2]; // deg/s
            s_drone_setpoint.z_target = fp[3];        // m
            s_drone_setpoint.armed = (rx_buf[16] != 0);
            uint8_t calib_req = rx_buf[17];

            s_drone_setpoint.last_packet_ms = now_ms;

            // Tự động lưu IP máy tính đang điều khiển để gửi Telemetry
            s_client_telemetry_addr.sin_addr = source_addr.sin_addr;
            s_client_telemetry_addr.sin_port = htons(8889);
            s_client_connected = true;

            // Xử lý lệnh Calib Gyro khẩn (Chỉ thực hiện khi DISARM)
            if (calib_req == 1 && !s_drone_setpoint.armed)
            {
                ESP_LOGW(TAG, "UDP Command: Triggering Dynamic Gyro Calibration...");
                state_estimator_calibrate_gyro();
            }
        }

        // Failsafe Watchdog: Quá 500ms không nhận dữ liệu -> DISARM
        if (now_ms - s_drone_setpoint.last_packet_ms > FAILSAFE_TIMEOUT_MS)
        {
            if (s_drone_setpoint.armed)
            {
                ESP_LOGW(TAG, "UDP Timeout! Disarming motors for safety.");
            }
            s_drone_setpoint.armed = false;
            s_drone_setpoint.z_target = 0.0f;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

// === CORE 0: UDP TELEMETRY TRANSMITTER (20Hz) ===
static void udp_tx_telemetry_task(void *pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create UDP TX socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // BẮT BỘC: Bật SO_BROADCAST để sendto() tới 192.168.4.255 không bị lỗi EACCES (errno 13)
    int broadcast_enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    // Tăng kích thước buffer lên 256 byte để tránh tràn chuỗi khi thêm thông số motor
    char tx_buf[256];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    ESP_LOGI(TAG, "UDP Telemetry Sender active on Port 8889 (Core 0)");

    while (1)
    {
        // Nhịp gửi 20Hz (50ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

        uav_state_t current_state;
        float motor_targets[4] = {0}; // M1, M2, M3, M4 (0.0f đến 1.0f)
        flight_ctrl_debug_t debug = {0};

        bool state_ok = state_estimator_get_state(&current_state);

        // Lấy công suất 4 động cơ và dữ liệu debug 4 tầng từ flight_control
        flight_control_get_motors(motor_targets);
        flight_control_get_debug(&debug);

        if (state_ok)
        {
            /*
             * ĐỊNH DẠNG CHUỖI DEBUG UDP TELEMETRY (24 tham số):
             * Thái độ gốc (State):     roll, pitch, yaw (deg)
             * Tầng 1 (Z Pos/Vel):      z_target, z_state, u_z
             * Tầng 2 (PosHold Vel):    vx_state, vy_state, target_pitch_deg, target_roll_deg
             * Tầng 3 (Angle):          roll_deg, pitch_deg, target_rate_roll, target_rate_pitch
             * Tầng 4 (Rate & Torque):  gyro_x, gyro_y, gyro_z, u_roll, u_pitch, u_yaw
             * Tầng 5 (Motor PWM %):    m1, m2, m3, m4
             */
            int len = snprintf(tx_buf, sizeof(tx_buf),
                               "/*%.2f,%.2f,%.2f,"              // State: Roll, Pitch, Yaw
                               "%.2f,%.2f,%.2f,"                // T1: z_tgt, z_st, u_z
                               "%.2f,%.2f,%.1f,%.1f,"           // T2: vx, vy, tgt_pitch, tgt_roll
                               "%.1f,%.1f,%.1f,%.1f,"           // T3: roll, pitch, tgt_r_roll, tgt_r_pitch
                               "%.1f,%.1f,%.1f,%.3f,%.3f,%.3f," // T4: gx, gy, gz, u_roll, u_pitch, u_yaw
                               "%.1f,%.1f,%.1f,%.1f*/\n",       // T5: m1, m2, m3, m4 (%)

                               // Thái độ gốc hiện tại
                               current_state.roll_deg, current_state.pitch_deg, current_state.yaw_deg,
                               // Tầng 1
                               debug.z_target, debug.z_state, debug.u_z,
                               // Tầng 2
                               debug.vx_state, debug.vy_state, debug.target_pitch_deg, debug.target_roll_deg,
                               // Tầng 3
                               debug.roll_deg, debug.pitch_deg, debug.target_rate_roll, debug.target_rate_pitch,
                               // Tầng 4
                               debug.gyro_x, debug.gyro_y, debug.gyro_z, debug.u_roll, debug.u_pitch, debug.u_yaw,
                               // Tầng 5 (Quy đổi ra 0% -> 100%)
                               motor_targets[0] * 100.0f, motor_targets[1] * 100.0f,
                               motor_targets[2] * 100.0f, motor_targets[3] * 100.0f);

            if (len > 0 && len < sizeof(tx_buf))
            {
                if (s_client_connected)
                {
                    // Gửi trực tiếp tới IP máy tính đã kết nối
                    sendto(sock, tx_buf, len, 0,
                           (struct sockaddr *)&s_client_telemetry_addr,
                           sizeof(s_client_telemetry_addr));
                }
                else
                {
                    // Gửi Broadcast 192.168.4.255
                    struct sockaddr_in bcast_addr = {
                        .sin_family = AF_INET,
                        .sin_port = htons(8889),
                        .sin_addr.s_addr = inet_addr("192.168.4.255")};
                    sendto(sock, tx_buf, len, 0,
                           (struct sockaddr *)&bcast_addr,
                           sizeof(bcast_addr));
                }
            }
        }
    }
}

// === CORE 1: HARD REAL-TIME STABILIZER LOOP (500Hz) ===
static void stabilizer_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2ms = 500Hz chuẩn real-time
    flight_control_init();
    // Khởi tạo phần cứng PWM LEDC và tham số Cascaded PID
    ESP_LOGI(TAG, "Stabilizer Task active @ 500Hz on Core 1");

    uint8_t uart_decimation = 0;

    while (1)
    {
        // Bù nhịp chính xác 2ms cho vòng lặp 500Hz
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Lấy trạng thái từ State Estimator (Thread-safe Mutex)
        if (state_estimator_get_state(&s_uav_state))
        {
            // Thực thi PosHold PID (Altitude Vz + Flow 2D + Angle + Rate) & xuất LEDC PWM
            flight_control_step(&s_uav_state, &s_drone_setpoint, 0.002f);

            // In dữ liệu Telemetry ra UART qua hàm state_estimator_stream_telemetry() ở nhịp 20Hz
            if (++uart_decimation >= 25)
            {
                uart_decimation = 0;
                state_estimator_stream_telemetry(&s_uav_state);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32 UAV FIRMWARE STARTING ===");
    flight_control_init();
    // 1. Khởi tạo Flash NVS (Bắt buộc cho stack Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Bật Wi-Fi SoftAP
    wifi_init_softap();

    // 3. Khởi tạo Mô-đun State Estimator (MPU6500, VL53L0X, PMW3901)
    if (state_estimator_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "State Estimator Init Failed! System Halted.");
        return;
    }

    // 4. Các Task chạy trên CORE 0 (PRO_CPU) - Xử lý mạng UDP
    xTaskCreatePinnedToCore(udp_rx_task, "udp_rx_task", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(udp_tx_telemetry_task, "udp_tx_task", 4096, NULL, 4, NULL, 0);

    // 5. Task duy nhất trên CORE 1 (APP_CPU) - Ưu tiên tuyệt đối cho điều khiển real-time
    xTaskCreatePinnedToCore(stabilizer_task, "stabilizer_task", 4096, NULL, 7, NULL, 1);

    ESP_LOGI(TAG, "System initialization complete. Ready for Position Hold flight.");
}