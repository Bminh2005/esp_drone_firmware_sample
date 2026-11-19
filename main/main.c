#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/sockets.h"

#include "state_estimator.h"
#include "flight_control.h"

static const char *TAG = "MAIN_APP";

// =========================================================================
// 1. CẤU HÌNH TRẠNG THÁI FLIGHT STATE & THAM SỐ AN TOÀN
// =========================================================================
typedef enum
{
    STATE_DISARMED = 0,
    STATE_ARMED_IDLE,
    STATE_TAKEOFF_RAMP,
    STATE_FLYING_MANUAL,
    STATE_LANDING_AUTO,
    STATE_EMERGENCY
} drone_flight_state_t;

// =========================================================================
// 1. CẤU HÌNH TRẠNG THÁI FLIGHT STATE & THAM SỐ AN TOÀN (ĐÃ TỐI ƯU CẤT CÁNH)
// =========================================================================
#define IDLE_THROTTLE 0.10f           // 10% Ga rốt-đa quay quét khi ARM
#define TARGET_TAKEOFF_THROTTLE 0.46f // Ga cất cánh cơ bản (42%)
#define TARGET_TAKEOFF_ALT 0.30f      // Cao độ mục tiêu (30cm)

// Ramp ga từ 10% -> 42% trong 0.9 giây để thoát mặt sàn dứt khoát
#define THROTTLE_RAMP_RATE 0.25f // Độ cao cất cánh mục tiêu (30cm)

// Siết thời gian chờ lệnh Wi-Fi về 2.0 giây cho an toàn
#define WIFI_COMMAND_TIMEOUT_US 2000000 // 2.0s Timeout -> Auto LANDING Failsafe
// --- CẤU HÌNH WIFI SOFTAP & UDP ---

static int s_udp_sock = -1;
static struct sockaddr_in s_client_addr;
static socklen_t s_client_addr_len = sizeof(s_client_addr);
static bool s_client_connected = false;

static drone_flight_state_t s_flight_state = STATE_DISARMED;
static int64_t s_last_cmd_timestamp = 0;
static uav_setpoint_t s_setpoint = {0};
// --- CẤU HÌNH ROUTER WIFI ---
#define ROUTER_SSID "TP-Link_318A"
#define ROUTER_PASS "18596229"
#define UDP_PORT 8888

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Lost connection to Router. Retrying...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, ">>> CONNECTED TO ROUTER! DRONE IP: " IPSTR " <<<", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ROUTER_SSID,
            .password = ROUTER_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Giới hạn công suất phát RF xuống 11dBm để tránh sụt áp LDO
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44)); // 44 = 11dBm (4 * 11)

    ESP_LOGI(TAG, "Connecting to Wi-Fi Router: %s ...", ROUTER_SSID);
    // Chờ cho tới khi nhận được địa chỉ IP thành công mới đi tiếp
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}
// // =========================================================================
// // 2. KHỞI TẠO WIFI SOFTAP & UDP BIDIRECTIONAL SOCKET
// // =========================================================================
// static void wifi_init_softap(void)
// {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_ap();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     wifi_config_t wifi_config = {
//         .ap = {
//             .ssid = WIFI_SSID,
//             .ssid_len = strlen(WIFI_SSID),
//             .channel = 1,
//             .password = WIFI_PASS,
//             .max_connection = 4,
//             .authmode = WIFI_AUTH_WPA2_PSK,
//         },
//     };
//     if (strlen(WIFI_PASS) == 0)
//     {
//         wifi_config.ap.authmode = WIFI_AUTH_OPEN;
//     }

//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());
//     ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_POWER_11dBm));
//     ESP_LOGI(TAG, "Wi-Fi SoftAP Ready. SSID: %s | Port: %d", WIFI_SSID, UDP_PORT);
// }

static void udp_socket_init(void)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0)
    {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};

    if (bind(s_udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
    {
        ESP_LOGE(TAG, "Failed to bind UDP socket: errno %d", errno);
        close(s_udp_sock);
        s_udp_sock = -1;
        return;
    }

    int flags = fcntl(s_udp_sock, F_GETFL, 0);
    fcntl(s_udp_sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "UDP Socket Bound & Set Non-Blocking on Port %d", UDP_PORT);
}

// =========================================================================
// 3. HÀM GỬI LOG VĂN BẢN VÀ TELEMETRY QUA WIFI UDP
// =========================================================================
void udp_send_log(const char *format, ...)
{
    if (s_udp_sock < 0 || !s_client_connected)
        return;

    char log_buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(log_buf, sizeof(log_buf), format, args);
    va_end(args);

    if (len > 0)
    {
        sendto(s_udp_sock, log_buf, len, MSG_DONTWAIT,
               (struct sockaddr *)&s_client_addr, s_client_addr_len);
    }
}

static void udp_stream_telemetry(const uav_state_t *state)
{
    if (s_udp_sock < 0 || !s_client_connected || state == NULL)
        return;

    float vbat = flight_control_get_vbat();
    const char *reason = flight_control_get_disarm_reason_str();

    char tx_buffer[200];
    int len = snprintf(tx_buffer, sizeof(tx_buffer),
                       "/*R:%.1f,P:%.1f,Y:%.1f,Z:%.2f,Vbat:%.2fV,State:%d,Err:%s*/\n",
                       state->roll_deg, state->pitch_deg, state->yaw_deg,
                       state->z, vbat, (int)s_flight_state, reason);

    // Cố định Port gửi log luôn là UDP_PORT (8888) thay vì lấy theo Port biến đổi của s_client_addr
    struct sockaddr_in target_addr = s_client_addr;
    target_addr.sin_port = htons(UDP_PORT);

    sendto(s_udp_sock, tx_buffer, len, MSG_DONTWAIT,
           (struct sockaddr *)&target_addr, sizeof(target_addr));
}
// =========================================================================
// 4. HÀM XỬ LÝ LỆNH TỪ BÀN ĐIỀU KHIỂN WIFI (COMMAND PARSER)
// =========================================================================
static void process_wifi_commands(void)
{
    if (s_udp_sock < 0)
        return;

    char rx_buffer[128];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    int len = recvfrom(s_udp_sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                       (struct sockaddr *)&from_addr, &from_len);

    if (len > 0)
    {
        rx_buffer[len] = '\0';

        memcpy(&s_client_addr, &from_addr, sizeof(from_addr));
        s_client_connected = true;
        s_last_cmd_timestamp = esp_timer_get_time();

        if (strncmp(rx_buffer, "ARM", 3) == 0)
        {
            if (s_flight_state == STATE_DISARMED)
            {
                s_flight_state = STATE_ARMED_IDLE;
                s_setpoint.hover_throttle = IDLE_THROTTLE;
                s_setpoint.alt_z_m = 0.0f;
                flight_control_set_setpoint(&s_setpoint);
                flight_control_set_mode(FLIGHT_MODE_STABILIZE);
                flight_control_arm();

                udp_send_log("[SYSTEM_LOG] -> ARMED OK! Vbat = %.2fV\n", flight_control_get_vbat());
                ESP_LOGW(TAG, "CMD RECV: >>> SYSTEM ARMED (IDLE) <<<");
            }
        }
        else if (strncmp(rx_buffer, "DISARM", 6) == 0)
        {
            s_flight_state = STATE_DISARMED;
            s_setpoint.hover_throttle = 0.0f;
            flight_control_set_setpoint(&s_setpoint);
            flight_control_disarm();

            udp_send_log("[SYSTEM_LOG] -> DISARM CMD EXECUTED\n");
            ESP_LOGW(TAG, "CMD RECV: >>> SYSTEM DISARMED <<<");
        }
        else if (strncmp(rx_buffer, "TAKEOFF", 7) == 0)
        {
            if (s_flight_state == STATE_ARMED_IDLE || s_flight_state == STATE_DISARMED)
            {
                if (!flight_control_is_armed())
                {
                    flight_control_arm();
                }
                s_flight_state = STATE_TAKEOFF_RAMP;
                flight_control_set_mode(FLIGHT_MODE_POS_HOLD);

                udp_send_log("[SYSTEM_LOG] -> TAKEOFF CMD INITIATED\n");
                ESP_LOGW(TAG, "CMD RECV: >>> AUTO TAKEOFF INITIATED <<<");
            }
        }
        else if (strncmp(rx_buffer, "LAND", 4) == 0)
        {
            if (s_flight_state == STATE_FLYING_MANUAL || s_flight_state == STATE_TAKEOFF_RAMP)
            {
                s_flight_state = STATE_LANDING_AUTO;
                udp_send_log("[SYSTEM_LOG] -> LAND CMD INITIATED\n");
                ESP_LOGW(TAG, "CMD RECV: >>> AUTO LANDING INITIATED <<<");
            }
        }
        else if (strncmp(rx_buffer, "EMERGENCY", 9) == 0)
        {
            s_flight_state = STATE_EMERGENCY;
            flight_control_disarm();
            udp_send_log("[ALERT_LOG] -> EMERGENCY STOP TRIGGERED!\n");
            ESP_LOGE(TAG, "CMD RECV: >>> EMERGENCY STOP TRIGGERED <<<");
        }
        // === XỬ LÝ LỆNH CALIB GYRO & RESET YAW VỀ 0.0 DEG ===
        else if (strncmp(rx_buffer, "CALIB", 5) == 0 || strncmp(rx_buffer, "RESET_GYRO", 10) == 0)
        {
            if (s_flight_state == STATE_DISARMED)
            {
                udp_send_log("[SYSTEM_LOG] -> CALIBRATING GYRO... KEEP DRONE STILL!\n");
                if (state_estimator_calibrate_gyro() == ESP_OK)
                {
                    udp_send_log("[SYSTEM_LOG] -> CALIB SUCCESS! Gyro Zeroed & Yaw Reset to 0.0 deg\n");
                    ESP_LOGW(TAG, "Gyro Calibrated Successfully via UDP Command.");
                }
                else
                {
                    udp_send_log("[ALERT_LOG] -> CALIB FAILED! Sensor Read Error\n");
                }
            }
            else
            {
                udp_send_log("[ALERT_LOG] -> REJECTED: Cannot Calibrate Gyro while ARMED/FLYING!\n");
            }
        }
        else if (strncmp(rx_buffer, "MOVE", 4) == 0)
        {
            if (s_flight_state == STATE_FLYING_MANUAL)
            {
                float r = 0.0f, p = 0.0f, y = 0.0f, t_or_alt = 0.0f;
                if (sscanf(rx_buffer, "MOVE %f %f %f %f", &r, &p, &y, &t_or_alt) == 4)
                {
                    s_setpoint.roll_deg = r;
                    s_setpoint.pitch_deg = p;
                    s_setpoint.yaw_rate_dps = y;

                    if (flight_control_is_armed())
                    {
                        s_setpoint.alt_z_m = t_or_alt;
                    }
                    flight_control_set_setpoint(&s_setpoint);
                }
            }
        }
    }
}

// =========================================================================
// 5. CHƯƠNG TRÌNH CHÍNH (APP MAIN LOOP)
// =========================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Wi-Fi Controlled Drone System...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    udp_socket_init();

    if (state_estimator_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "State Estimator Init Failed! System Halted.");
        return;
    }

    if (flight_control_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Flight Control Init Failed! System Halted.");
        return;
    }

    flight_control_disarm();

    s_setpoint.roll_deg = 0.0f;
    s_setpoint.pitch_deg = 0.0f;
    s_setpoint.yaw_rate_dps = 0.0f;
    s_setpoint.alt_z_m = 0.00f;
    s_setpoint.hover_throttle = 0.00f;
    flight_control_set_setpoint(&s_setpoint);

    uav_state_t state = {0};
    int64_t last_time = esp_timer_get_time();

    ESP_LOGI(TAG, "=== DRONE READY. WAITING FOR WIFI UDP COMMANDS... ===");

    while (1)
    {
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;
        if (dt <= 0.001f || dt > 0.05f)
            dt = 0.01f;

        // 1. Xử lý lệnh Wi-Fi UDP
        process_wifi_commands();

        // 2. Lấy dữ liệu cảm biến
        if (state_estimator_get_state(&state))
        {
            // 3. FAILSAFE: Mất kết nối Wi-Fi khi đang bay
            if (s_client_connected &&
                (s_flight_state == STATE_FLYING_MANUAL || s_flight_state == STATE_TAKEOFF_RAMP))
            {
                if (now - s_last_cmd_timestamp > WIFI_COMMAND_TIMEOUT_US)
                {
                    udp_send_log("[ALERT_LOG] -> WIFI SIGNAL LOST! ACTIVATING AUTO-LANDING\n");
                    ESP_LOGE(TAG, ">>> WIFI SIGNAL LOST! ACTIVATING EMERGENCY AUTO-LANDING <<<");
                    s_flight_state = STATE_LANDING_AUTO;
                }
            }

            // 4. BẢO VỆ NGẮT KHẨN CẤP PHẦN CỨNG (Inverted / Low Battery / Manual Disarm)
            if (!flight_control_is_armed() && s_flight_state != STATE_DISARMED)
            {
                s_flight_state = STATE_DISARMED;
                udp_send_log("[ALERT_LOG] -> DISARM TRIGGERED! Reason: %s | Vbat: %.2fV\n",
                             flight_control_get_disarm_reason_str(), flight_control_get_vbat());
            }

            // 5. THỰC THI FSM THEO LỆNH
            switch (s_flight_state)
            {
            case STATE_DISARMED:
            case STATE_EMERGENCY:
                flight_control_disarm();
                break;

            case STATE_ARMED_IDLE:
                s_setpoint.hover_throttle = IDLE_THROTTLE;
                s_setpoint.alt_z_m = 0.00f;
                flight_control_set_setpoint(&s_setpoint);
                break;

            case STATE_TAKEOFF_RAMP:
                // 1. Tăng ga dứt khoát 18%/s (10% -> 48% trong ~2.1s)
                if (s_setpoint.hover_throttle < TARGET_TAKEOFF_THROTTLE)
                {
                    s_setpoint.hover_throttle += THROTTLE_RAMP_RATE * dt;
                    if (s_setpoint.hover_throttle > TARGET_TAKEOFF_THROTTLE)
                        s_setpoint.hover_throttle = TARGET_TAKEOFF_THROTTLE;
                }

                // 2. Nâng setpoint độ cao Z song song từ 0m -> 0.30m (với tốc độ 0.15m/s)
                if (s_setpoint.alt_z_m < TARGET_TAKEOFF_ALT)
                {
                    s_setpoint.alt_z_m += 0.15f * dt;
                    if (s_setpoint.alt_z_m > TARGET_TAKEOFF_ALT)
                        s_setpoint.alt_z_m = TARGET_TAKEOFF_ALT;
                }
                flight_control_set_setpoint(&s_setpoint);

                // 3. Khi ga đạt 48% và độ cao tiệm cận 30cm -> Chuyển sang MANUAL bay tự do
                if (s_setpoint.hover_throttle >= TARGET_TAKEOFF_THROTTLE &&
                    s_setpoint.alt_z_m >= (TARGET_TAKEOFF_ALT - 0.05f))
                {
                    s_flight_state = STATE_FLYING_MANUAL;
                    udp_send_log("[SYSTEM_LOG] -> Takeoff Complete -> Entered MANUAL Mode\n");
                    ESP_LOGI(TAG, "Takeoff Complete -> Entered MANUAL FLYING Mode.");
                }
                break;

            case STATE_FLYING_MANUAL:
                break;

            case STATE_LANDING_AUTO:
                s_setpoint.alt_z_m -= 0.05f * dt;
                if (s_setpoint.alt_z_m < 0.00f)
                    s_setpoint.alt_z_m = 0.00f;

                if (s_setpoint.alt_z_m < 0.10f && s_setpoint.hover_throttle > IDLE_THROTTLE)
                {
                    s_setpoint.hover_throttle -= THROTTLE_RAMP_RATE * dt;
                    if (s_setpoint.hover_throttle < IDLE_THROTTLE)
                        s_setpoint.hover_throttle = IDLE_THROTTLE;
                }
                flight_control_set_setpoint(&s_setpoint);

                if (s_setpoint.alt_z_m <= 0.00f && (state.z < 0.08f || s_setpoint.hover_throttle <= IDLE_THROTTLE))
                {
                    s_flight_state = STATE_DISARMED;
                    s_setpoint.hover_throttle = 0.00f;
                    flight_control_set_setpoint(&s_setpoint);
                    flight_control_disarm();

                    udp_send_log("[SYSTEM_LOG] -> AUTO LANDING COMPLETED -> DISARMED\n");
                    ESP_LOGW(TAG, "=== AUTO LANDING COMPLETED -> DISARMED ===");
                }
                break;

            default:
                break;
            }

            // 6. Cập nhật vòng lặp PID
            flight_control_update(&state, dt);

            // 7. Gửi Telemetry định kỳ qua UDP
            udp_stream_telemetry(&state);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}