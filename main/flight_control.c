#include "flight_control.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "FLIGHT_CONTROL";

// --- CHÂN NỐI ĐỘNG CƠ CHUẨN MẠCH HÃNG ---
// Index 0: Rear-Left  (RL) = GPIO 12
// Index 1: Rear-Right (RR) = GPIO 13
// Index 2: Front-Right(FR) = GPIO 15
// Index 3: Front-Left (FL) = GPIO 14
static const gpio_num_t MOTOR_PINS[4] = {
    GPIO_NUM_12, // MOTOR_REAR_LEFT  (Index 0)
    GPIO_NUM_13, // MOTOR_REAR_RIGHT (Index 1)
    GPIO_NUM_15, // MOTOR_FRONT_RIGHT(Index 2)
    GPIO_NUM_14  // MOTOR_FRONT_LEFT (Index 3)
};

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 0 -> 1023
#define LEDC_FREQ_HZ 78000              // 78kHz PWM Mosfet chuẩn hãng
#define MAX_PWM_OUTPUT 1023.0f

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

// --- THAM SỐ ĐIỆN ÁP PIN (TỪ battery.ino & esp-drone) ---
#define VBAT_ADC_CHANNEL ADC_CHANNEL_0 // GPIO 36 (ADC1_CH0)
#define VBAT_DIVIDER 1.342f            // Tỷ lệ phân áp thực tế hiệu chỉnh qua VOM
#define VBAT_NOMINAL 3.70f
#define VBAT_WARN_THRESHOLD 3.30f
#define VBAT_CRITICAL_THR 2.55f // ĐÃ SỬA: Hạ từ 2.75V xuống 2.55V cho pin 1S dưới tải xả
#define VBAT_MIN_CLAMP 2.50f
#define VBAT_LPF_ALPHA 0.08f

// --- THAM SỐ ĐỘNG CƠ VÀ AN TOÀN ---
static float s_mot_thr_min = 0.10f; // 10% Idle Thrust
static float s_mot_thr_max = 0.88f; // 95% Upper Duty Limit
#define MAX_PWM_SLEW_RATE 1.00f     // Giới hạn d(PWM)/dt = 180%/giây (chống sụt áp dòng xả)

static bool s_is_armed = false;
static flight_mode_t s_current_mode = FLIGHT_MODE_DISARMED;
static uav_setpoint_t s_setpoint = {0};

static float s_current_motor_pwm[4] = {0.10f, 0.10f, 0.10f, 0.10f};
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static float s_battery_voltage = VBAT_NOMINAL;

// Cấu trúc PID mở rộng bổ sung bộ lọc LPF cho D-term
typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;
    float i_term;
    float prev_error;
    float d_term_lpf; // Lọc LPF vi phân D-term
} pid_controller_ext_t;

// PID Angular Rate (Vòng trong cùng - Gyro Rate PID)
static pid_controller_ext_t pid_rate_roll = {.kp = 0.06f, .ki = 0.20f, .kd = 0.002f, .i_max = 0.35f, .out_max = 1.0f};
static pid_controller_ext_t pid_rate_pitch = {.kp = 0.05f, .ki = 0.20f, .kd = 0.001f, .i_max = 0.35f, .out_max = 1.0f};
static pid_controller_ext_t pid_rate_yaw = {.kp = 0.30f, .ki = 0.01f, .kd = 0.010f, .i_max = 0.30f, .out_max = 1.0f};

// PID Angle (Vòng ngoài - Euler Angle PID)
static pid_controller_ext_t pid_angle_roll = {.kp = 7.00f, .ki = 0.00f, .kd = 0.000f, .i_max = 0.00f, .out_max = 6.28f};
static pid_controller_ext_t pid_angle_pitch = {.kp = 7.00f, .ki = 0.00f, .kd = 0.000f, .i_max = 0.00f, .out_max = 6.28f};

// PID Altitude Z
static pid_controller_ext_t pid_alt_z = {.kp = 1.00f, .ki = 0.10f, .kd = 0.30f, .i_max = 0.15f, .out_max = 0.25f};

static disarm_reason_t s_disarm_reason = DISARM_REASON_NONE;

float flight_control_get_vbat(void) { return s_battery_voltage; }
disarm_reason_t flight_control_get_disarm_reason(void) { return s_disarm_reason; }

const char *flight_control_get_disarm_reason_str(void)
{
    switch (s_disarm_reason)
    {
    case DISARM_REASON_LOW_BATTERY:
        return "LOW_BATTERY_SAG";
    case DISARM_REASON_INVERTED:
        return "INVERTED_ANGLE_>60DEG";
    case DISARM_REASON_USER_CMD:
        return "USER_DISARM_CMD";
    case DISARM_REASON_WIFI_LOST:
        return "WIFI_SIGNAL_LOST";
    default:
        return "NONE";
    }
}

void flight_control_disarm_with_reason(disarm_reason_t reason)
{
    s_disarm_reason = reason;
    flight_control_disarm();
}

// Hàm tính PID có lọc LPF cho khâu vi phân D-term
static float pid_compute_ext(pid_controller_ext_t *pid, float error, float dt, bool use_d_lpf)
{
    if (dt <= 0.001f)
        dt = 0.01f;

    float p_term = pid->kp * error;

    pid->i_term += pid->ki * error * dt;
    if (pid->i_term > pid->i_max)
        pid->i_term = pid->i_max;
    if (pid->i_term < -pid->i_max)
        pid->i_term = -pid->i_max;

    float d_term_raw = 0.0f;
    if (dt > 0.001f)
    {
        d_term_raw = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    // Lọc LPF 40Hz cho D-term (RATES_D_LPF_ALPHA = 0.2)
    if (use_d_lpf)
    {
        pid->d_term_lpf = pid->d_term_lpf + 0.20f * (d_term_raw - pid->d_term_lpf);
    }
    else
    {
        pid->d_term_lpf = d_term_raw;
    }

    float output = p_term + pid->i_term + pid->d_term_lpf;
    if (output > pid->out_max)
        output = pid->out_max;
    if (output < -pid->out_max)
        output = -pid->out_max;

    return output;
}

static void pid_reset_ext(pid_controller_ext_t *pid)
{
    pid->i_term = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_term_lpf = 0.0f;
}

// --- HÀM ĐỌC VÀ LỌC LPF ĐIỆN ÁP PIN ---
static float read_battery_voltage_internal(void)
{
    if (s_adc_handle == NULL)
        return VBAT_NOMINAL;

    int sum_raw = 0;
    const int samples = 16;
    for (int i = 0; i < samples; i++)
    {
        int raw = 0;
        adc_oneshot_read(s_adc_handle, VBAT_ADC_CHANNEL, &raw);
        sum_raw += raw;
    }
    float avg_raw = (float)sum_raw / (float)samples;
    float v_adc = (avg_raw / 4095.0f) * 3.3f;
    return v_adc * VBAT_DIVIDER;
}

esp_err_t flight_control_init(void)
{
    ESP_LOGI(TAG, "Initializing Flight Control Hardware PWM (78kHz Mosfet)...");

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = MOTOR_PINS[i],
            .duty = 0,
            .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        s_current_motor_pwm[i] = 0.0f;
    }

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    if (adc_oneshot_new_unit(&init_config1, &s_adc_handle) == ESP_OK)
    {
        adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_12,
        };
        adc_oneshot_config_channel(s_adc_handle, VBAT_ADC_CHANNEL, &config);
        ESP_LOGI(TAG, "Battery Voltage Monitor ADC Initialized.");
    }

    s_battery_voltage = read_battery_voltage_internal();
    s_is_armed = false;
    s_current_mode = FLIGHT_MODE_DISARMED;
    ESP_LOGI(TAG, "Flight Control Initialized Successfully!");
    return ESP_OK;
}

void flight_control_arm(void)
{
    // Kiểm tra điện áp trước khi ARM: Nếu pin dưới 3.30V thì cấm ARM
    float raw_vbat = read_battery_voltage_internal();
    if (raw_vbat > 0.5f && raw_vbat < VBAT_WARN_THRESHOLD)
    {
        s_disarm_reason = DISARM_REASON_LOW_BATTERY;
        ESP_LOGE(TAG, ">>> CANNOT ARM! BATTERY LOW (%.2fV < %.2fV) <<<", raw_vbat, VBAT_WARN_THRESHOLD);
        flight_control_disarm();
        return;
    }

    s_is_armed = true;
    if (s_current_mode == FLIGHT_MODE_DISARMED)
    {
        s_current_mode = FLIGHT_MODE_STABILIZE;
    }

    pid_reset_ext(&pid_rate_roll);
    pid_reset_ext(&pid_rate_pitch);
    pid_reset_ext(&pid_rate_yaw);
    pid_reset_ext(&pid_angle_roll);
    pid_reset_ext(&pid_angle_pitch);
    pid_reset_ext(&pid_alt_z);

    for (int i = 0; i < 4; i++)
    {
        s_current_motor_pwm[i] = s_mot_thr_min;
    }

    ESP_LOGW(TAG, ">>> SYSTEM ARMED! MOTORS ACTIVE <<<");
}

void flight_control_disarm(void)
{
    s_is_armed = false;
    s_current_mode = FLIGHT_MODE_DISARMED;

    for (int i = 0; i < 4; i++)
    {
        s_current_motor_pwm[i] = 0.0f;
        ledc_set_duty(LEDC_MODE, (ledc_channel_t)i, 0);
        ledc_update_duty(LEDC_MODE, (ledc_channel_t)i);
    }
    ESP_LOGW(TAG, ">>> SYSTEM DISARMED! MOTORS OFF <<<");
}

bool flight_control_is_armed(void) { return s_is_armed; }
void flight_control_set_mode(flight_mode_t mode) { s_current_mode = mode; }
void flight_control_set_setpoint(const uav_setpoint_t *setpoint)
{
    if (setpoint != NULL)
        s_setpoint = *setpoint;
}

static inline float thrust_to_pwm_linear(float thrust)
{
    if (thrust <= 0.0f)
        return 0.0f;
    return sqrtf(thrust);
}

static void handle_thrust_saturation(float motors[4])
{
    float max_m = motors[0];
    for (int i = 1; i < 4; i++)
    {
        if (motors[i] > max_m)
            max_m = motors[i];
    }

    if (max_m > 1.0f)
    {
        float shift = max_m - 1.0f;
        for (int i = 0; i < 4; i++)
        {
            motors[i] -= shift;
        }
    }

    float min_m = motors[0];
    for (int i = 1; i < 4; i++)
    {
        if (motors[i] < min_m)
            min_m = motors[i];
    }

    if (min_m < s_mot_thr_min)
    {
        float shift = s_mot_thr_min - min_m;
        for (int i = 0; i < 4; i++)
        {
            motors[i] += shift;
        }
    }
}

static void apply_motor_slew_limit(const float target_motors[4], float dt)
{
    float max_change = MAX_PWM_SLEW_RATE * dt;
    for (int i = 0; i < 4; i++)
    {
        float diff = target_motors[i] - s_current_motor_pwm[i];
        if (diff > max_change)
        {
            s_current_motor_pwm[i] += max_change;
        }
        else if (diff < -max_change)
        {
            s_current_motor_pwm[i] -= max_change;
        }
        else
        {
            s_current_motor_pwm[i] = target_motors[i];
        }
    }
}

// --- VÒNG LẶP ĐIỀU KHIỂN CHÍNH (FLIGHT CONTROL LOOP) ---
void flight_control_update(const uav_state_t *state, float dt)
{
    if (!s_is_armed || s_current_mode == FLIGHT_MODE_DISARMED || state == NULL)
    {
        s_disarm_reason = DISARM_REASON_USER_CMD;
        flight_control_disarm();
        return;
    }

    // 1. ĐỌC VÀ LỌC LPF ĐIỆN ÁP PIN TRÁNH SỤT ÁP TỨC THỜI CHẤP CHỚN
    float raw_vbat = read_battery_voltage_internal();
    if (raw_vbat > 0.5f)
    {
        s_battery_voltage = (s_battery_voltage * (1.0f - VBAT_LPF_ALPHA)) + (raw_vbat * VBAT_LPF_ALPHA);
    }

    float base_thrust = s_setpoint.hover_throttle;
    bool is_flying = (base_thrust > 0.15f);

    // 2. LOGIC FAILSAFE PIN 2 TẦNG (IDLE VS FLYING)
    // 2. LOGIC FAILSAFE PIN 2 TẦNG (ĐÃ SỬA CHỐNG NGẮT NGÁNG KHI AUTO-LANDING)
    static uint8_t s_low_bat_counter = 0;
    if (is_flying)
    {
        // Khi đang bay: Ngắt khẩn cấp nếu áp LPF < 2.75V duy trì quá 500ms (50 mẫu)
        if (s_battery_voltage < VBAT_CRITICAL_THR)
        {
            s_low_bat_counter++;
            if (s_low_bat_counter >= 50)
            {
                s_disarm_reason = DISARM_REASON_LOW_BATTERY;
                ESP_LOGE(TAG, ">>> CRITICAL BATTERY SAG (%.2fV < 2.75V)! EMERGENCY DISARM <<<", s_battery_voltage);
                flight_control_disarm();
                s_low_bat_counter = 0;
                return;
            }
        }
        else
        {
            s_low_bat_counter = 0;
        }
    }
    else
    {
        // Khi ở dải Idle / Tiếp đất: Hạ ngưỡng xuống 3.00V và thêm bộ lọc Debounce 300ms
        // Giúp drone hoàn tất quá trình Auto-Landing mượt mà mà không bị DISARM giữa chừng ở 3.25V
        if (s_battery_voltage < 3.00f)
        {
            s_low_bat_counter++;
            if (s_low_bat_counter >= 30) // 30 mẫu = 300ms
            {
                s_disarm_reason = DISARM_REASON_LOW_BATTERY;
                ESP_LOGE(TAG, ">>> LOW BATTERY IDLE (%.2fV < 3.00V)! DISARM <<<", s_battery_voltage);
                flight_control_disarm();
                s_low_bat_counter = 0;
                return;
            }
        }
        else
        {
            s_low_bat_counter = 0;
        }
    }

    // 3. INVERTED FAILSAFE (NGẮT KHẨN CẤP KHI NGHIÊNG > 60 ĐỘ)
    if (fabsf(state->roll_deg) > 60.0f || fabsf(state->pitch_deg) > 60.0f)
    {
        s_disarm_reason = DISARM_REASON_INVERTED;
        ESP_LOGE(TAG, ">>> INVERTED PROTECTION TRIGGERED! DISARMING... <<<");
        flight_control_disarm();
        return;
    }

    // 4. KHI GA Ở MỨC IDLE/RỐT-ĐA (<= 12%): TẮT HOÀN TOÀN PID CHỐNG WINDUP TRÊN MẶT ĐẤT
    // 4. CHỐNG WINDUP TRÊN MẶT ĐẤT (GROUND UNSTICK PROTECTION)
    // Nếu ga thấp HOẶC cảm biến báo drone vẫn chưa rời sàn (Z < 0.07m), liên tục reset I-term PID
    bool is_on_ground = (state->z < 0.07f) || (base_thrust <= 0.12f);
    if (is_on_ground)
    {
        pid_rate_roll.i_term = 0.0f;
        pid_rate_pitch.i_term = 0.0f;
        pid_rate_yaw.i_term = 0.0f;
        pid_angle_roll.i_term = 0.0f;
        pid_angle_pitch.i_term = 0.0f;
        pid_alt_z.i_term = 0.0f;
    }

    if (base_thrust <= (s_mot_thr_min + 0.02f) || base_thrust <= 0.12f)
    {
        pid_reset_ext(&pid_rate_roll);
        pid_reset_ext(&pid_rate_pitch);
        pid_reset_ext(&pid_rate_yaw);
        pid_reset_ext(&pid_angle_roll);
        pid_reset_ext(&pid_angle_pitch);
        pid_reset_ext(&pid_alt_z);

        for (int i = 0; i < 4; i++)
        {
            s_current_motor_pwm[i] = s_mot_thr_min;
            uint32_t duty = (uint32_t)(s_mot_thr_min * MAX_PWM_OUTPUT);
            ledc_set_duty(LEDC_MODE, (ledc_channel_t)i, duty);
            ledc_update_duty(LEDC_MODE, (ledc_channel_t)i);
        }
        return;
    }

    // 5. TẦNG ALTITUDE PID
    if (s_current_mode == FLIGHT_MODE_ALT_HOLD || s_current_mode == FLIGHT_MODE_POS_HOLD)
    {
        float err_z = s_setpoint.alt_z_m - state->z;
        float alt_adj = pid_compute_ext(&pid_alt_z, err_z, dt, false);
        base_thrust += alt_adj;
    }

    // 6. TẦNG ANGLE PID (VÒNG NGOÀI)
    float target_roll_rad = s_setpoint.roll_deg * DEG_TO_RAD;
    float target_pitch_rad = s_setpoint.pitch_deg * DEG_TO_RAD;

    float err_angle_roll = target_roll_rad - state->roll;
    float err_angle_pitch = target_pitch_rad - state->pitch;

    float target_rate_roll = pid_compute_ext(&pid_angle_roll, err_angle_roll, dt, false);
    float target_rate_pitch = pid_compute_ext(&pid_angle_pitch, err_angle_pitch, dt, false);
    float target_rate_yaw = s_setpoint.yaw_rate_dps * DEG_TO_RAD;

    // 7. TẦNG RATE PID (VÒNG TRONG - TORQUE TARGET CO LỌC LPF D-TERM)
    float err_rate_roll = target_rate_roll - state->gyro_x;
    float err_rate_pitch = target_rate_pitch - state->gyro_y;
    float err_rate_yaw = target_rate_yaw - state->gyro_z;

    float torque_x = pid_compute_ext(&pid_rate_roll, err_rate_roll, dt, true);
    float torque_y = pid_compute_ext(&pid_rate_pitch, err_rate_pitch, dt, true);
    float torque_z = pid_compute_ext(&pid_rate_yaw, err_rate_yaw, dt, true);

    // 8. MIXER CỦA HÃNG CHUẨN
    // Index 0: RL, Index 1: RR, Index 2: FR, Index 3: FL
    float raw_motors[4];
    raw_motors[0] = base_thrust + torque_x + torque_y + torque_z; // RL
    raw_motors[1] = base_thrust - torque_x + torque_y - torque_z; // RR
    raw_motors[2] = base_thrust - torque_x - torque_y + torque_z; // FR
    raw_motors[3] = base_thrust + torque_x - torque_y - torque_z; // FL

    // 9. XỬ LÝ BÃO HÒA THRUST & ƯU TIÊN GIỮ GÓC (esp-drone)
    handle_thrust_saturation(raw_motors);

    // 10. TUYẾN TÍNH HÓA PWM THEO LỰC ĐẨY VÀ BÙ NGUỒN PIN THEO DỮ LIỆU ĐÃ LỌC LPF
    float vbat_comp = s_battery_voltage;
    if (vbat_comp < VBAT_MIN_CLAMP)
        vbat_comp = VBAT_MIN_CLAMP;

    float vbat_factor = VBAT_NOMINAL / vbat_comp; // Tỷ lệ bù điện áp phẳng

    // CHỐNG VÒNG LẶP VỌT DÒNG: Khống chế hệ số bù không quá 1.20x (tối đa bù 20% ga)
    if (vbat_factor > 1.20f)
    {
        vbat_factor = 1.20f;
    }

    float target_pwm[4];
    for (int i = 0; i < 4; i++)
    {
        float pwm_lin = thrust_to_pwm_linear(raw_motors[i]);
        float pwm_final = pwm_lin * vbat_factor;

        if (pwm_final < s_mot_thr_min)
            pwm_final = s_mot_thr_min;
        if (pwm_final > s_mot_thr_max)
            pwm_final = s_mot_thr_max;

        target_pwm[i] = pwm_final;
    }
    // 11. GIỚI HẠN d(PWM)/dt CHỐNG SỤT ÁP TỨC THỜI (esp-drone)
    apply_motor_slew_limit(target_pwm, dt);

    // 12. XUẤT PWM RA DRIVER LEDC
    for (int i = 0; i < 4; i++)
    {
        uint32_t pwm_duty = (uint32_t)(s_current_motor_pwm[i] * MAX_PWM_OUTPUT);
        ledc_set_duty(LEDC_MODE, (ledc_channel_t)i, pwm_duty);
        ledc_update_duty(LEDC_MODE, (ledc_channel_t)i);
    }
}