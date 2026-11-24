#include "flight_control.h"
#include "esp_log.h"
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.1415926535f)
// Mức ga lơ lửng nền chuẩn hóa (45%)

typedef struct
{
    float kp, ki, kd;
    float i_term;
    float last_input;
    float d_lpf; // [CẢI TIẾN XINWEI] Lưu giá trị D-term đã qua bộ lọc LPF
    float i_max;
    float out_max;
} pid_t;
static float s_current_motor_targets[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// Các bộ điều khiển PID phân tầng
static pid_t pid_alt_pos, pid_alt_vel;
static pid_t pid_vel_x, pid_vel_y;
static pid_t pid_angle_roll, pid_angle_pitch;
static pid_t pid_rate_roll, pid_rate_pitch, pid_rate_yaw;

static float s_motor_pwm[4] = {0.0f};
static float s_inverted_timer = 0.0f; // Bộ đếm thời gian lật ngửa
static const char *TAG = "FLIGHT_CTRL";
static flight_ctrl_debug_t s_debug_data = {0};
// --- THUẬT TOÁN PID COMPUTATION ĐÃ NÂNG CẤP BỘ LỌC D-TERM LPF ---
static inline float pid_compute(pid_t *pid, float setpoint, float input, float dt)
{
    float error = setpoint - input;

    // 1. Proportional (P-term)
    float p_out = pid->kp * error;

    // 2. Integral (I-term) với Anti-Windup
    pid->i_term += pid->ki * error * dt;
    if (pid->i_term > pid->i_max)
        pid->i_term = pid->i_max;
    if (pid->i_term < -pid->i_max)
        pid->i_term = -pid->i_max;

    // 3. Derivative (D-term) trên Input + LPF 40Hz (Alpha = 0.2f)
    float d_raw = -pid->kd * (input - pid->last_input) / dt;
    pid->last_input = input;

    // Lọc hạ thông (Low-Pass Filter) loại bỏ nhiễu tần số cao từ động cơ
    pid->d_lpf += 0.2f * (d_raw - pid->d_lpf);

    // 4. Tổng hợp Output
    float output = p_out + pid->i_term + pid->d_lpf;
    if (output > pid->out_max)
        output = pid->out_max;
    if (output < -pid->out_max)
        output = -pid->out_max;

    return output;
}

static inline void pid_reset(pid_t *pid)
{
    pid->i_term = 0.0f;
    pid->last_input = 0.0f;
    pid->d_lpf = 0.0f;
}

void flight_control_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_1_GPIO) | (1ULL << MOTOR_2_GPIO) |
                        (1ULL << MOTOR_3_GPIO) | (1ULL << MOTOR_4_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(MOTOR_1_GPIO, 0);
    gpio_set_level(MOTOR_2_GPIO, 0);
    gpio_set_level(MOTOR_3_GPIO, 0);
    gpio_set_level(MOTOR_4_GPIO, 0);
    // 1. CẤU HÌNH PHẦN CỨNG LEDC PWM DRIVER CHO 4 ĐỘNG CƠ
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_PWM_RES,
        .freq_hz = LEDC_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    gpio_num_t motor_gpios[4] = {MOTOR_1_GPIO, MOTOR_2_GPIO, MOTOR_3_GPIO, MOTOR_4_GPIO};
    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = motor_gpios[i],
            .duty = 0,
            .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    // 2. CẤU HÌNH BỘ THAM SỐ PID TỐI ƯU (THAM CHIẾU XINWEI E1)
    // Phân tầng Cao độ Z (Vị trí Z -> Vz Target max 0.35 m/s)
    pid_alt_pos = (pid_t){.kp = 2.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 0.35f};
    // pid_alt_vel = (pid_t){.kp = 0.45f, .ki = 0.08f, .kd = 0.03f, .i_max = 0.25f, .out_max = 0.35f};
    pid_alt_vel = (pid_t){.kp = 0.55f, .ki = 0.15f, .kd = 0.03f, .i_max = 0.35f, .out_max = 0.45f};
    // Vận tốc ngang Optical Flow (Velocity -> Tilt Angle Target max 12 deg) (old)
    // pid_vel_x = (pid_t){.kp = 10.0f, .ki = 1.0f, .kd = 0.4f, .i_max = 4.0f, .out_max = 12.0f};
    // pid_vel_y = (pid_t){.kp = 10.0f, .ki = 1.0f, .kd = 0.4f, .i_max = 4.0f, .out_max = 12.0f};
    // pid_vel_x = (pid_t){.kp = 4.0f, .ki = 0.2f, .kd = 0.1f, .i_max = 2.0f, .out_max = 6.0f};
    // pid_vel_y = (pid_t){.kp = 4.0f, .ki = 0.2f, .kd = 0.1f, .i_max = 2.0f, .out_max = 6.0f};
    pid_vel_x = (pid_t){.kp = 3.5f, .ki = 0.1f, .kd = 0.05f, .i_max = 2.0f, .out_max = 5.0f};
    pid_vel_y = (pid_t){.kp = 3.5f, .ki = 0.1f, .kd = 0.05f, .i_max = 2.0f, .out_max = 5.0f};
    // Vòng ngoài Góc nghiêng (Angle -> Rate Target max 180 deg/s) - Kp = 7.0f chuẩn XinWEI
    pid_angle_roll = (pid_t){.kp = 7.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 180.0f};
    pid_angle_pitch = (pid_t){.kp = 7.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 180.0f};

    // Vòng trong Tốc độ góc (Rate -> Motor Torque Offset)
    pid_rate_roll = (pid_t){.kp = 0.0016f, .ki = 0.0004f, .kd = 0.000030f, .i_max = 0.08f, .out_max = 0.25f};
    pid_rate_pitch = (pid_t){.kp = 0.0016f, .ki = 0.0004f, .kd = 0.000030f, .i_max = 0.08f, .out_max = 0.25f};
    pid_rate_yaw = (pid_t){.kp = 0.0030f, .ki = 0.0002f, .kd = 0.000000f, .i_max = 0.08f, .out_max = 0.25f};

    ESP_LOGI(TAG, "LEDC Motors & LPF Filtered PID Controller Initialized");
    flight_control_disarm_motors();
}

void flight_control_disarm_motors(void)
{
    for (int i = 0; i < 4; i++)
    {
        s_motor_pwm[i] = 0.0f;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

void flight_control_step(const uav_state_t *state, const drone_setpoint_t *setpoint, float dt)
{
    // === BẢO VỆ 1: KHI TẮT ĐỘNG CƠ (DISARM) ===
    if (!setpoint->armed)
    {
        s_inverted_timer = 0.0f;
        flight_control_disarm_motors();
        return;
    }

    // === BẢO VỆ 2: BẢO VỆ NGỬA BỤNG THÔNG MINH (INVERTED FAILSAFE - CHỐNG NGẮT NHẦM) ===
    // Trục Z máy bay chiếu lên hệ tọa độ thế giới (cos_tilt)
    float cos_tilt = cosf(state->roll) * cosf(state->pitch);
    if (cos_tilt < 0.0f) // Góc nghiêng vượt quá 90 độ (bắt đầu lật ngửa)
    {
        s_inverted_timer += dt;
        if (s_inverted_timer > 0.3f) // Ngắt khẩn cấp sau 0.3 giây
        {
            ESP_LOGE(TAG, "Inverted Failsafe Triggered! Tilt > 90deg. Disarming.");
            flight_control_disarm_motors();
            return;
        }
    }
    else
    {
        s_inverted_timer = 0.0f;
    }

    // === BẢO VỆ 3: XẢ TÍCH PHÂN I-TERM KHI NẰM TRÊN SÀN (Z < 7CM HOẶC KHÔNG ĐẶT ĐỘ CAO) ===
    if (state->z < UNSTICK_ALTITUDE_M || setpoint->z_target <= 0.01f)
    {
        pid_reset(&pid_alt_pos);
        pid_reset(&pid_alt_vel);
        pid_reset(&pid_vel_x);
        pid_reset(&pid_vel_y);
        pid_reset(&pid_angle_roll);
        pid_reset(&pid_angle_pitch);
        pid_reset(&pid_rate_roll);
        pid_reset(&pid_rate_pitch);
        pid_reset(&pid_rate_yaw);
    }

    // --- TẦNG 1: PHÂN TẦNG CAO ĐỘ (ALTITUDE POS -> VEL -> THRUST) ---
    float z_error = setpoint->z_target - state->z;
    float vz_target = pid_alt_pos.kp * z_error;
    if (vz_target > pid_alt_pos.out_max)
        vz_target = pid_alt_pos.out_max;
    if (vz_target < -pid_alt_pos.out_max)
        vz_target = -pid_alt_pos.out_max;

    float thrust_offset = pid_compute(&pid_alt_vel, vz_target, state->vz, dt);
    // float target_thrust = (setpoint->z_target > 0.01f) ? (HOVER_BASE_THRUST + thrust_offset) : 0.05f; // Ga Idle 5% nếu z_target = 0
    if (thrust_offset > 0.12f)
        thrust_offset = 0.12f;
    if (thrust_offset < -0.12f)
        thrust_offset = -0.12f;

    float target_thrust = 0.08f; // Mặc định Ga Arm Idle 8%

    if (setpoint->armed)
    {
        if (setpoint->z_target > 0.01f)
        {
            // Đã phát lệnh cất cánh: Ga lơ lửng nền (ví dụ 0.60f cho pin nặng 23g) + bù PID Z
            target_thrust = HOVER_BASE_THRUST + thrust_offset;
            if (target_thrust > 0.75f)
                target_thrust = 0.75f;
        }
        else
        {
            // Đã Arm nhưng chưa cất cánh: Duy trì Ga Idle 8% để 4 motor quay đều chống shock
            target_thrust = 0.08f;
        }
    }

    //-- -TẦNG 2 : POSHOLD OPTICAL FLOW(VELOCITY->PITCH / ROLL ANGLE TARGET)-- -
    float target_pitch_deg = 0.0f;
    float target_roll_deg = 0.0f;

    if (setpoint->armed && state->z > 0.10f)
    {
        target_pitch_deg = pid_compute(&pid_vel_x, setpoint->vx_target, state->vx, dt);
        target_roll_deg = pid_compute(&pid_vel_y, setpoint->vy_target, state->vy, dt);
    }
    else
    {
        // Reset khâu I và đặt góc target bằng 0 khi còn ở mặt đất
        pid_reset(&pid_vel_x);
        pid_reset(&pid_vel_y);
        target_pitch_deg = 0.0f;
        target_roll_deg = 0.0f;
    }

    // // --- TẦNG 3: ATTITUDE ANGLE PID ---
    float target_rate_roll = pid_compute(&pid_angle_roll, target_roll_deg, state->roll_deg, dt);
    float target_rate_pitch = pid_compute(&pid_angle_pitch, target_pitch_deg, state->pitch_deg, dt);
    float target_rate_yaw = setpoint->yaw_rate_target;
    // TEST 4
    // --- TẦNG 4: ATTITUDE RATE PID ---
    float roll_rate_deg = state->gyro_x * RAD_TO_DEG;
    float pitch_rate_deg = state->gyro_y * RAD_TO_DEG;
    float yaw_rate_deg = state->gyro_z * RAD_TO_DEG;

    float u_roll = pid_compute(&pid_rate_roll, target_rate_roll, roll_rate_deg, dt);
    float u_pitch = pid_compute(&pid_rate_pitch, target_rate_pitch, pitch_rate_deg, dt);
    float u_yaw = pid_compute(&pid_rate_yaw, target_rate_yaw, yaw_rate_deg, dt);

    // --- TẦNG 5: MOTOR MIXER MATRIX (ĐÃ CHUẨN HÓA DẤU PHẢN HỒI ÂM) ---
    float motor_targets[4];
    motor_targets[0] = target_thrust - u_pitch - u_roll + u_yaw; // M1 CCW (Trước-Phải)
    motor_targets[1] = target_thrust + u_pitch - u_roll - u_yaw; // M2 CW  (Sau-Phải)
    motor_targets[2] = target_thrust + u_pitch + u_roll + u_yaw; // M3 CCW (Sau-Trái)
    motor_targets[3] = target_thrust - u_pitch + u_roll - u_yaw; // M4 CW  (Trước-Trái)
    s_current_motor_targets[0] = motor_targets[0];
    s_current_motor_targets[1] = motor_targets[1];
    s_current_motor_targets[2] = motor_targets[2];
    s_current_motor_targets[3] = motor_targets[3];

    for (int i = 0; i < 4; i++)
    {
        if (motor_targets[i] > MAX_MOTOR_PWM)
            motor_targets[i] = MAX_MOTOR_PWM;
        if (motor_targets[i] < MIN_MOTOR_PWM)
            motor_targets[i] = MIN_MOTOR_PWM;

        // Limiter gia tốc PWM chống sụt áp LDO 3.3V
        float max_step = MAX_PWM_SLEW_RATE * dt;
        float diff = motor_targets[i] - s_motor_pwm[i];
        if (diff > max_step)
            diff = max_step;
        if (diff < -max_step)
            diff = -max_step;

        s_motor_pwm[i] += diff;

        uint32_t duty = (uint32_t)(s_motor_pwm[i] * 2047.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
    s_debug_data.z_target = setpoint->z_target;
    s_debug_data.z_state = state->z;
    s_debug_data.u_z = thrust_offset;
    s_debug_data.vx_state = state->vx;
    s_debug_data.vy_state = state->vy;
    s_debug_data.target_pitch_deg = target_pitch_deg;
    s_debug_data.target_roll_deg = target_roll_deg;
    s_debug_data.roll_deg = state->roll_deg;
    s_debug_data.pitch_deg = state->pitch_deg;
    s_debug_data.target_rate_roll = target_rate_roll;
    s_debug_data.target_rate_pitch = target_rate_pitch;
    s_debug_data.gyro_x = roll_rate_deg;
    s_debug_data.gyro_y = pitch_rate_deg;
    s_debug_data.gyro_z = yaw_rate_deg;
    s_debug_data.u_roll = u_roll;
    s_debug_data.u_pitch = u_pitch;
    s_debug_data.u_yaw = u_yaw;
}

void flight_control_get_motors(float motors[4])
{
    if (motors == NULL)
        return;
    motors[0] = s_current_motor_targets[0];
    motors[1] = s_current_motor_targets[1];
    motors[2] = s_current_motor_targets[2];
    motors[3] = s_current_motor_targets[3];
}
void flight_control_get_debug(flight_ctrl_debug_t *out_debug)
{
    if (out_debug)
        *out_debug = s_debug_data;
}