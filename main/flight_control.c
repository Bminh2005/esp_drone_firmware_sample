#include "flight_control.h"
#include "esp_log.h"
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.1415926535f)
#define HOVER_BASE_THRUST 0.45f // Xung PWM cơ bản ước tính để lơ lửng

typedef struct
{
    float kp, ki, kd;
    float i_term;
    float last_input;
    float i_max;
    float out_max;
} pid_t;

// Các bộ điều khiển PID phân tầng
static pid_t pid_alt_pos, pid_alt_vel;                    // Phân tầng Cao độ: Position Z -> Velocity Vz -> Thrust
static pid_t pid_vel_x, pid_vel_y;                        // Vận tốc ngang Optical Flow
static pid_t pid_angle_roll, pid_angle_pitch;             // Góc nghiêng Attitude
static pid_t pid_rate_roll, pid_rate_pitch, pid_rate_yaw; // Tốc độ góc Angular Rate

static float s_motor_pwm[4] = {0.0f};
static const char *TAG = "FLIGHT_CTRL";

static inline float pid_compute(pid_t *pid, float setpoint, float input, float dt)
{
    float error = setpoint - input;

    // Proportional
    float p_out = pid->kp * error;

    // Integral với Anti-windup
    pid->i_term += pid->ki * error * dt;
    if (pid->i_term > pid->i_max)
        pid->i_term = pid->i_max;
    if (pid->i_term < -pid->i_max)
        pid->i_term = -pid->i_max;

    // Derivative trên Input (Chống D-kick)
    float d_out = -pid->kd * (input - pid->last_input) / dt;
    pid->last_input = input;

    float output = p_out + pid->i_term + d_out;
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
}

void flight_control_init(void)
{
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

    // 2. KHOI TẠO THAM SỐ PID PHÂN TẦNG (CASCADED PID)
    // Altitude Outer Loop (Vị trí Z -> Vz Target m/s, giới hạn tốc độ leo max 0.05 m/s)
    pid_alt_pos = (pid_t){.kp = 1.8f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 0.35f};

    // Altitude Inner Loop (Vận tốc Vz -> Thrust Offset, Damping chống vọt ga)
    // pid_alt_vel = (pid_t){.kp = 0.25f, .ki = 0.05f, .kd = 0.03f, .i_max = 0.15f, .out_max = 0.25f};
    pid_alt_vel = (pid_t){.kp = 0.45f, .ki = 0.08f, .kd = 0.03f, .i_max = 0.25f, .out_max = 0.35f};

    // Velocity 2D (Flow Deck -> Pitch/Roll Angle Target)
    pid_vel_x = (pid_t){.kp = 10.0f, .ki = 1.0f, .kd = 0.4f, .i_max = 4.0f, .out_max = 12.0f};
    pid_vel_y = (pid_t){.kp = 10.0f, .ki = 1.0f, .kd = 0.4f, .i_max = 4.0f, .out_max = 12.0f};

    // Attitude Angle (Angle -> Angular Rate Target deg/s)
    pid_angle_roll = (pid_t){.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 180.0f};
    pid_angle_pitch = (pid_t){.kp = 6.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 180.0f};

    // Angular Rate (Rate -> Motor Torque Offset)
    pid_rate_roll = (pid_t){.kp = 0.0014f, .ki = 0.0004f, .kd = 0.000025f, .i_max = 0.08f, .out_max = 0.25f};
    pid_rate_pitch = (pid_t){.kp = 0.0014f, .ki = 0.0004f, .kd = 0.000025f, .i_max = 0.08f, .out_max = 0.25f};
    pid_rate_yaw = (pid_t){.kp = 0.0028f, .ki = 0.0002f, .kd = 0.000000f, .i_max = 0.08f, .out_max = 0.25f};

    ESP_LOGI(TAG, "LEDC Motors & Cascaded Vz Altitude Controller Initialized");
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
    // === BẢO VỆ 1: NGẮT KHẨN CẤP KHI LẬT GÓC > 55 DEG ===
    if (fabsf(state->roll_deg) > 55.0f || fabsf(state->pitch_deg) > 55.0f || !setpoint->armed)
    {
        flight_control_disarm_motors();
        return;
    }

    // === BẢO VỆ 2: UNSTICK PROTECTION (XẢ I-TERM DƯỚI 7CM SÀN) ===
    if (state->z < UNSTICK_ALTITUDE_M)
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

    // --- TẦNG 1: PHÂN TẦNG CAO ĐỘ DỰA TRÊN Vz (ALTITUDE POS -> VEL -> THRUST) ---
    // 1. Vòng ngoài: Vị trí Z tính ra tốc độ leo mong muốn Vz_target
    float z_error = setpoint->z_target - state->z;
    float vz_target = pid_alt_pos.kp * z_error;
    if (vz_target > pid_alt_pos.out_max)
        vz_target = pid_alt_pos.out_max;
    if (vz_target < -pid_alt_pos.out_max)
        vz_target = -pid_alt_pos.out_max;

    // 2. Vòng trong: Sai số vận tốc Vz điều khiển ga tăng vừa đủ
    float thrust_offset = pid_compute(&pid_alt_vel, vz_target, state->vz, dt);
    float target_thrust = HOVER_BASE_THRUST + thrust_offset;

    // --- TẦNG 2: POSHOLD OPTICAL FLOW (VELOCITY -> PITCH/ROLL ANGLE TARGET) ---
    float target_pitch_deg = -pid_compute(&pid_vel_x, setpoint->vx_target, state->vx, dt);
    float target_roll_deg = pid_compute(&pid_vel_y, setpoint->vy_target, state->vy, dt);

    // --- TẦNG 3: ATTITUDE ANGLE PID ---
    float target_rate_roll = pid_compute(&pid_angle_roll, target_roll_deg, state->roll_deg, dt);
    float target_rate_pitch = pid_compute(&pid_angle_pitch, target_pitch_deg, state->pitch_deg, dt);
    float target_rate_yaw = setpoint->yaw_rate_target;

    // --- TẦNG 4: ATTITUDE RATE PID ---
    float roll_rate_deg = state->gyro_x * RAD_TO_DEG;
    float pitch_rate_deg = state->gyro_y * RAD_TO_DEG;
    float yaw_rate_deg = state->gyro_z * RAD_TO_DEG;

    float u_roll = pid_compute(&pid_rate_roll, target_rate_roll, roll_rate_deg, dt);
    float u_pitch = pid_compute(&pid_rate_pitch, target_rate_pitch, pitch_rate_deg, dt);
    float u_yaw = pid_compute(&pid_rate_yaw, target_rate_yaw, yaw_rate_deg, dt);

    // --- TẦNG 5: MOTOR MIXER MATRIX (CHUẨN CRAZYFLIE / XINWEI E1) ---
    float motor_targets[4];
    // M1 CCW (Trước - Phải): Đẩy đầu xuống (-u_pitch), đẩy bên phải xuống (+u_roll)
    motor_targets[0] = target_thrust - u_pitch - u_roll - u_yaw;

    // M2 CW  (Sau - Phải)  : Đẩy đuôi xuống (+u_pitch), đẩy bên phải xuống (+u_roll)
    motor_targets[1] = target_thrust + u_pitch - u_roll + u_yaw;

    // M3 CCW (Sau - Trái)  : Đẩy đuôi xuống (+u_pitch), đẩy bên trái xuống (-u_roll)
    motor_targets[2] = target_thrust + u_pitch + u_roll - u_yaw;

    // M4 CW  (Trước - Trái) : Đẩy đầu xuống (-u_pitch), đẩy bên trái xuống (-u_roll)
    motor_targets[3] = target_thrust - u_pitch + u_roll + u_yaw;

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
}