#include "flight_control.h"
#include "esp_log.h"
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.1415926535f)
#define DEG_TO_RAD (3.1415926535f / 180.0f)
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

    // 2. CẤU HÌNH BỘ THAM SỐ PID TỐI ƯU (CHUẨN XINWEI E1)

    // --- TẦNG 1: Phân tầng Cao độ Z (Giới hạn thrust_offset tối đa +15% PWM) ---
    pid_alt_pos = (pid_t){.kp = 2.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 0.35f};
    pid_alt_vel = (pid_t){.kp = 0.40f, .ki = 0.05f, .kd = 0.02f, .i_max = 0.10f, .out_max = 0.15f};

    // --- TẦNG 2: Vận tốc ngang Optical Flow (Nâng góc nghiêng mục tiêu lên 10 deg để hãm đà tốt) ---
    // pid_vel_x = (pid_t){.kp = 3.5f, .ki = 0.10f, .kd = 0.05f, .i_max = 2.0f, .out_max = 10.0f};
    // pid_vel_y = (pid_t){.kp = 3.5f, .ki = 0.10f, .kd = 0.05f, .i_max = 2.0f, .out_max = 10.0f};
    pid_vel_x = (pid_t){.kp = 0.80f, .ki = 0.00f, .kd = 0.000f, .i_max = 0.0f, .out_max = 4.0f};
    pid_vel_y = (pid_t){.kp = 0.80f, .ki = 0.00f, .kd = 0.000f, .i_max = 0.0f, .out_max = 4.0f};

    // --- TẦNG 3: Góc nghiêng Attitude (Angle -> Rate Target max 180 deg/s) ---
    pid_angle_roll = (pid_t){.kp = 7.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 180.0f};
    pid_angle_pitch = (pid_t){.kp = 7.0f, .ki = 0.0f, .kd = 0.0f, .i_max = 0.0f, .out_max = 180.0f};

    // --- TẦNG 4: Tốc độ góc Attitude Rate (Siêu kẹp biên Yaw out_max = 0.08f) ---
    // pid_rate_roll = (pid_t){.kp = 0.0016f, .ki = 0.0004f, .kd = 0.000030f, .i_max = 0.08f, .out_max = 0.25f};
    // pid_rate_pitch = (pid_t){.kp = 0.0016f, .ki = 0.0004f, .kd = 0.000030f, .i_max = 0.08f, .out_max = 0.25f};
    pid_rate_roll = (pid_t){.kp = 0.0022f, .ki = 0.0005f, .kd = 0.000040f, .i_max = 0.10f, .out_max = 0.30f};
    pid_rate_pitch = (pid_t){.kp = 0.0022f, .ki = 0.0005f, .kd = 0.000040f, .i_max = 0.10f, .out_max = 0.30f};
    pid_rate_yaw = (pid_t){.kp = 0.0025f, .ki = 0.0001f, .kd = 0.000000f, .i_max = 0.04f, .out_max = 0.08f};

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

// Helper hàm kẹp giá trị inline gọn gàng
static inline float clampf(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
void flight_control_step(const uav_state_t *state, const drone_setpoint_t *setpoint, float dt)
{
    s_debug_data.z_target = setpoint->z_target;
    s_debug_data.z_state = state->z;
    s_debug_data.vx_state = state->vx;
    s_debug_data.vy_state = state->vy;
    s_debug_data.roll_deg = state->roll_deg;
    s_debug_data.pitch_deg = state->pitch_deg;
    s_debug_data.gyro_x = state->gyro_x * RAD_TO_DEG;
    s_debug_data.gyro_y = state->gyro_y * RAD_TO_DEG;
    s_debug_data.gyro_z = state->gyro_z * RAD_TO_DEG;
    // === BẢO VỆ 1: KHI TẮT ĐỘNG CƠ (DISARM) ===
    if (!setpoint->armed)
    {
        s_inverted_timer = 0.0f;
        // Xóa các giá trị đầu ra điều khiển debug về 0
        s_debug_data.u_z = 0.0f;
        s_debug_data.target_pitch_deg = 0.0f;
        s_debug_data.target_roll_deg = 0.0f;
        s_debug_data.target_rate_roll = 0.0f;
        s_debug_data.target_rate_pitch = 0.0f;
        s_debug_data.u_roll = 0.0f;
        s_debug_data.u_pitch = 0.0f;
        s_debug_data.u_yaw = 0.0f;

        flight_control_disarm_motors();
        return;
    }

    // === BẢO VỆ 2: BẢO VỆ NGỬA BỤNG NGHĨA PHỤC (INVERTED FAILSAFE - 0.3s TIMEOUT) ===
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

    // === BẢO VỆ 3: XẢ TÍCH PHÂN I-TERM KHI NẰM TRÊN SÀN ===
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

    // // --- TẦNG 1: CAO ĐỘ (ALTITUDE POS -> VEL -> THRUST) ---
    // float z_error = setpoint->z_target - state->z;
    // float vz_target = pid_alt_pos.kp * z_error;
    // if (vz_target > pid_alt_pos.out_max)
    //     vz_target = pid_alt_pos.out_max;
    // if (vz_target < -pid_alt_pos.out_max)
    //     vz_target = -pid_alt_pos.out_max;

    // float thrust_offset = pid_compute(&pid_alt_vel, vz_target, state->vz, dt);

    // // Đảm bảo thrust_offset không làm vọt ga nền quá mức (+/- 12% PWM max)
    // if (thrust_offset > 0.12f)
    //     thrust_offset = 0.12f;
    // if (thrust_offset < -0.12f)
    //     thrust_offset = -0.12f;

    // float target_thrust = 0.08f; // Ga Idle 8% khi ở mặt đất

    // if (setpoint->z_target > 0.01f)
    // {
    //     target_thrust = HOVER_BASE_THRUST + thrust_offset;
    //     // Giới hạn ga nền tối đa 75% để luôn dành 25% headroom cho Roll/Pitch
    //     if (target_thrust > 0.75f)
    //         target_thrust = 0.75f;
    // }

    // // =========================================================================
    // // --- TẦNG 1: CAO ĐỘ (ALTITUDE POS -> VEL -> THRUST) [OPTIMIZED] ---
    // // =========================================================================

    // // 1. Ramp z_target để cất/hạ cánh từ từ, triệt tiêu xung ga dội sàn (Ground Effect)
    // static float s_ramped_z_target = 0.0f;

    // if (setpoint->armed && setpoint->z_target > 0.01f)
    // {
    //     if (s_ramped_z_target < setpoint->z_target)
    //     {
    //         s_ramped_z_target += 0.30f * dt; // Tăng dần 30cm/s khi cất cánh
    //         if (s_ramped_z_target > setpoint->z_target)
    //             s_ramped_z_target = setpoint->z_target;
    //     }
    //     else if (s_ramped_z_target > setpoint->z_target)
    //     {
    //         s_ramped_z_target -= 0.40f * dt; // Hạ dần 40cm/s khi hạ cánh
    //         if (s_ramped_z_target < setpoint->z_target)
    //             s_ramped_z_target = setpoint->z_target;
    //     }
    // }
    // else
    // {
    //     s_ramped_z_target = 0.0f;
    // }

    // float target_thrust = 0.0f;

    // if (setpoint->armed)
    // {
    //     if (s_ramped_z_target > 0.01f)
    //     {
    //         // P-Control: Tính vận tốc Z mục tiêu từ sai số cao độ đã Ramp
    //         float z_error = s_ramped_z_target - state->z;
    //         float vz_target = clampf(pid_alt_pos.kp * z_error, -pid_alt_pos.out_max, pid_alt_pos.out_max);

    //         // PID Control: Tính Thrust Offset bù ga
    //         float thrust_offset = pid_compute(&pid_alt_vel, vz_target, state->vz, dt);

    //         // Mở rộng giới hạn bù ga lên +/- 0.20 (20%) để thích ứng khi sụt áp pin LiPo 1S
    //         thrust_offset = clampf(thrust_offset, -0.20f, 0.20f);

    //         // Tính ga nền cơ bản
    //         target_thrust = HOVER_BASE_THRUST + thrust_offset;

    //         // 2. BÙ GÓC NGHIÊNG (Attitude Tilt Compensation)
    //         // Khi drone nghiêng Roll/Pitch để hãm trôi, lực nâng theo phương đứng bị giảm theo cos(theta).
    //         // Phép chia này giúp giữ nguyên độ cao Z ngay cả khi PosHold làm drone nghiêng gắt.
    //         float cos_roll = cosf(state->roll_deg * DEG_TO_RAD);
    //         float cos_pitch = cosf(state->pitch_deg * DEG_TO_RAD);
    //         float tilt_comp = cos_roll * cos_pitch;

    //         // Kẹp giới hạn cos_tilt >= 0.707 (tương đương nghiêng max 45 deg) để tránh chia cho 0
    //         if (tilt_comp < 0.707f)
    //             tilt_comp = 0.707f;
    //         target_thrust /= tilt_comp;

    //         if (target_thrust > 0.60f)
    //         {
    //             target_thrust = 0.60f;
    //         }

    //         // -----------------------------------------------------------------
    //         // CHÈN MỤC 1 Ở ĐÂY: KHÓA GA & RESET PID KHI CÒN TRÊN SÀN (Z < 0.05m)
    //         // -----------------------------------------------------------------
    //         if (state->z < 0.05f)
    //         {
    //             // Khống chế ga nền tối đa 25% khi sát đất để không bị bão hòa ga
    //             if (target_thrust > 0.25f)
    //             {
    //                 target_thrust = 0.25f;
    //             }

    //             // Reset I-term các tầng góc nghiêng để không bị tích lũy lực bật ảo trên sàn
    //             pid_reset(&pid_angle_roll);
    //             pid_reset(&pid_angle_pitch);
    //             pid_reset(&pid_rate_roll);
    //             pid_reset(&pid_rate_pitch);
    //         }
    //         // Giới hạn ga nền trong khoảng [5% - 75%] để luôn dành 25% Headroom cho Roll/Pitch/Yaw
    //         target_thrust = clampf(target_thrust, 0.05f, 0.6f);
    //     }
    //     else
    //     {
    //         // Khi Armed nhưng chưa cất cánh (z_target = 0): Chạy ga Idle và Reset PID
    //         pid_reset(&pid_alt_vel);
    //         target_thrust = 0.06f; // 6% PWM quay nhẹ cánh báo hiệu sẵn sàng
    //     }
    // }
    // else
    // {
    //     // Disarmed -> Ngắt hoàn toàn ga và xóa tích lũy I-term
    //     pid_reset(&pid_alt_vel);
    //     target_thrust = 0.0f;
    // }

    // =========================================================================
    // TẦNG 1: CAO ĐỘ (ALTITUDE POS -> VEL -> THRUST)
    // =========================================================================
    static float s_ramped_z_target = 0.0f;
    float target_thrust = 0.0f;

    if (setpoint->armed)
    {
        if (s_ramped_z_target > 0.01f)
        {
            float z_error = s_ramped_z_target - state->z;
            float vz_target = clampf(pid_alt_pos.kp * z_error, -pid_alt_pos.out_max, pid_alt_pos.out_max);
            float thrust_offset = pid_compute(&pid_alt_vel, vz_target, state->vz, dt);
            thrust_offset = clampf(thrust_offset, -0.20f, 0.20f);

            target_thrust = HOVER_BASE_THRUST + thrust_offset;

            // 1. Bù góc nghiêng (Tilt Compensation)
            float cos_roll = cosf(state->roll_deg * DEG_TO_RAD);
            float cos_pitch = cosf(state->pitch_deg * DEG_TO_RAD);
            float tilt_comp = cos_roll * cos_pitch;
            if (tilt_comp < 0.707f)
                tilt_comp = 0.707f;
            target_thrust /= tilt_comp;

            // 2. Kẹp an toàn dải ga bay [5% - 60%] (Dành 40% Headroom cho Roll/Pitch)
            target_thrust = clampf(target_thrust, 0.05f, 0.60f);

            // 3. Khóa ga nền & Reset I-term khi drone còn nằm sát sàn (Z < 0.05m)
            if (state->z < 0.05f)
            {
                // Ép ga nền tối đa 25% sát mặt đất để không bị nạp vọt ga dội sàn
                if (target_thrust > 0.25f)
                {
                    target_thrust = 0.25f;
                }

                // Reset I-term các tầng góc nghiêng tránh tích lũy lực bật ảo trên sàn
                pid_reset(&pid_angle_roll);
                pid_reset(&pid_angle_pitch);
                pid_reset(&pid_rate_roll);
                pid_reset(&pid_rate_pitch);
            }
        }
        else
        {
            pid_reset(&pid_alt_vel);
            target_thrust = 0.06f; // Ga Idle 6% khi nằm sàn
        }
    }
    else
    {
        pid_reset(&pid_alt_vel);
        target_thrust = 0.0f; // Disarmed -> Tắt hoàn toàn ga
    }
    // --- TẦNG 2: POSHOLD OPTICAL FLOW ---
    float target_pitch_deg = 0.0f;
    float target_roll_deg = 0.0f;

    if (state->z > UNSTICK_ALTITUDE_M)
    {
        target_pitch_deg = -pid_compute(&pid_vel_x, setpoint->vx_target, state->vx, dt);
        target_roll_deg = -pid_compute(&pid_vel_y, setpoint->vy_target, state->vy, dt);
    }
    else
    {
        pid_reset(&pid_vel_x);
        pid_reset(&pid_vel_y);
    }
    static float s_filtered_target_pitch = 0.0f;
    static float s_filtered_target_roll = 0.0f;
    s_filtered_target_pitch += 0.15f * (target_pitch_deg - s_filtered_target_pitch);
    s_filtered_target_roll += 0.15f * (target_roll_deg - s_filtered_target_roll);
    // --- TẦNG 3: ATTITUDE ANGLE PID ---
    // float target_rate_roll = pid_compute(&pid_angle_roll, target_roll_deg, state->roll_deg, dt);
    // float target_rate_pitch = pid_compute(&pid_angle_pitch, target_pitch_deg, state->pitch_deg, dt);
    float target_rate_roll = pid_compute(&pid_angle_roll, s_filtered_target_roll, state->roll_deg, dt);
    float target_rate_pitch = pid_compute(&pid_angle_pitch, s_filtered_target_pitch, state->pitch_deg, dt);
    float target_rate_yaw = setpoint->yaw_rate_target;

    // --- TẦNG 4: ATTITUDE RATE PID ---
    float roll_rate_deg = state->gyro_x * RAD_TO_DEG;
    float pitch_rate_deg = state->gyro_y * RAD_TO_DEG;
    float yaw_rate_deg = state->gyro_z * RAD_TO_DEG;

    float u_roll = pid_compute(&pid_rate_roll, target_rate_roll, roll_rate_deg, dt);
    float u_pitch = pid_compute(&pid_rate_pitch, target_rate_pitch, pitch_rate_deg, dt);
    float u_yaw = pid_compute(&pid_rate_yaw, target_rate_yaw, yaw_rate_deg, dt);

    // KẸP BIÊN U_YAW: Không cho phép Yaw chiếm quá 10% công suất Motor
    if (u_yaw > 0.10f)
        u_yaw = 0.10f;
    if (u_yaw < -0.10f)
        u_yaw = -0.10f;

    // --- TẦNG 5: MOTOR MIXER MATRIX ---
    float motor_targets[4];
    motor_targets[0] = target_thrust - u_pitch - u_roll + u_yaw; // M1 CCW
    motor_targets[1] = target_thrust + u_pitch - u_roll - u_yaw; // M2 CW
    motor_targets[2] = target_thrust + u_pitch + u_roll + u_yaw; // M3 CCW
    motor_targets[3] = target_thrust - u_pitch + u_roll - u_yaw; // M4 CW

    // === THUẬT TOÁN CHỐNG BÃO HÒA MIXER (MOTOR DESATURATION) ===
    // Tìm motor có công suất lớn nhất và nhỏ nhất
    float max_motor = motor_targets[0];
    float min_motor = motor_targets[0];
    for (int i = 1; i < 4; i++)
    {
        if (motor_targets[i] > max_motor)
            max_motor = motor_targets[i];
        if (motor_targets[i] < min_motor)
            min_motor = motor_targets[i];
    }

    // Trường hợp 1: Có motor vượt trần MAX_MOTOR_PWM -> Hạ toàn bộ Mixer xuống
    if (max_motor > MAX_MOTOR_PWM)
    {
        float excess = max_motor - MAX_MOTOR_PWM;
        for (int i = 0; i < 4; i++)
        {
            motor_targets[i] -= excess;
        }
    }
    // Trường hợp 2: Có motor tụt dưới MIN_MOTOR_PWM -> Nâng toàn bộ Mixer lên
    else if (min_motor < MIN_MOTOR_PWM)
    {
        float deficit = MIN_MOTOR_PWM - min_motor;
        for (int i = 0; i < 4; i++)
        {
            motor_targets[i] += deficit;
        }
    }

    // Ép biên an toàn tuyệt đối
    for (int i = 0; i < 4; i++)
    {
        if (motor_targets[i] > MAX_MOTOR_PWM)
            motor_targets[i] = MAX_MOTOR_PWM;
        if (motor_targets[i] < MIN_MOTOR_PWM)
            motor_targets[i] = MIN_MOTOR_PWM;

        // Limiter gia tốc PWM chống sụt áp LDO
        float max_step = MAX_PWM_SLEW_RATE * dt;
        float diff = motor_targets[i] - s_motor_pwm[i];
        if (diff > max_step)
            diff = max_step;
        if (diff < -max_step)
            diff = -max_step;

        s_motor_pwm[i] += diff;

        // Ghi lại giá trị thực tế sau khi Desaturation vào biến debug/log
        s_current_motor_targets[i] = s_motor_pwm[i];

        uint32_t duty = (uint32_t)(s_motor_pwm[i] * 2047.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }

    // Lưu dữ liệu Telemetry
    s_debug_data.u_z = target_thrust;
    s_debug_data.target_pitch_deg = target_pitch_deg;
    s_debug_data.target_roll_deg = target_roll_deg;
    s_debug_data.target_rate_roll = target_rate_roll;
    s_debug_data.target_rate_pitch = target_rate_pitch;
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