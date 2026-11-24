#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "state_estimator.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// --- GÁN CHÂN GPIO ĐỘNG CƠ (KHUNG XINWEI E1 / ESP32 MINI DRONE) ---
#define MOTOR_1_GPIO GPIO_NUM_15 // M1: Trước - Phải
#define MOTOR_2_GPIO GPIO_NUM_13 // M2: Sau - Phải
#define MOTOR_3_GPIO GPIO_NUM_12 // M3: Sau - Trái
#define MOTOR_4_GPIO GPIO_NUM_14 // M4: Trước - Trái

// --- CẤU HÌNH PWM LEDC DRIVER ---
// Đổi độ phân giải LEDC từ 13-bit sang 11-bit (hoặc 10-bit)
#define LEDC_PWM_FREQ_HZ 20000         // Tần số 20kHz cho motor coreless
#define LEDC_PWM_RES LEDC_TIMER_11_BIT // 11-bit resolution (0 -> 2047)

// --- GIỚI HẠN AN TOÀN PHẦN CỨNG ---
#define MAX_MOTOR_PWM 0.88f      // Kẹp trần 88% chống sụt áp LDO
#define MIN_MOTOR_PWM 0.05f      // Xung Idle nhỏ nhất (5%)
#define MAX_PWM_SLEW_RATE 3.50f  // Giới hạn gia tốc PWM (1/s) chống dòng xả đỉnh
#define UNSTICK_ALTITUDE_M 0.08f // Ngưỡng rời sàn 3cm
#define FAILSAFE_TIMEOUT_MS 500  // Timeout ngắt UDP (ms)

#define MOTOR_ARM_IDLE_THRUST 0.10f
#define HOVER_BASE_THRUST 0.65f
#define TAKEOFF_PULSE_THRUST 0.75f

typedef struct
{
    float z_target, z_state, u_z;
    float vx_state, vy_state, target_pitch_deg, target_roll_deg;
    float roll_deg, pitch_deg, target_rate_roll, target_rate_pitch;
    float gyro_x, gyro_y, gyro_z, u_roll, u_pitch, u_yaw;
} flight_ctrl_debug_t;

void flight_control_init(void);
void flight_control_step(const uav_state_t *state, const drone_setpoint_t *setpoint, float dt);
void flight_control_disarm_motors(void);
void flight_control_get_motors(float motors[4]);
void flight_control_get_debug(flight_ctrl_debug_t *out_debug);
#endif // FLIGHT_CONTROL_H