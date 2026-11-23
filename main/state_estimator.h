#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Struct trạng thái ước lượng từ IMU + Sensors
typedef struct
{
    float roll;       // rad
    float pitch;      // rad
    float yaw;        // rad
    float roll_deg;   // deg
    float pitch_deg;  // deg
    float yaw_deg;    // deg
    float gyro_x;     // rad/s (Trục X Body)
    float gyro_y;     // rad/s (Trục Y Body)
    float gyro_z;     // rad/s (Trục Z Body)
    float x;          // m (Position X)
    float y;          // m (Position Y)
    float z;          // m (Altitude ToF)
    float vx;         // m/s (Velocity X)
    float vy;         // m/s (Velocity Y)
    float vz;         // m/s (Velocity Z)
    uint16_t dist_mm; // Khoảng cách thô ToF (mm)
    uint8_t squal;    // Chất lượng ảnh Optical Flow
    bool is_initialized;
} uav_state_t;

// Struct lệnh điều khiển nhận từ UDP
typedef struct
{
    float roll_target;       // deg (-30 đến +30)
    float pitch_target;      // deg (-30 đến +30)
    float yaw_rate_target;   // deg/s
    float z_target;          // m
    float vx_target;         // m/s
    float vy_target;         // m/s
    float base_thrust;       // 0.0f -> 1.0f (PWM ratio)
    bool armed;              // Trạng thái cho phép chạy motor
    uint32_t last_packet_ms; // Timestamp gói tin UDP cuối cùng
} drone_setpoint_t;

// Public APIs
esp_err_t state_estimator_init(void);
esp_err_t state_estimator_calibrate_gyro(void);
bool state_estimator_get_state(uav_state_t *out_state);
void state_estimator_stream_telemetry(const uav_state_t *state);

#endif // STATE_ESTIMATOR_H