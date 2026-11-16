#include <math.h>
#include "maholy.h"

#define MAHONY_KP MAHONY_DEFAULT_KP                                      // Hệ số P (Tín nhiệm Accel)
#define MAHONY_KI MAHONY_DEFAULT_KI                                      // Hệ số I (Bù Gyro Drift)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // Quaternion trạng thái initial
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // Tích phân lỗi

void mahony_init(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    mahony_reset_integral();
}

void mahony_reset_integral(void)
{
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void mahony_get_quaternion(float *out_q0, float *out_q1, float *out_q2, float *out_q3)
{
    if (out_q0)
        *out_q0 = q0;
    if (out_q1)
        *out_q1 = q1;
    if (out_q2)
        *out_q2 = q2;
    if (out_q3)
        *out_q3 = q3;
}
void mahony_ahrs_update_6dof(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt, ahrs_state_t *output)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float qa, qb, qc;

    // 1. Nếu không có đọc gia tốc (hoặc đang rơi tự do |a| ~ 0), bỏ qua bước hiệu chỉnh Accel
    float a_norm_sq = ax * ax + ay * ay + az * az;
    if (a_norm_sq > 0.001f)
    {
        // Chuẩn hóa vectơ gia tốc
        recipNorm = 1.0f / sqrtf(a_norm_sq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Ước lượng hướng vectơ trọng lực v = [vx, vy, vz] từ Quaternion hiện tại
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Tính vectơ sai lệch e = a x v (Cross product giữa Accel đo được và Trọng lực dự đoán)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Khâu tích phân (Integral Feedback)
        if (MAHONY_KI > 0.0f)
        {
            integralFBx += MAHONY_KI * ex * dt;
            integralFBy += MAHONY_KI * ey * dt;
            integralFBz += MAHONY_KI * ez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Khâu tỷ lệ (Proportional Feedback)
        gx += MAHONY_KP * ex;
        gy += MAHONY_KP * ey;
        gz += MAHONY_KP * ez;
    }

    // 2. Tích phân Quaternion theo tốc độ quay đã được bù lỗi
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Chuẩn hóa lại Quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 3. Trích xuất góc Euler (Roll, Pitch)
    output->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    output->pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    output->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    // 4. Trích xuất gia tốc trục Z phẳng (World Frame) - trừ đi trọng lực 1G
    // Dùng trực tiếp cho bộ lọc 1D Altitude EKF ở Bước 3
    float raw_az_world = 2.0f * (q1 * q3 - q0 * q2) * ax +
                         2.0f * (q0 * q1 + q2 * q3) * ay +
                         (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az;
    output->az_world = (raw_az_world - 1.0f) * 9.81f; // Trừ 1G và đổi sang m/s^2
}