#include "flow_filter.h"
#include "ekf.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "2D_FLOW_EKF";

// Class EKF 1D ước lượng [Vị trí, Vận tốc]^T cho 1 trục (X hoặc Y)
class Axis2DEKF : public ekf
{
public:
    // 2 states (pos, vel), 1 input (accel)
    Axis2DEKF() : ekf(2, 1) {}

    void Init() override
    {
        X(0, 0) = 0.0f; // Position
        X(1, 0) = 0.0f; // Velocity

        // Hiệp phương sai P ban đầu
        P(0, 0) = 0.1f;
        P(0, 1) = 0.0f;
        P(1, 0) = 0.0f;
        P(1, 1) = 0.1f;

        // Nhiễu quá trình Q (gia tốc IMU)
        Q(0, 0) = 0.2f;
    }
    void ZeroVelocity()
    {
        X(1, 0) = 0.0f;  // Chỉ ép Vận tốc về 0 (X(0,0) Vị trí giữ nguyên)
        P(1, 1) = 0.01f; // Thu hẹp độ tin tưởng vận tốc về 0
    }

    void LinearizeFG(dspm::Mat &x, float *u) override
    {
        // F matrix: pos_dot = vel, vel_dot = 0
        F(0, 0) = 0.0f;
        F(0, 1) = 1.0f;
        F(1, 0) = 0.0f;
        F(1, 1) = 0.0f;

        // G matrix: input = accel
        G(0, 0) = 0.0f;
        G(1, 0) = 1.0f;
    }
};

static Axis2DEKF *s_ekf_x = nullptr;
static Axis2DEKF *s_ekf_y = nullptr;
static bool s_initialized = false;

extern "C"
{

    esp_err_t flow_filter_init(void)
    {
        if (s_ekf_x == nullptr)
            s_ekf_x = new Axis2DEKF();
        if (s_ekf_y == nullptr)
            s_ekf_y = new Axis2DEKF();

        s_ekf_x->Init();
        s_ekf_y->Init();

        s_initialized = true;
        ESP_LOGI(TAG, "2D Optical Flow Decoupled EKF (esp-dsp) Initialized successfully!");
        return ESP_OK;
    }

    void flow_filter_predict(float ax_world, float ay_world, float dt)
    {
        if (!s_initialized)
            return;

        float ux[1] = {ax_world};
        float uy[1] = {ay_world};

        s_ekf_x->Process(ux, dt);
        s_ekf_y->Process(uy, dt);
    }

    void flow_filter_update(float vx_flow, float vy_flow, uint8_t qual)
    {
        if (!s_initialized)
            return;

        // Bỏ qua update nếu chất lượng bề mặt quan sát quá kém (SQUAL < 15)
        if (qual < 15)
            return;

        // Ma trận H = [0, 1] vì cảm biến Optical Flow đo VẬN TỐC (Velocity), không phải Vị trí
        dspm::Mat H(1, 2);
        H(0, 0) = 0.0f;
        H(0, 1) = 1.0f;

        // Nhiễu đo lường R động phụ thuộc vào chất lượng hình ảnh (SQUAL)
        float r_val = 0.01f + (255.0f - (float)qual) * 0.0005f;
        float R[1] = {r_val};

        // Update X Axis
        float meas_x[1] = {vx_flow};
        float exp_x[1] = {s_ekf_x->X(1, 0)}; // So sánh với s_ekf_x.vel
        s_ekf_x->Update(H, meas_x, exp_x, R);

        // Update Y Axis
        float meas_y[1] = {vy_flow};
        float exp_y[1] = {s_ekf_y->X(1, 0)}; // So sánh với s_ekf_y.vel
        s_ekf_y->Update(H, meas_y, exp_y, R);
    }

    void flow_filter_get_estimate(pos2d_estimate_t *est)
    {
        if (est != nullptr && s_initialized)
        {
            est->x = s_ekf_x->X(0, 0);
            est->vx = s_ekf_x->X(1, 0);
            est->y = s_ekf_y->X(0, 0);
            est->vy = s_ekf_y->X(1, 0);
        }
    }
    void flow_filter_reset(void)
    {
        if (s_ekf_x != nullptr && s_ekf_y != nullptr)
        {
            s_ekf_x->Init();
            s_ekf_y->Init();
        }
    }
    void flow_filter_zero_velocity(void)
    {
        if (s_ekf_x != nullptr && s_ekf_y != nullptr)
        {
            s_ekf_x->ZeroVelocity();
            s_ekf_y->ZeroVelocity();
        }
    }
} // extern "C"