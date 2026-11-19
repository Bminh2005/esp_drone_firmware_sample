#include "altitude_filter.h"
#include "ekf.h"
#include "esp_log.h"

static const char *TAG = "ALTITUDE_EKF_CPP";

// 1. Kế thừa class ekf của esp-dsp cho bộ lọc cao độ 1D
class AltitudeEKF : public ekf{
    public :
        // Call constructor gốc: 2 states (z, vz), 1 noise input (az)
        AltitudeEKF() : ekf(2, 1){}

    void Init() override{
        // State vector X = [z, vz]^T
        X(0, 0) = 0.0f;
X(1, 0) = 0.0f;

// Ma trận hiệp phương sai ban đầu P (2x2)
P(0, 0) = 0.01f;
P(0, 1) = 0.0f;
P(1, 0) = 0.0f;
P(1, 1) = 0.1f;

// Nhiễu quá trình Q
Q(0, 0) = 0.5f;
}

// Định nghĩa hệ thống vi phân: xDot = F*x + G*u
void LinearizeFG(dspm::Mat &x, float *u) override
{
    // Ma trận F (2x2)
    F(0, 0) = 0.0f;
    F(0, 1) = 1.0f;
    F(1, 0) = 0.0f;
    F(1, 1) = 0.0f;

    // Ma trận G (2x1)
    G(0, 0) = 0.0f;
    G(1, 0) = 1.0f;
}
}
;

static AltitudeEKF *s_ekf = nullptr;

// 2. Expose các hàm wrapper chuẩn C để main.c gọi bình thường
extern "C"
{

    esp_err_t altitude_filter_init(float initial_z)
    {
        if (s_ekf == nullptr)
        {
            s_ekf = new AltitudeEKF();
        }
        s_ekf->Init();
        s_ekf->X(0, 0) = initial_z;

        ESP_LOGI(TAG, "esp-dsp C++ AltitudeEKF initialized (z_0 = %.3fm)", initial_z);
        return ESP_OK;
    }

    void altitude_filter_predict(float az_world, float dt)
    {
        if (s_ekf != nullptr)
        {
            float u[1] = {az_world};
            // Hàm Process của ekf thực hiện RungeKutta & CovariancePrediction
            s_ekf->Process(u, dt);
        }
    }

    void altitude_filter_update(float z_meas)
    {
        if (s_ekf != nullptr)
        {
            // Ma trận đo lường H = [1, 0] (chỉ đo z)
            dspm::Mat H(1, 2);
            H(0, 0) = 1.0f;
            H(0, 1) = 0.0f;

            float measured[1] = {z_meas};
            float expected[1] = {s_ekf->X(0, 0)};
            float R[1] = {0.0025f}; // Nhiễu ToF (~5cm)^2

            s_ekf->Update(H, measured, expected, R);
        }
    }

    float altitude_filter_get_z(void)
    {
        return (s_ekf != nullptr) ? s_ekf->X(0, 0) : 0.0f;
    }

    float altitude_filter_get_vz(void)
    {
        return (s_ekf != nullptr) ? s_ekf->X(1, 0) : 0.0f;
    }

    void altitude_filter_reset(float z_reset)
    {
        if (s_ekf != nullptr)
        {
            s_ekf->Init();
            s_ekf->X(0, 0) = z_reset;
        }
    }

} // extern "C"