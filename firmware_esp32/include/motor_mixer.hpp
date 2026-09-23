#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include "math_utils.hpp"

namespace DroneControllers {

/**
 * @brief Bộ trộn động cơ (Motor Mixer) cho Quadrotor khung chữ X (x500 geometry)
 */
class MotorMixer {
public:
    explicit MotorMixer(float max_rot_velocity = 1000.0f)
        : max_rot_velocity_(max_rot_velocity) {}

    std::array<float, 4> compute_motor_speeds(float thrust, float roll, float pitch, float yaw, bool is_armed = true) const {
        std::array<float, 4> speeds{0.0f, 0.0f, 0.0f, 0.0f};

        if (!is_armed || thrust <= 0.02f) {
            return speeds;
        }

        float u0 = thrust - roll - pitch - yaw; // Front-Right (CCW)
        float u1 = thrust + roll + pitch - yaw; // Rear-Left   (CCW)
        float u2 = thrust + roll - pitch + yaw; // Front-Left  (CW)
        float u3 = thrust - roll + pitch + yaw; // Rear-Right  (CW)

        std::array<float, 4> u = {u0, u1, u2, u3};

        // Mixer Desaturation
        float max_u = *std::max_element(u.begin(), u.end());
        if (max_u > 1.0f) {
            float excess = max_u - 1.0f;
            for (size_t i = 0; i < 4; ++i) {
                u[i] -= excess;
            }
        }

        float min_u = *std::min_element(u.begin(), u.end());
        if (min_u < 0.0f) {
            float deficit = -min_u;
            for (size_t i = 0; i < 4; ++i) {
                u[i] += deficit;
            }
        }

        for (size_t i = 0; i < 4; ++i) {
            float clamped_u = DroneMath::clamp(u[i], 0.0f, 1.0f);
            speeds[i] = max_rot_velocity_ * std::sqrt(clamped_u);
        }

        return speeds;
    }

private:
    float max_rot_velocity_{1000.0f};
};

} // namespace DroneControllers
