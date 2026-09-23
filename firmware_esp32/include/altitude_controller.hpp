#pragma once
#include <algorithm>
#include "pid.hpp"

namespace DroneControllers {

/**
 * @brief Bộ điều khiển độ cao chuyên biệt (PI + Velocity Damping) cho ESP32
 */
class AltitudeController {
public:
    AltitudeController(double hover_thrust = 0.59, 
                       double kp = 0.35, double ki = 0.08, double kd = 0.22)
        : hover_thrust_(hover_thrust),
          pid_(kp, ki, 0.0, -0.30, 0.25, 0.15) {}

    void reset() {
        pid_.reset();
    }

    float compute_thrust(double target_alt, double current_alt, double vz, double dt) {
        double delta_thrust = pid_.update(target_alt, current_alt, dt);
        const double kv = 0.32;
        double total_thrust = hover_thrust_ + delta_thrust - kv * vz;
        return static_cast<float>(std::clamp(total_thrust, 0.10, 0.85));
    }

    float compute_thrust(double target_alt, double current_alt, double dt) {
        return compute_thrust(target_alt, current_alt, 0.0, dt);
    }

    void set_hover_thrust(double hover_thrust) { hover_thrust_ = hover_thrust; }

private:
    double hover_thrust_{0.59};
    PIDController pid_;
};

} // namespace DroneControllers
