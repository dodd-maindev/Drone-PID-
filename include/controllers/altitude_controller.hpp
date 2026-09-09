#pragma once
#include "controllers/pid.hpp"

namespace DroneControllers {

/**
 * @brief Bộ điều khiển độ cao chuyên biệt cho Quadcopter x500 trong Gazebo
 */
class AltitudeController {
public:
    // Điểm lơ lửng lý thuyết của x500 (m=2.06kg, g=9.8, 4 động cơ km=8.55e-6): hover ~ 0.59
    AltitudeController(double hover_thrust = 0.59, 
                       double kp = 0.35, double ki = 0.08, double kd = 0.22);

    void reset();
    float compute_thrust(double target_alt, double current_alt, double vz, double dt);
    float compute_thrust(double target_alt, double current_alt, double dt);

    void set_hover_thrust(double hover_thrust) { hover_thrust_ = hover_thrust; }

private:
    double hover_thrust_{0.59};
    PIDController pid_;
};

} // namespace DroneControllers
