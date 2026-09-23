#pragma once
#include <algorithm>
#include "flight_config.h"
#include "pid.hpp"

namespace DroneControllers {

/**
 * @brief Bộ điều khiển độ cao (PI + Velocity Damping) cho ESP32
 * Tham số được nạp tự động từ FlightConfig::Altitude
 */
class AltitudeController {
public:
    AltitudeController()
        : hover_thrust_(FlightConfig::Altitude::HOVER_THRUST),
          pid_(FlightConfig::Altitude::KP_ALT, 
               FlightConfig::Altitude::KI_ALT, 
               0.0, -0.30, 0.25, 
               FlightConfig::Altitude::INT_ALT_LIMIT) {}

    void reset() {
        pid_.reset();
    }

    float compute_thrust(double target_alt, double current_alt, double vz, double dt = FlightConfig::Timing::DT) {
        double delta_thrust = pid_.update(target_alt, current_alt, dt);
        double total_thrust = hover_thrust_ + delta_thrust - FlightConfig::Altitude::KV_DAMPING * vz;
        return static_cast<float>(std::clamp(total_thrust, 
                                             static_cast<double>(FlightConfig::Altitude::MIN_TOTAL_THRUST), 
                                             static_cast<double>(FlightConfig::Altitude::MAX_TOTAL_THRUST)));
    }

    void set_hover_thrust(double hover_thrust) { hover_thrust_ = hover_thrust; }

private:
    double hover_thrust_{FlightConfig::Altitude::HOVER_THRUST};
    PIDController pid_;
};

} // namespace DroneControllers
