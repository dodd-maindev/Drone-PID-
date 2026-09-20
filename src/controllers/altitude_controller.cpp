#include "controllers/altitude_controller.hpp"
#include <algorithm>

namespace DroneControllers {

AltitudeController::AltitudeController(double hover_thrust, double kp, double ki, double kd)
    : hover_thrust_(hover_thrust),
      pid_(kp, ki, 0.0, -0.30, 0.25, 0.15) {}

void AltitudeController::reset() {
    pid_.reset();
}

float AltitudeController::compute_thrust(double target_alt, double current_alt, double vz, double dt) {
    // 1. Tính lượng bù ga từ PI độ cao (không dùng vi phân vị trí rời rạc để tránh nhiễu và giật ga)
    double delta_thrust = pid_.update(target_alt, current_alt, dt);

    // 2. Phanh vi phân vận tốc thẳng đứng (Velocity Damping) trực tiếp từ vz:
    // Tránh hoàn toàn hiện tượng Derivative Kick khi đổi setpoint và triệt tiêu rung giật
    const double kv = 0.32;
    double total_thrust = hover_thrust_ + delta_thrust - kv * vz;

    // 3. Giới hạn dải ga [0.10, 0.85] (giữ lại ít nhất 15% dự phòng cho cân bằng tư thế)
    return static_cast<float>(std::clamp(total_thrust, 0.10, 0.85));
}

float AltitudeController::compute_thrust(double target_alt, double current_alt, double dt) {
    return compute_thrust(target_alt, current_alt, 0.0, dt);
}

} // namespace DroneControllers
