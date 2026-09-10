#pragma once
#include <atomic>
#include <string>

namespace DroneCore {

/**
 * @brief Struct lưu trữ toàn bộ trạng thái cảm biến (Telemetry) của Drone
 */
struct VehicleState {
    std::atomic<float> pos_x{0.0f};             // Tọa độ vị trí X (mét)
    std::atomic<float> pos_y{0.0f};             // Tọa độ vị trí Y (mét)
    std::atomic<float> altitude{0.0f};          // Độ cao Z (mét, dương hướng lên)
    std::atomic<float> vz{0.0f};                // Vận tốc thẳng đứng (m/s)
    std::atomic<float> vx{0.0f};                // Vận tốc hướng Bắc (m/s)
    std::atomic<float> vy{0.0f};                // Vận tốc hướng Đông (m/s)

    std::atomic<float> roll{0.0f};              // Góc nghiêng Roll (rad)
    std::atomic<float> pitch{0.0f};             // Góc nghiêng Pitch (rad)
    std::atomic<float> yaw{0.0f};               // Góc quay Yaw (rad)

    std::atomic<float> p{0.0f};                 // Vận tốc góc quanh trục X (rad/s)
    std::atomic<float> q{0.0f};                 // Vận tốc góc quanh trục Y (rad/s)
    std::atomic<float> r{0.0f};                 // Vận tốc góc quanh trục Z (rad/s)

    // Các chỉ số THỰC TẾ gửi sang Gazebo qua topic /x500/command/motor_speed:
    std::atomic<float> motor_speed_0{0.0f};     // Rotor 0 (Front-Right CCW) [rad/s]
    std::atomic<float> motor_speed_1{0.0f};     // Rotor 1 (Rear-Left CCW)   [rad/s]
    std::atomic<float> motor_speed_2{0.0f};     // Rotor 2 (Front-Left CW)   [rad/s]
    std::atomic<float> motor_speed_3{0.0f};     // Rotor 3 (Rear-Right CW)   [rad/s]

    // Mô-men và lực đẩy đầu ra của Controller:
    std::atomic<float> current_thrust{0.0f};    // Lực nâng tổng chuẩn hóa [0.0 - 1.0]
    std::atomic<float> tau_roll{0.0f};          // Mô-men Roll bù góc
    std::atomic<float> tau_pitch{0.0f};         // Mô-men Pitch bù góc
    std::atomic<float> tau_yaw{0.0f};           // Mô-men Yaw bù góc

    std::atomic<bool> is_armed{false};          // Động cơ đã mở khóa chưa
    std::atomic<bool> is_connected{false};      // Đã nhận tín hiệu từ Gazebo chưa
    std::atomic<uint8_t> system_id{1};          // ID của Drone (thường là 1)
    std::atomic<uint8_t> component_id{1};       // Component ID của Drone (thường là 1)
};

} // namespace DroneCore
