#pragma once
#include <atomic>
#include <string>

namespace DroneCore {

/**
 * @brief Struct lưu trữ toàn bộ trạng thái cảm biến (Telemetry) của Drone
 */
struct VehicleState {
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

    std::atomic<bool> is_armed{false};          // Động cơ đã mở khóa chưa
    std::atomic<bool> is_connected{false};      // Đã nhận Heartbeat từ drone chưa
    std::atomic<uint8_t> system_id{1};          // ID của Drone (thường là 1)
    std::atomic<uint8_t> component_id{1};       // Component ID của Drone (thường là 1)
};

} // namespace DroneCore
