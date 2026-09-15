#pragma once
#include <thread>
#include <atomic>
#include <chrono>
#include <array>
#include "comm/gz_bridge.hpp"
#include "comm/mavlink_bridge.hpp"
#include "controllers/motor_mixer.hpp"
#include "core/vehicle_state.hpp"
#include "utils/data_logger.hpp"

namespace DroneCore {

/**
 * @brief Class điều phối phương tiện cấp cao (Vehicle Commander cho Gazebo Sim & QGC)
 */
class VehicleCommander {
public:
    VehicleCommander();
    ~VehicleCommander();

    bool start(int dummy_port = 0);
    void stop();
    bool wait_for_connection(int timeout_seconds = 10);

    // Các lệnh hành vi
    void arm(bool force = true);
    void disarm();
    void set_offboard_mode();

    // Gửi góc nghiêng và lực đẩy (Roll, Pitch, Yaw theo rad, Thrust [0.0 - 1.0])
    void send_attitude_target(float roll, float pitch, float yaw, float thrust);

    // Truy cập dữ liệu trạng thái máy bay
    const VehicleState& state() const { return state_; }

    // Quản lý MAVLink Bridge (QGroundControl)
    DroneComm::MavlinkBridge& mavlink() { return mavlink_bridge_; }

    // Quản lý logger dữ liệu
    DroneUtils::DataLogger& logger() { return logger_; }

private:
    void control_worker();

    DroneComm::GzBridge gz_bridge_;
    DroneComm::MavlinkBridge mavlink_bridge_;
    DroneControllers::MotorMixer mixer_;
    VehicleState state_;
    DroneUtils::DataLogger logger_;

    std::atomic<float> target_roll_{0.0f};
    std::atomic<float> target_pitch_{0.0f};
    std::atomic<float> target_yaw_{0.0f};
    std::atomic<float> target_thrust_{0.0f};

    std::atomic<bool> running_{false};
    std::thread control_thread_;
    std::chrono::steady_clock::time_point start_time_;
    uint64_t log_counter_{0};
};

} // namespace DroneCore
