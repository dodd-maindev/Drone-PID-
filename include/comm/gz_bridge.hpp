#pragma once
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <string>
#include <array>
#include <memory>
#include "core/vehicle_state.hpp"

namespace DroneComm {

/**
 * @brief Cầu nối giao tiếp trực tiếp với Gazebo Sim qua GZ Transport
 */
class GzBridge {
public:
    GzBridge();
    ~GzBridge();

    /**
     * @brief Khởi động node GZ-Transport, lắng nghe Odometry và sẵn sàng gửi lệnh motor
     */
    bool init(DroneCore::VehicleState* state_ptr,
              const std::string& odom_topic = "/model/x500/odometry",
              const std::string& motor_topic = "/x500/command/motor_speed");

    /**
     * @brief Bơm vận tốc 4 động cơ vào Gazebo Sim
     */
    void send_motor_velocities(const std::array<float, 4>& speeds);

    /**
     * @brief Đóng các kết nối
     */
    void shutdown();

    bool is_initialized() const { return initialized_; }

private:
    void on_odometry(const gz::msgs::Odometry& msg);

    std::unique_ptr<gz::transport::Node> node_;
    gz::transport::Node::Publisher motor_pub_;
    gz::transport::Node::Publisher motor_pub_alt_; // fallback cho /model/x500/command/motor_speed

    DroneCore::VehicleState* state_{nullptr};
    bool initialized_{false};
};

} // namespace DroneComm
