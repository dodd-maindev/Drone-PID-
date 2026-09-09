#include "comm/gz_bridge.hpp"
#include "math/math_utils.hpp"
#include <iostream>

namespace DroneComm {

GzBridge::GzBridge() : node_(std::make_unique<gz::transport::Node>()) {}

GzBridge::~GzBridge() {
    shutdown();
}

bool GzBridge::init(DroneCore::VehicleState* state_ptr,
                    const std::string& odom_topic,
                    const std::string& motor_topic) {
    state_ = state_ptr;
    if (!state_) return false;

    // 1. Subscribe topic Odometry để nhận dữ liệu trạng thái
    if (!node_->Subscribe(odom_topic, &GzBridge::on_odometry, this)) {
        std::cerr << "[GzBridge] [!] Lỗi khi subscribe topic: " << odom_topic << std::endl;
        return false;
    }

    // 2. Advertise topic điều khiển 4 cánh quạt
    motor_pub_ = node_->Advertise<gz::msgs::Actuators>(motor_topic);
    // Advertise thêm alias /model/x500/command/motor_speed để đảm bảo tương thích 100%
    motor_pub_alt_ = node_->Advertise<gz::msgs::Actuators>("/model/x500/command/motor_speed");

    initialized_ = true;
    std::cout << "[GzBridge] [✓] Đã mở cầu nối GZ-Transport: Odom [" << odom_topic 
              << "], Motor [" << motor_topic << "]" << std::endl;
    return true;
}

void GzBridge::on_odometry(const gz::msgs::Odometry& msg) {
    if (!state_) return;

    // Cập nhật độ cao và vận tốc tịnh tiến
    state_->altitude.store(msg.pose().position().z());
    state_->vx.store(msg.twist().linear().x());
    state_->vy.store(msg.twist().linear().y());
    state_->vz.store(msg.twist().linear().z());

    // Cập nhật góc Euler Roll, Pitch, Yaw từ Quaternion
    float w = msg.pose().orientation().w();
    float x = msg.pose().orientation().x();
    float y = msg.pose().orientation().y();
    float z = msg.pose().orientation().z();
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    DroneMath::quaternion_to_euler(w, x, y, z, roll, pitch, yaw);

    state_->roll.store(roll);
    state_->pitch.store(pitch);
    state_->yaw.store(yaw);

    // Cập nhật vận tốc góc p, q, r
    state_->p.store(msg.twist().angular().x());
    state_->q.store(msg.twist().angular().y());
    state_->r.store(msg.twist().angular().z());

    state_->is_connected.store(true);
}

void GzBridge::send_motor_velocities(const std::array<float, 4>& speeds) {
    if (!initialized_) return;

    gz::msgs::Actuators act_msg;
    for (int i = 0; i < 4; ++i) {
        act_msg.add_velocity(speeds[i]);
    }

    motor_pub_.Publish(act_msg);
    motor_pub_alt_.Publish(act_msg);
}

void GzBridge::shutdown() {
    if (initialized_) {
        // Gửi lệnh dừng 4 động cơ khi ngắt kết nối
        std::array<float, 4> zero_speeds{0.0f, 0.0f, 0.0f, 0.0f};
        send_motor_velocities(zero_speeds);
        initialized_ = false;
    }
}

} // namespace DroneComm
