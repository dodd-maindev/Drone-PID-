#include "core/vehicle_commander.hpp"
#include "math/math_utils.hpp"
#include <iostream>

namespace DroneCore {

VehicleCommander::VehicleCommander() = default;

VehicleCommander::~VehicleCommander() {
    stop();
}

bool VehicleCommander::start(int /*dummy_port*/) {
    if (!gz_bridge_.init(&state_)) {
        std::cerr << "[VehicleCommander] [!] Không thể khởi động GzBridge!" << std::endl;
        return false;
    }

    running_ = true;
    control_thread_ = std::thread(&VehicleCommander::control_worker, this);
    return true;
}

void VehicleCommander::stop() {
    if (running_) {
        disarm();
        running_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        gz_bridge_.shutdown();
    }
}

bool VehicleCommander::wait_for_connection(int timeout_seconds) {
    std::cout << "[VehicleCommander] Đang chờ dữ liệu từ Gazebo Sim (/model/x500/odometry)..." << std::endl;
    auto start_wait = std::chrono::steady_clock::now();

    while (running_) {
        if (state_.is_connected.load()) {
            std::cout << "[VehicleCommander] [✓] Đã kết nối thành công với Drone x500 trong Gazebo Sim!" << std::endl;
            // Khởi tạo mục tiêu yaw ban đầu trùng với hướng máy bay lúc xuất phát
            target_yaw_.store(state_.yaw.load());
            return true;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_wait
        ).count();

        if (elapsed > timeout_seconds) {
            std::cerr << "[VehicleCommander] [!] Quá thời gian chờ kết nối Gazebo! Hãy đảm bảo đã chạy world mô phỏng." << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void VehicleCommander::arm(bool /*force*/) {
    std::cout << "[VehicleCommander] Gửi lệnh ARM động cơ (Gazebo)..." << std::endl;
    state_.is_armed.store(true);
}

void VehicleCommander::disarm() {
    std::cout << "[VehicleCommander] Gửi lệnh DISARM tắt động cơ..." << std::endl;
    state_.is_armed.store(false);
    target_thrust_.store(0.0f);
    gz_bridge_.send_motor_velocities({0.0f, 0.0f, 0.0f, 0.0f});
}

void VehicleCommander::set_offboard_mode() {
    std::cout << "[VehicleCommander] Kích hoạt Direct Controller Mode (Bỏ qua PX4)..." << std::endl;
}

void VehicleCommander::send_attitude_target(float roll, float pitch, float yaw, float thrust) {
    target_roll_.store(roll);
    target_pitch_.store(pitch);
    target_yaw_.store(yaw);
    target_thrust_.store(thrust);
}

void VehicleCommander::control_worker() {
    // Vòng lặp điều khiển tư thế & phân phối lực motor tần số ~200Hz (5ms)
    const float kp_att = 0.45f;
    const float kd_att = 0.08f;
    const float kp_yaw = 0.35f;
    const float kd_yaw = 0.06f;

    while (running_) {
        if (!state_.is_armed.load() || target_thrust_.load() <= 0.02f) {
            gz_bridge_.send_motor_velocities({0.0f, 0.0f, 0.0f, 0.0f});
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        float r_cmd = target_roll_.load();
        float p_cmd = target_pitch_.load();
        float y_cmd = target_yaw_.load();
        float t_cmd = target_thrust_.load();

        float r_cur = state_.roll.load();
        float p_cur = state_.pitch.load();
        float y_cur = state_.yaw.load();

        float p_rate = state_.p.load();
        float q_rate = state_.q.load();
        float r_rate = state_.r.load();

        // 1. Tính toán sai số và mô-men phản hồi PD
        float err_roll = r_cmd - r_cur;
        float tau_roll = kp_att * err_roll - kd_att * p_rate;

        float err_pitch = p_cmd - p_cur;
        float tau_pitch = kp_att * err_pitch - kd_att * q_rate;

        float err_yaw = DroneMath::normalize_angle(y_cmd - y_cur);
        float tau_yaw = kp_yaw * err_yaw - kd_yaw * r_rate;

        // Giới hạn mô-men tối đa để không gây mất lực nâng tổng
        tau_roll = DroneMath::clamp(tau_roll, -0.25f, 0.25f);
        tau_pitch = DroneMath::clamp(tau_pitch, -0.25f, 0.25f);
        tau_yaw = DroneMath::clamp(tau_yaw, -0.20f, 0.20f);

        // 2. Chuyển đổi qua Motor Mixer thành vận tốc quay 4 cánh quạt
        auto speeds = mixer_.compute_motor_speeds(t_cmd, tau_roll, tau_pitch, tau_yaw, true);

        // 3. Bơm lệnh tốc độ quay trực tiếp vào Gazebo Sim
        gz_bridge_.send_motor_velocities(speeds);

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

} // namespace DroneCore
