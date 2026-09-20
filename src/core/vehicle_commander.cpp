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

    // Khởi động MAVLink Bridge kết nối QGroundControl
    mavlink_bridge_.init(&state_);
    mavlink_bridge_.set_arm_callback([this](bool arm_val) {
        if (arm_val) {
            this->arm();
        } else {
            this->disarm();
        }
    });

    start_time_ = std::chrono::steady_clock::now();
    log_counter_ = 0;
    logger_.start("/home/do/drone_control_cpp/log", "flight_telemetry");

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
        mavlink_bridge_.shutdown();
        gz_bridge_.shutdown();
        logger_.stop();
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
            mavlink_bridge_.send_statustext("Gazebo Sim Connected", 6);
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
    mavlink_bridge_.send_statustext("Vehicle ARMED", 6);
}

void VehicleCommander::disarm() {
    std::cout << "[VehicleCommander] Gửi lệnh DISARM tắt động cơ..." << std::endl;
    state_.is_armed.store(false);
    target_thrust_.store(0.0f);
    gz_bridge_.send_motor_velocities({0.0f, 0.0f, 0.0f, 0.0f});
    state_.motor_speed_0.store(0.0f);
    state_.motor_speed_1.store(0.0f);
    state_.motor_speed_2.store(0.0f);
    state_.motor_speed_3.store(0.0f);
    state_.current_thrust.store(0.0f);
    mavlink_bridge_.send_statustext("Vehicle DISARMED", 6);
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
    const float ki_att = 0.05f;
    const float kd_att = 0.08f;
    const float kp_yaw = 0.30f;
    const float kd_yaw = 0.15f;

    float int_roll = 0.0f;
    float int_pitch = 0.0f;

    while (running_) {
        if (!state_.is_armed.load() || target_thrust_.load() <= 0.02f) {
            int_roll = 0.0f;
            int_pitch = 0.0f;
            gz_bridge_.send_motor_velocities({0.0f, 0.0f, 0.0f, 0.0f});
            state_.motor_speed_0.store(0.0f);
            state_.motor_speed_1.store(0.0f);
            state_.motor_speed_2.store(0.0f);
            state_.motor_speed_3.store(0.0f);
            state_.current_thrust.store(0.0f);
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

        // 1. Tính toán sai số và mô-men phản hồi PID tư thế
        float err_roll = r_cmd - r_cur;
        int_roll += err_roll * 0.005f;
        int_roll = DroneMath::clamp(int_roll, -0.05f, 0.05f); // Anti-windup
        float tau_roll = kp_att * err_roll + ki_att * int_roll - kd_att * p_rate;

        float err_pitch = p_cmd - p_cur;
        int_pitch += err_pitch * 0.005f;
        int_pitch = DroneMath::clamp(int_pitch, -0.05f, 0.05f); // Anti-windup
        float tau_pitch = kp_att * err_pitch + ki_att * int_pitch - kd_att * q_rate;

        float err_yaw = DroneMath::normalize_angle(y_cmd - y_cur);
        float tau_yaw = kp_yaw * err_yaw - kd_yaw * r_rate;

        // Giới hạn mô-men tối đa để không gây mất lực nâng tổng
        tau_roll = DroneMath::clamp(tau_roll, -0.25f, 0.25f);
        tau_pitch = DroneMath::clamp(tau_pitch, -0.25f, 0.25f);
        tau_yaw = DroneMath::clamp(tau_yaw, -0.20f, 0.20f);

        // Tự động bù hao hụt lực nâng do góc nghiêng thân (Tilt Compensation)
        float cos_tilt = std::cos(r_cur) * std::cos(p_cur);
        if (cos_tilt < 0.65f) cos_tilt = 0.65f;
        float effective_thrust = t_cmd / cos_tilt;

        // 2. Chuyển đổi qua Motor Mixer thành vận tốc quay 4 cánh quạt
        auto speeds = mixer_.compute_motor_speeds(effective_thrust, tau_roll, tau_pitch, tau_yaw, true);

        // 3. Cập nhật các chỉ số thực tế vào state_ để người dùng có thể đọc theo thời gian thực
        state_.motor_speed_0.store(speeds[0]);
        state_.motor_speed_1.store(speeds[1]);
        state_.motor_speed_2.store(speeds[2]);
        state_.motor_speed_3.store(speeds[3]);
        state_.current_thrust.store(t_cmd);
        state_.tau_roll.store(tau_roll);
        state_.tau_pitch.store(tau_pitch);
        state_.tau_yaw.store(tau_yaw);

        // 4. Bơm lệnh tốc độ quay trực tiếp vào Gazebo Sim
        gz_bridge_.send_motor_velocities(speeds);

        // 5. Ghi log các chỉ số thực tế sang file CSV (tần số 50Hz)
        if (++log_counter_ % 4 == 0 && logger_.is_logging()) {
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
            DroneUtils::FlightLogEntry entry;
            entry.time_s = elapsed;
            entry.target_thrust = t_cmd;
            entry.target_roll_deg = r_cmd * 180.0f / static_cast<float>(M_PI);
            entry.target_pitch_deg = p_cmd * 180.0f / static_cast<float>(M_PI);
            entry.target_yaw_deg = y_cmd * 180.0f / static_cast<float>(M_PI);

            entry.actual_x = state_.pos_x.load();
            entry.actual_y = state_.pos_y.load();
            entry.actual_vx = state_.vx.load();
            entry.actual_vy = state_.vy.load();
            entry.actual_alt = state_.altitude.load();
            entry.actual_vz = state_.vz.load();
            entry.actual_roll_deg = r_cur * 180.0f / static_cast<float>(M_PI);
            entry.actual_pitch_deg = p_cur * 180.0f / static_cast<float>(M_PI);
            entry.actual_yaw_deg = y_cur * 180.0f / static_cast<float>(M_PI);

            entry.rate_p = p_rate;
            entry.rate_q = q_rate;
            entry.rate_r = r_rate;

            entry.tau_roll = tau_roll;
            entry.tau_pitch = tau_pitch;
            entry.tau_yaw = tau_yaw;

            entry.w0 = speeds[0];
            entry.w1 = speeds[1];
            entry.w2 = speeds[2];
            entry.w3 = speeds[3];

            logger_.log(entry);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

} // namespace DroneCore
