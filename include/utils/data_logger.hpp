#pragma once
#include <string>
#include <fstream>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <iomanip>

namespace DroneUtils {

/**
 * @brief Cấu trúc dữ liệu ghi nhận một mẫu dữ liệu bay (Telemetry Record)
 */
struct FlightLogEntry {
    double time_s{0.0};
    float target_thrust{0.0f};
    float target_roll_deg{0.0f};
    float target_pitch_deg{0.0f};
    float target_yaw_deg{0.0f};

    float actual_x{0.0f};
    float actual_y{0.0f};
    float actual_vx{0.0f};
    float actual_vy{0.0f};
    float actual_alt{0.0f};
    float actual_vz{0.0f};
    float actual_roll_deg{0.0f};
    float actual_pitch_deg{0.0f};
    float actual_yaw_deg{0.0f};

    float rate_p{0.0f};
    float rate_q{0.0f};
    float rate_r{0.0f};

    float tau_roll{0.0f};
    float tau_pitch{0.0f};
    float tau_yaw{0.0f};

    // Tốc độ góc thực tế 4 cánh quạt gửi sang Gazebo qua topic /x500/command/motor_speed (rad/s)
    float w0{0.0f}; // Rotor 0: Front-Right CCW
    float w1{0.0f}; // Rotor 1: Rear-Left CCW
    float w2{0.0f}; // Rotor 2: Front-Left CW
    float w3{0.0f}; // Rotor 3: Rear-Right CW
};

/**
 * @brief Cấu trúc dữ liệu ghi nhận bám quỹ đạo bay tự động (Trajectory Tracking Log)
 * Phục vụ vẽ đồ thị định lượng X, Y, Z, Yaw lý thuyết vs thực tế theo yêu cầu NCKH / đồ án
 */
struct TrajectoryLogEntry {
    double time_s{0.0};

    // Quỹ đạo lý thuyết đặt trước (Expected Reference Trajectory)
    float target_x{0.0f};
    float target_y{0.0f};
    float target_z{0.0f};
    float target_yaw_rad{0.0f};
    float target_yaw_deg{0.0f};

    // Quỹ đạo bay thực tế đo được từ Odometry (Actual Trajectory)
    float actual_x{0.0f};
    float actual_y{0.0f};
    float actual_z{0.0f};
    float actual_yaw_rad{0.0f};
    float actual_yaw_deg{0.0f};

    // Sai số bám quỹ đạo định lượng (Tracking Errors)
    float err_x{0.0f};
    float err_y{0.0f};
    float err_z{0.0f};
    float err_dist_3d{0.0f};
    float err_yaw_deg{0.0f};

    // Vận tốc và lệnh điều khiển
    float actual_vx{0.0f};
    float actual_vy{0.0f};
    float actual_vz{0.0f};
    float cmd_vx{0.0f};
    float cmd_vy{0.0f};
    float cmd_alt_vel{0.0f};
    float roll_deg{0.0f};
    float pitch_deg{0.0f};

    // Tốc độ 4 động cơ
    float w0{0.0f};
    float w1{0.0f};
    float w2{0.0f};
    float w3{0.0f};

    uint16_t wp_seq{0};
    int nav_mode{0}; // 1: GUIDED, 2: MISSION, 3: AUTO_LAND
};

/**
 * @brief Class chuyên trách ghi log dữ liệu bám quỹ đạo ra file CSV tự động
 */
class TrajectoryLogger {
public:
    TrajectoryLogger();
    ~TrajectoryLogger();

    bool start(const std::string& log_dir = "logs/trajectory",
               const std::string& prefix = "trajectory_log");
    void log(const TrajectoryLogEntry& entry);
    void stop();

    bool is_logging() const { return is_logging_; }
    std::string get_current_log_path() const { return current_log_path_; }
    std::string get_latest_log_path() const { return latest_log_path_; }
    size_t get_records_count() const { return records_count_; }

private:
    std::ofstream file_;
    std::string current_log_path_;
    std::string latest_log_path_;
    std::mutex mutex_;
    bool is_logging_{false};
    size_t records_count_{0};
};

/**
 * @brief Class chuyên trách ghi log dữ liệu bay thời gian thực ra file CSV
 */
class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    bool start(const std::string& log_dir = "/home/do/drone_control_cpp/log",
               const std::string& prefix = "flight_log");
    void log(const FlightLogEntry& entry);
    void stop();

    bool is_logging() const { return is_logging_; }
    std::string get_current_log_path() const { return current_log_path_; }
    std::string get_latest_log_path() const { return latest_log_path_; }
    size_t get_records_count() const { return records_count_; }

private:
    std::ofstream file_;
    std::string current_log_path_;
    std::string latest_log_path_;
    std::mutex mutex_;
    bool is_logging_{false};
    size_t records_count_{0};
};

} // namespace DroneUtils
