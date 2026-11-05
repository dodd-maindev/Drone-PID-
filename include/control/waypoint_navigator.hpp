#pragma once

#include <vector>
#include <mutex>
#include <cmath>
#include <chrono>
#include <functional>
#include <string>
#include <iostream>

namespace DroneControl {

/**
 * @brief Cấu trúc dữ liệu một điểm Waypoint
 */
struct WaypointItem {
    uint16_t seq{0};
    uint16_t command{16}; // MAV_CMD_NAV_WAYPOINT = 16, MAV_CMD_NAV_TAKEOFF = 22, MAV_CMD_NAV_LAND = 21
    double lat{0.0};      // Vĩ độ (deg)
    double lon{0.0};      // Kinh độ (deg)
    float x{0.0f};        // Tọa độ Gazebo X (Bắc, mét)
    float y{0.0f};        // Tọa độ Gazebo Y (Đông, mét)
    float z{0.0f};        // Cao độ mục tiêu (mét)
    float hold_time{0.0f};// Thời gian dừng tại waypoint (giây)
    float accept_radius{1.2f}; // Bán kính chấp nhận tới đích (mét)
    float yaw_deg{0.0f};  // Góc hướng mong muốn (độ)
};

/**
 * @brief Các chế độ dẫn đường tự động
 */
enum class NavMode {
    MANUAL = 0,            // Thủ công (Phi công lái bằng tay cầm / phím)
    GUIDED = 1,            // Chỉ điểm bay ngay (Click & Fly / Reposition)
    MISSION = 2,           // Bay tự động theo danh sách Waypoints
    AUTO_LAND = 3          // Tự động hạ cánh
};

/**
 * @brief Lệnh điều khiển sinh ra từ bộ điều hướng chuyển tới ESP32
 */
struct NavCommand {
    float cmd_vx{0.0f};      // Vận tốc tới/lui trong Body frame (m/s)
    float cmd_vy{0.0f};      // Vận tốc trái/phải trong Body frame (m/s)
    float cmd_alt_vel{0.0f}; // Vận tốc thẳng đứng (m/s)
    float cmd_yaw_rate{0.0f};// Vận tốc xoay góc hướng (rad/s)
    bool is_driving_x{false};
    bool is_driving_y{false};
    bool is_alt_active{false};
    bool is_yaw_active{false};
    bool is_armed{true};
    bool is_landing{false};
    std::string status_text;
};

/**
 * @brief Bộ dẫn đường và quản lý Waypoints tương thích chuẩn MAVLink QGroundControl
 */
class WaypointNavigator {
public:
    WaypointNavigator();

    // Cài đặt tọa độ gốc GPS (Home Reference)
    void set_home_gps(double lat, double lon);
    double get_home_lat() const { return home_lat_; }
    double get_home_lon() const { return home_lon_; }

    // Chuyển đổi qua lại giữa GPS (Lat, Lon, Alt) và Gazebo Local (X, Y, Z mét)
    void gps_to_local(double lat, double lon, float alt, float& out_x, float& out_y, float& out_z) const;
    void local_to_gps(float x, float y, float z, double& out_lat, double& out_lon, float& out_alt) const;

    // Quản lý nhiệm vụ (Mission Management)
    void clear_mission();
    void add_waypoint(const WaypointItem& wp);
    void set_mission(const std::vector<WaypointItem>& waypoints);
    size_t get_waypoint_count() const;
    bool get_waypoint(size_t index, WaypointItem& out_wp) const;
    uint16_t get_current_seq() const { return current_seq_; }
    void set_current_seq(uint16_t seq);

    // Kích hoạt các chế độ bay tự động
    void start_mission();
    void start_guided_reposition(float target_x, float target_y, float target_z, float yaw_deg = -1.0f);
    void start_auto_land();
    void cancel_auto_mode(const std::string& reason = "Manual Override");

    // Vòng lặp cập nhật điều hướng chính (chạy ở tần số 100Hz)
    NavCommand update(float cur_x, float cur_y, float cur_z, float cur_yaw,
                      float cur_vx, float cur_vy, float cur_vz, float dt);

    NavMode get_nav_mode() const { return nav_mode_; }
    bool is_auto_active() const { return nav_mode_ != NavMode::MANUAL; }

    // Đăng ký Callback sự kiện
    void set_waypoint_reached_callback(std::function<void(uint16_t seq)> cb) {
        reached_cb_ = cb;
    }
    void set_current_seq_callback(std::function<void(uint16_t seq)> cb) {
        current_seq_cb_ = cb;
    }
    void set_status_callback(std::function<void(const std::string& msg, uint8_t severity)> cb) {
        status_cb_ = cb;
    }

private:
    double home_lat_{21.028511};
    double home_lon_{105.854167};
    double meters_per_deg_lat_{111320.0};
    double meters_per_deg_lon_{103906.66};

    mutable std::recursive_mutex mission_mutex_;
    std::vector<WaypointItem> waypoints_;
    uint16_t current_seq_{0};
    NavMode nav_mode_{NavMode::MANUAL};

    // Mục tiêu Guided tạm thời
    WaypointItem guided_target_;

    // Bộ đếm thời gian giữ tại Waypoint
    bool is_holding_at_wp_{false};
    float hold_timer_{0.0f};

    // Tốc độ bay tuần tra tối đa (Cruise Speed)
    float max_horizontal_speed_{2.2f}; // m/s
    float max_climb_speed_{1.2f};      // m/s
    float max_descent_speed_{0.8f};    // m/s

    // Callbacks
    std::function<void(uint16_t)> reached_cb_{nullptr};
    std::function<void(uint16_t)> current_seq_cb_{nullptr};
    std::function<void(const std::string&, uint8_t)> status_cb_{nullptr};
};

} // namespace DroneControl

