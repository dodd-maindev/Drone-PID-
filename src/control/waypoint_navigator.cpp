#include "control/waypoint_navigator.hpp"
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace DroneControl {

static float normalize_angle(float angle_rad) {
    while (angle_rad > static_cast<float>(M_PI)) angle_rad -= 2.0f * static_cast<float>(M_PI);
    while (angle_rad < -static_cast<float>(M_PI)) angle_rad += 2.0f * static_cast<float>(M_PI);
    return angle_rad;
}

WaypointNavigator::WaypointNavigator() {
    set_home_gps(home_lat_, home_lon_);
}

void WaypointNavigator::set_home_gps(double lat, double lon) {
    home_lat_ = lat;
    home_lon_ = lon;
    meters_per_deg_lat_ = 111320.0;
    meters_per_deg_lon_ = 111320.0 * std::cos(home_lat_ * M_PI / 180.0);
}

void WaypointNavigator::gps_to_local(double lat, double lon, float alt,
                                     float& out_x, float& out_y, float& out_z) const {
    out_x = static_cast<float>((lat - home_lat_) * meters_per_deg_lat_);
    out_y = static_cast<float>((lon - home_lon_) * meters_per_deg_lon_);
    out_z = alt;
}

void WaypointNavigator::local_to_gps(float x, float y, float z,
                                     double& out_lat, double& out_lon, float& out_alt) const {
    out_lat = home_lat_ + (static_cast<double>(x) / meters_per_deg_lat_);
    out_lon = home_lon_ + (static_cast<double>(y) / meters_per_deg_lon_);
    out_alt = z;
}

void WaypointNavigator::clear_mission() {
    std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
    waypoints_.clear();
    current_seq_ = 0;
    is_holding_at_wp_ = false;
    hold_timer_ = 0.0f;
    if (nav_mode_ == NavMode::MISSION) {
        nav_mode_ = NavMode::MANUAL;
    }
}

void WaypointNavigator::add_waypoint(const WaypointItem& wp) {
    std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
    WaypointItem item = wp;
    if (item.command == 20 || (std::abs(item.lat) < 0.1 && std::abs(item.lon) < 0.1 && std::abs(item.x) < 0.001f && std::abs(item.y) < 0.001f)) {
        if (!waypoints_.empty() && (std::abs(waypoints_[0].lat) > 0.1 || std::abs(waypoints_[0].lon) > 0.1)) {
            item.lat = waypoints_[0].lat;
            item.lon = waypoints_[0].lon;
            item.x   = waypoints_[0].x;
            item.y   = waypoints_[0].y;
        } else {
            item.lat = home_lat_;
            item.lon = home_lon_;
            item.x   = 0.0f;
            item.y   = 0.0f;
        }
    } else if (std::abs(item.x) < 0.001f && std::abs(item.y) < 0.001f && (std::abs(item.lat) > 0.1 || std::abs(item.lon) > 0.1)) {
        gps_to_local(item.lat, item.lon, item.z, item.x, item.y, item.z);
    }
    waypoints_.push_back(item);
}

void WaypointNavigator::set_mission(const std::vector<WaypointItem>& waypoints) {
    std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
    waypoints_ = waypoints;

    // Xác định tọa độ Home / Return chuẩn (ưu tiên Planned Home ở Waypoint #0 nếu có)
    double ret_lat = home_lat_;
    double ret_lon = home_lon_;
    float ret_x = 0.0f;
    float ret_y = 0.0f;
    if (!waypoints_.empty() && (std::abs(waypoints_[0].lat) > 0.1 || std::abs(waypoints_[0].lon) > 0.1)) {
        gps_to_local(waypoints_[0].lat, waypoints_[0].lon, waypoints_[0].z, waypoints_[0].x, waypoints_[0].y, waypoints_[0].z);
        ret_lat = waypoints_[0].lat;
        ret_lon = waypoints_[0].lon;
        ret_x   = waypoints_[0].x;
        ret_y   = waypoints_[0].y;
    }

    for (auto& item : waypoints_) {
        // Nếu là lệnh Return To Launch (cmd=20) hoặc tọa độ GPS bằng (0,0):
        // Gán tọa độ đích về đúng vị trí Home / Return để drone bay về điểm H và dừng lại!
        if (item.command == 20 || (std::abs(item.lat) < 0.1 && std::abs(item.lon) < 0.1)) {
            item.lat = ret_lat;
            item.lon = ret_lon;
            item.x   = ret_x;
            item.y   = ret_y;
        } else {
            gps_to_local(item.lat, item.lon, item.z, item.x, item.y, item.z);
        }
    }
    current_seq_ = 0;
    is_holding_at_wp_ = false;
    hold_timer_ = 0.0f;
}

size_t WaypointNavigator::get_waypoint_count() const {
    std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
    return waypoints_.size();
}

bool WaypointNavigator::get_waypoint(size_t index, WaypointItem& out_wp) const {
    std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
    if (index >= waypoints_.size()) return false;
    out_wp = waypoints_[index];
    return true;
}

void WaypointNavigator::set_current_seq(uint16_t seq) {
    std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
    current_seq_ = seq;
    is_holding_at_wp_ = false;
    hold_timer_ = 0.0f;
}

void WaypointNavigator::start_mission() {
    std::function<void(uint16_t)> seq_cb = nullptr;
    std::function<void(const std::string&, uint8_t)> stat_cb = nullptr;
    size_t count = 0;
    uint16_t start_seq = 0;

    {
        std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
        if (waypoints_.empty()) {
            std::cout << "[Navigator] [!] Không thể chạy: Danh sách Waypoint rỗng!" << std::endl;
            stat_cb = status_cb_;
        } else {
            // Nếu Waypoint 0 là Planned Home (z < 0.5m) và danh sách có nhiều hơn 1 WP,
            // bắt đầu ngay từ Waypoint 1 để drone bay tới điểm đầu tiên của nhiệm vụ!
            if (waypoints_.size() > 1 && waypoints_[0].seq == 0 && waypoints_[0].z < 0.5f) {
                current_seq_ = 1;
            } else {
                current_seq_ = 0;
            }
            start_seq = current_seq_;
            is_holding_at_wp_ = false;
            hold_timer_ = 0.0f;
            segment_started_ = false;
            segment_time_ = 0.0f;
            prev_e_ct_ = 0.0f;
            d_e_ct_filtered_ = 0.0f;
            virtual_yaw_initialized_ = false;
            nav_mode_ = NavMode::MISSION;
            count = waypoints_.size();
            seq_cb = current_seq_cb_;
            stat_cb = status_cb_;
        }
    }

    if (count == 0) {
        if (stat_cb) stat_cb("Mission empty! Please plan waypoints in QGC", 4);
        return;
    }

    std::cout << "[Navigator] [🚀 AUTO] Bắt đầu thực thi nhiệm vụ (" 
              << count << " Waypoints, khởi động từ WP #" << start_seq << ")..." << std::endl;
    if (stat_cb) stat_cb("Starting Mission: " + std::to_string(count) + " WPs", 6);
    if (seq_cb) seq_cb(start_seq);
}

void WaypointNavigator::start_guided_reposition(float target_x, float target_y, float target_z, float yaw_deg) {
    std::function<void(const std::string&, uint8_t)> stat_cb = nullptr;
    {
        std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
        guided_target_.x = target_x;
        guided_target_.y = target_y;
        guided_target_.z = std::max(0.5f, target_z);
        guided_target_.yaw_deg = yaw_deg;
        guided_target_.accept_radius = 0.85f;
        guided_target_.hold_time = 0.0f;
        local_to_gps(target_x, target_y, target_z, guided_target_.lat, guided_target_.lon, guided_target_.z);

        nav_mode_ = NavMode::GUIDED;
        is_holding_at_wp_ = false;
        hold_timer_ = 0.0f;
        segment_started_ = false;
        segment_time_ = 0.0f;
        prev_e_ct_ = 0.0f;
        d_e_ct_filtered_ = 0.0f;
        virtual_yaw_initialized_ = false;
        stat_cb = status_cb_;
    }

    std::cout << "[Navigator] [📍 GUIDED] Nhận lệnh bay tới điểm: X=" 
              << target_x << "m, Y=" << target_y << "m, Z=" << target_z << "m" << std::endl;
    if (stat_cb) stat_cb("Flying to Reposition Target", 6);
}

void WaypointNavigator::start_auto_land() {
    nav_mode_ = NavMode::AUTO_LAND;
    std::cout << "[Navigator] [🛑 LAND] Kích hoạt chế độ Auto-Land từ QGC." << std::endl;
    if (status_cb_) status_cb_("Auto-Landing engaged", 6);
}

void WaypointNavigator::cancel_auto_mode(const std::string& reason) {
    bool was_active = false;
    {
        std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
        if (nav_mode_ != NavMode::MANUAL) {
            nav_mode_ = NavMode::MANUAL;
            is_holding_at_wp_ = false;
            hold_timer_ = 0.0f;
            virtual_yaw_initialized_ = false;
            was_active = true;
        }
    }
    if (was_active) {
        std::cout << "\n[Navigator] [⚠️ MANUAL] Tạm dừng tự động: " << reason << std::endl;
        if (status_cb_) status_cb_("Auto Mode Cancelled: " + reason, 5);
    }
}

NavCommand WaypointNavigator::update(float cur_x, float cur_y, float cur_z, float cur_yaw,
                                     float cur_vx, float cur_vy, float cur_vz, float dt) {
    NavCommand cmd;
    cmd.is_armed = true;

    if (nav_mode_ == NavMode::MANUAL) {
        cmd.status_text = "MANUAL";
        return cmd;
    }

    if (nav_mode_ == NavMode::HOLD) {
        // Chế độ khóa vị trí chủ động tại đích đến (Active Hold Target - Triệt tiêu 100% hiện tượng trôi máy bay)
        float dx = hold_target_x_ - cur_x;
        float dy = hold_target_y_ - cur_y;
        float dz = hold_target_z_ - cur_z;
        float dist_xy = std::sqrt(dx * dx + dy * dy);

        float cy = std::cos(cur_yaw);
        float sy = std::sin(cur_yaw);
        float v_world_x = std::clamp(1.2f * dx, -0.90f, 0.90f);
        float v_world_y = std::clamp(1.2f * dy, -0.90f, 0.90f);

        cmd.cmd_vx =  cy * v_world_x + sy * v_world_y;
        cmd.cmd_vy = -sy * v_world_x + cy * v_world_y;
        cmd.cmd_alt_vel = std::clamp(1.0f * dz, -0.6f, 0.6f);
        cmd.cmd_yaw_rate = 0.0f;
        cmd.is_driving_x = (dist_xy > 0.05f);
        cmd.is_driving_y = (dist_xy > 0.05f);
        cmd.is_alt_active = (std::abs(dz) > 0.05f);
        cmd.is_yaw_active = false;
        cmd.has_target = true;
        cmd.target_x = hold_target_x_;
        cmd.target_y = hold_target_y_;
        cmd.target_z = hold_target_z_;
        cmd.target_yaw_rad = virtual_target_yaw_;
        cmd.status_text = "MISSION COMPLETE (ACTIVE HOLD)";
        return cmd;
    }

    if (nav_mode_ == NavMode::AUTO_LAND) {
        cmd.is_landing = true;
        cmd.is_alt_active = true;
        cmd.cmd_alt_vel = -0.65f;
        cmd.status_text = "AUTO LANDING...";
        return cmd;
    }

    WaypointItem target_wp;
    bool has_target = false;

    {
        std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
        if (nav_mode_ == NavMode::GUIDED) {
            target_wp = guided_target_;
            has_target = true;
        } else if (nav_mode_ == NavMode::MISSION) {
            if (current_seq_ < waypoints_.size()) {
                target_wp = waypoints_[current_seq_];
                has_target = true;
            } else {
                // Đã hoàn thành hết tất cả waypoints -> Khóa giữ vị trí đích chủ động
                hold_target_x_ = !waypoints_.empty() ? waypoints_.back().x : cur_x;
                hold_target_y_ = !waypoints_.empty() ? waypoints_.back().y : cur_y;
                hold_target_z_ = !waypoints_.empty() ? waypoints_.back().z : cur_z;
                nav_mode_ = NavMode::HOLD;
                std::cout << "\n[Navigator] [✓ HOÀN THÀNH] Toàn bộ lộ trình đã hoàn tất! Chuyển sang ACTIVE HOLD khóa vị trí đích." << std::endl;
                if (status_cb_) status_cb_("Mission Completed! Active Holding at destination", 6);
                cmd.has_target = true;
                cmd.target_x = hold_target_x_;
                cmd.target_y = hold_target_y_;
                cmd.target_z = hold_target_z_;
                cmd.target_yaw_rad = virtual_target_yaw_;
                cmd.status_text = "MISSION COMPLETE (HOLD)";
                return cmd;
            }
        }
    }

    if (!has_target) {
        nav_mode_ = NavMode::MANUAL;
        cmd.status_text = "MANUAL";
        return cmd;
    }

    // Bắt đầu ghi nhận mốc đoạn đường mới nếu chưa có
    if (!segment_started_) {
        if (nav_mode_ == NavMode::MISSION && current_seq_ > 0 && current_seq_ - 1 < waypoints_.size()) {
            segment_start_x_ = waypoints_[current_seq_ - 1].x;
            segment_start_y_ = waypoints_[current_seq_ - 1].y;
            segment_start_z_ = waypoints_[current_seq_ - 1].z;
        } else {
            segment_start_x_ = cur_x;
            segment_start_y_ = cur_y;
            segment_start_z_ = cur_z;
        }
        segment_time_ = 0.0f;
        segment_started_ = true;
        prev_e_ct_ = 0.0f;
        d_e_ct_filtered_ = 0.0f;
    }

    // 1. Chuẩn hóa tham số Waypoint (chống NaN từ QGC và giới hạn trần bay 10m của ESP32)
    bool is_final_wp = (nav_mode_ == NavMode::GUIDED) ||
                       (current_seq_ + 1 >= waypoints_.size()) ||
                       (target_wp.command == 20) ||
                       (target_wp.command == 21);

    float target_z = (std::isnan(target_wp.z) || target_wp.z < 1.0f) ? 3.5f : std::min(target_wp.z, 9.5f);
    float eff_hold = (std::isnan(target_wp.hold_time) || target_wp.hold_time <= 0.05f) ? 0.0f : target_wp.hold_time;
    // Bán kính chuyển góc cua chặt chẽ (0.75m để drone chạm vào đỉnh góc cua, 0.45m cho điểm kết thúc)
    float default_radius = is_final_wp ? 0.45f : 0.75f;
    float effective_radius = (std::isnan(target_wp.accept_radius) || target_wp.accept_radius < 0.4f)
                             ? default_radius
                             : std::min(target_wp.accept_radius, 0.85f);

    // Tính toán sai lệch khoảng cách tới Waypoint
    float dx = target_wp.x - cur_x;
    float dy = target_wp.y - cur_y;
    float dz = target_z - cur_z;
    float dist_xy = std::sqrt(dx * dx + dy * dy);

    // 2. Kiểm tra điều kiện đến đích (Waypoint Arrival)
    // Đạt cự ly ngang <= effective_radius VÀ đã bay ở độ cao an toàn (|dz| <= 2.5m hoặc đã gần chạm trần 9.0m của ESP32)
    bool reached_horizontal = (dist_xy <= effective_radius);
    bool reached_vertical   = (cur_z >= 1.2f) && (std::abs(dz) <= 2.5f || cur_z >= 8.8f);

    // Log debug telemetry mỗi 1.0 giây để theo dõi cự ly tiếp cận chính xác
    static auto last_debug_print = std::chrono::steady_clock::now();
    auto now_print = std::chrono::steady_clock::now();
    if (std::chrono::duration<float>(now_print - last_debug_print).count() >= 1.0f) {
        last_debug_print = now_print;
        std::cout << "[Nav Telemetry] " << (nav_mode_ == NavMode::GUIDED ? "GUIDED" : "AUTO WP ")
                  << (nav_mode_ == NavMode::GUIDED ? 1 : (target_wp.seq + 1)) << "/" 
                  << (nav_mode_ == NavMode::GUIDED ? 1 : waypoints_.size())
                  << " | Cự ly: " << std::fixed << std::setprecision(1) << dist_xy << "m (R=" << effective_radius << "m)"
                  << " | Alt: " << cur_z << "/" << target_z << "m"
                  << " | Đích: (" << target_wp.x << ", " << target_wp.y << ")"
                  << std::endl;
    }

    if (reached_horizontal && reached_vertical) {
        // Nếu là điểm đích đơn lẻ (GUIDED mode) -> Đã tới nơi, dừng giữ vị trí
        if (nav_mode_ == NavMode::GUIDED) {
            nav_mode_ = NavMode::MANUAL;
            std::cout << "[Navigator] [✓ GUIDED REACHED] Đã bay đến điểm chỉ định thành công!" << std::endl;
            if (status_cb_) status_cb_("Reached Target Location (Holding)", 6);
            cmd.status_text = "REPOSITION REACHED (HOLD)";
            return cmd;
        }

        // Nếu là lệnh hạ cánh (MAV_CMD_NAV_LAND = 21) -> Chuyển sang hạ cánh tự động
        if (target_wp.command == 21) {
            if (reached_cb_) reached_cb_(target_wp.seq);
            nav_mode_ = NavMode::AUTO_LAND;
            cmd.is_landing = true;
            cmd.is_alt_active = true;
            cmd.cmd_alt_vel = -0.65f;
            cmd.status_text = "AUTO LANDING AT WP...";
            return cmd;
        }

        // Nếu là lệnh Return To Launch (MAV_CMD_NAV_RETURN_TO_LAUNCH = 20) -> Dừng giữ vị trí ngay tại điểm Return (H)!
        if (target_wp.command == 20) {
            if (reached_cb_) reached_cb_(target_wp.seq);
            hold_target_x_ = target_wp.x;
            hold_target_y_ = target_wp.y;
            hold_target_z_ = target_z;
            nav_mode_ = NavMode::HOLD;
            std::cout << "\n[Navigator] [✓ RETURN REACHED] Đã quay về vị trí Return (Home) và khóa giữ vị trí!" << std::endl;
            if (status_cb_) status_cb_("Returned to Home! Active Holding position", 6);
            cmd.has_target = true;
            cmd.target_x = hold_target_x_;
            cmd.target_y = hold_target_y_;
            cmd.target_z = hold_target_z_;
            cmd.target_yaw_rad = virtual_target_yaw_;
            cmd.status_text = "RETURN COMPLETE (HOLD)";
            return cmd;
        }

        // Nếu là lộ trình Mission:
        if (!is_holding_at_wp_) {
            is_holding_at_wp_ = true;
            hold_timer_ = 0.0f;
            std::cout << "\n[Navigator] [✓ WP REACHED] Đã tới Waypoint #" << (target_wp.seq + 1) 
                      << " (Hold " << eff_hold << "s)" << std::endl;
            if (reached_cb_) reached_cb_(target_wp.seq);
        }

        // Nếu không có yêu cầu dừng giữ (eff_hold == 0.0f):
        // Chuyển tiếp tức thì sang Waypoint tiếp theo và tiếp tục tính toán hành trình ngay trong frame này!
        if (eff_hold <= 0.05f) {
            is_holding_at_wp_ = false;
            hold_timer_ = 0.0f;
            uint16_t next_seq = 0;
            std::function<void(uint16_t)> seq_cb = nullptr;
            bool mission_completed = false;
            {
                std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
                current_seq_++;
                next_seq = current_seq_;
                seq_cb = current_seq_cb_;
                if (current_seq_ >= waypoints_.size()) {
                    mission_completed = true;
                } else {
                    target_wp = waypoints_[current_seq_];
                }
            }
            if (seq_cb) seq_cb(next_seq);

            if (mission_completed) {
                hold_target_x_ = target_wp.x;
                hold_target_y_ = target_wp.y;
                hold_target_z_ = target_z;
                nav_mode_ = NavMode::HOLD;
                std::cout << "\n[Navigator] [✓ HOÀN THÀNH] Toàn bộ lộ trình đã hoàn tất! Khóa giữ vị trí đích." << std::endl;
                if (status_cb_) status_cb_("Mission Completed! Active Holding at destination", 6);
                cmd.has_target = true;
                cmd.target_x = hold_target_x_;
                cmd.target_y = hold_target_y_;
                cmd.target_z = hold_target_z_;
                cmd.target_yaw_rad = virtual_target_yaw_;
                cmd.status_text = "MISSION COMPLETE (HOLD)";
                return cmd;
            }

            // Ghi nhận mốc điểm bắt đầu cho đoạn đường mới từ chính xác tọa độ Waypoint trước đó
            if (next_seq > 0 && next_seq - 1 < waypoints_.size()) {
                segment_start_x_ = waypoints_[next_seq - 1].x;
                segment_start_y_ = waypoints_[next_seq - 1].y;
                segment_start_z_ = waypoints_[next_seq - 1].z;
            } else {
                segment_start_x_ = cur_x;
                segment_start_y_ = cur_y;
                segment_start_z_ = cur_z;
            }
            segment_time_ = 0.0f;
            segment_started_ = true;
            prev_e_ct_ = 0.0f;
            d_e_ct_filtered_ = 0.0f;

            std::cout << "[Navigator] [➡️ TIẾP THEO] Chuyển tiếp tức thì sang Waypoint #" << (next_seq + 1) << "..." << std::endl;
            // Cập nhật lại vector khoảng cách tới waypoint mới để bay liên tục không khựng
            is_final_wp = (current_seq_ + 1 >= waypoints_.size()) || (target_wp.command == 20) || (target_wp.command == 21);
            target_z = (std::isnan(target_wp.z) || target_wp.z < 1.0f) ? 3.5f : std::min(target_wp.z, 9.5f);
            dx = target_wp.x - cur_x;
            dy = target_wp.y - cur_y;
            dz = target_z - cur_z;
            dist_xy = std::sqrt(dx * dx + dy * dy);
        } else {
            // Có yêu cầu dừng giữ: Đợi hết thời gian hold_timer
            hold_timer_ += dt;
            if (hold_timer_ >= eff_hold) {
                is_holding_at_wp_ = false;
                hold_timer_ = 0.0f;
                uint16_t next_seq = 0;
                std::function<void(uint16_t)> seq_cb = nullptr;
                {
                    std::lock_guard<std::recursive_mutex> lock(mission_mutex_);
                    current_seq_++;
                    next_seq = current_seq_;
                    seq_cb = current_seq_cb_;
                }
                if (seq_cb) seq_cb(next_seq);
                if (next_seq > 0 && next_seq - 1 < waypoints_.size()) {
                    segment_start_x_ = waypoints_[next_seq - 1].x;
                    segment_start_y_ = waypoints_[next_seq - 1].y;
                    segment_start_z_ = waypoints_[next_seq - 1].z;
                } else {
                    segment_start_x_ = cur_x;
                    segment_start_y_ = cur_y;
                    segment_start_z_ = cur_z;
                }
                segment_time_ = 0.0f;
                segment_started_ = true;
                prev_e_ct_ = 0.0f;
                d_e_ct_filtered_ = 0.0f;
                std::cout << "[Navigator] [➡️ TIẾP THEO] Chuyển tiếp sang Waypoint #" << (next_seq + 1) << "..." << std::endl;
            }

            // Trong lúc dừng giữ: Triệt tiêu vận tốc tới lui, giữ độ cao và hướng ổn định
            cmd.cmd_vx = 0.0f;
            cmd.cmd_vy = 0.0f;
            cmd.cmd_alt_vel = 0.0f;
            cmd.cmd_yaw_rate = 0.0f;
            cmd.is_driving_x = false;
            cmd.is_driving_y = false;
            cmd.is_alt_active = false;
            cmd.is_yaw_active = false;
            cmd.status_text = "WP #" + std::to_string(target_wp.seq + 1) + " HOLDING...";
            return cmd;
        }
    }

    // 2b. Kiểm tra an toàn cất cánh thẳng đứng (Vertical Takeoff Safety Gate):
    // Nếu drone đang ở dưới đất (cur_z < 0.80m) mà mục tiêu ở trên cao (target_z >= 1.0m):
    // Khóa chặt di chuyển ngang (vx = 0, vy = 0) để bốc thẳng đứng lên cao độ an toàn trước,
    // tuyệt đối không kéo lê chân đáp trên mặt đất hoặc nghiêng lật khi chưa thoát khỏi sàn!
    if (cur_z < 0.80f && target_z >= 1.0f) {
        cmd.cmd_vx = 0.0f;
        cmd.cmd_vy = 0.0f;
        cmd.cmd_yaw_rate = 0.0f;
        cmd.is_driving_x = false;
        cmd.is_driving_y = false;
        cmd.is_yaw_active = false;
        cmd.is_alt_active = true;
        cmd.cmd_alt_vel = max_climb_speed_; // Leo thẳng đứng với tốc độ tối đa
        cmd.status_text = "AUTO TAKEOFF: Climbing vertically to safe altitude...";
        return cmd;
    }

    // 3. Quy luật điều khiển bám tim đường thẳng tuyệt đối có Giảm chấn tới hạn (Critically Damped Line Following)
    float seg_dx = target_wp.x - segment_start_x_;
    float seg_dy = target_wp.y - segment_start_y_;
    float seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

    // Quản lý góc hướng Yaw bằng Setpoint Profiler (Hướng mũi luôn bay dọc theo tim đường của từng chặng)
    if (!virtual_yaw_initialized_) {
        virtual_target_yaw_ = cur_yaw;
        virtual_yaw_initialized_ = true;
    }

    float desired_yaw = (target_wp.yaw_deg > 0.05f)
                        ? (target_wp.yaw_deg * static_cast<float>(M_PI) / 180.0f)
                        : ((seg_len > 0.5f) ? std::atan2(seg_dy, seg_dx) : virtual_target_yaw_);

    // Tính sai số giữa hướng mục tiêu và góc đặt nội suy
    float err_target_yaw = normalize_angle(desired_yaw - virtual_target_yaw_);
    const float max_turn_rate = 0.50f; // rad/s (~28 deg/s) - êm dịu, không tạo lực ly tâm
    float yaw_rate_cmd = 0.0f;
    if (std::abs(err_target_yaw) > 0.025f) { // Vùng chết 1.4 độ
        yaw_rate_cmd = std::clamp(1.5f * err_target_yaw, -max_turn_rate, max_turn_rate);
        virtual_target_yaw_ = normalize_angle(virtual_target_yaw_ + yaw_rate_cmd * dt);
        cmd.is_yaw_active = true;
    } else {
        yaw_rate_cmd = 0.0f;
        cmd.is_yaw_active = false;
    }
    cmd.cmd_yaw_rate = yaw_rate_cmd;

    // Giảm tốc độ góc cua khi đang xoay đầu để drone ôm cua sắc nét, triệt tiêu quán tính văng ly tâm:
    float err_yaw_actual = normalize_angle(virtual_target_yaw_ - cur_yaw);
    float heading_align = std::clamp(std::cos(err_yaw_actual), 0.40f, 1.0f);

    float cruise_speed = max_horizontal_speed_;
    // Khi đến gần góc cua trong bán kính 2.5m, hãm tốc độ để ôm cua mượt mà, không bị vọt lố hay cắt góc
    float corner_approach_speed = std::clamp(0.65f * dist_xy, 0.40f, cruise_speed);
    float target_speed = is_final_wp
                         ? ((dist_xy > 3.0f) ? cruise_speed : (0.50f * dist_xy))
                         : corner_approach_speed;
    target_speed = std::clamp(target_speed, 0.10f, cruise_speed) * heading_align;

    float ux = (dist_xy > 0.001f) ? (dx / dist_xy) : 0.0f;
    float uy = (dist_xy > 0.001f) ? (dy / dist_xy) : 0.0f;

    float v_world_x = 0.0f;
    float v_world_y = 0.0f;

    if (seg_len >= 1.0f) {
        // Vector đơn vị dọc tim đường thẳng nối 2 Waypoint
        float line_ux = seg_dx / seg_len;
        float line_uy = seg_dy / seg_len;
        // Vector pháp tuyến vuông góc tim đường (hướng sang trái)
        float line_nx = -line_uy;
        float line_ny = line_ux;

        // Tính độ lệch vuông góc khỏi tim đường (Cross-Track Error)
        float p_x = cur_x - segment_start_x_;
        float p_y = cur_y - segment_start_y_;
        float e_ct = p_x * line_nx + p_y * line_ny;

        // Tính đạo hàm tốc độ rời xa/tiến lại tim đường để làm thành phần giảm chấn vi phân (Damping)
        float raw_d_e_ct = (dt > 0.001f) ? ((e_ct - prev_e_ct_) / dt) : 0.0f;
        prev_e_ct_ = e_ct;
        d_e_ct_filtered_ = 0.75f * d_e_ct_filtered_ + 0.25f * raw_d_e_ct;

        // Bộ điều khiển PD bám tim đường có giảm chấn tới hạn (Critically Damped):
        // - Kp kéo drone về tim đường
        // - Kd phanh tốc độ trôi ngang khi drone tiến sát tim đường, triệt tiêu 100% hiện tượng lắc võng (snaking)
        float Kp_ct = 0.45f;
        float Kd_ct = 0.30f;
        float v_cross = -Kp_ct * e_ct - Kd_ct * d_e_ct_filtered_;
        v_cross = std::clamp(v_cross, -0.35f, 0.35f);

        float v_along = target_speed;
        v_world_x = v_along * line_ux + v_cross * line_nx;
        v_world_y = v_along * line_uy + v_cross * line_ny;

        // Quỹ đạo lý thuyết nội suy liên tục theo thời gian (Continuous Ramp Reference Trajectory)
        segment_time_ += dt;
        float s_exp = std::min(segment_time_ * (cruise_speed * 0.90f), seg_len);
        cmd.target_x = segment_start_x_ + s_exp * line_ux;
        cmd.target_y = segment_start_y_ + s_exp * line_uy;
    } else {
        v_world_x = target_speed * ux;
        v_world_y = target_speed * uy;
        cmd.target_x = target_wp.x;
        cmd.target_y = target_wp.y;
    }

    // Chuyển đổi sang hệ trục thân máy bay đồng nhất 100% với PositionController của ESP32:
    //   err_xb =  cy * err_x_w + sy * err_y_w
    //   err_yb = -sy * err_x_w + cy * err_y_w
    float cy = std::cos(cur_yaw);
    float sy = std::sin(cur_yaw);
    cmd.cmd_vx =  cy * v_world_x + sy * v_world_y;
    cmd.cmd_vy = -sy * v_world_x + cy * v_world_y;
    cmd.is_driving_x = true;
    cmd.is_driving_y = true;

    // 5. Điều khiển vận tốc độ cao (Vertical Climb / Descent)
    cmd.cmd_alt_vel = std::clamp(0.95f * dz, -max_descent_speed_, max_climb_speed_);
    cmd.is_alt_active = (std::abs(dz) > 0.08f);

    // 6. Hiển thị trạng thái & Ghi nhận quỹ đạo mong muốn
    cmd.target_z = target_z;
    cmd.target_yaw_rad = virtual_target_yaw_;
    cmd.target_wp_seq = target_wp.seq;
    cmd.has_target = true;

    std::ostringstream ss;
    if (nav_mode_ == NavMode::GUIDED) {
        ss << "GUIDED -> (" << std::fixed << std::setprecision(1) << target_wp.x << ", " 
           << target_wp.y << ", " << target_z << "m) d=" << dist_xy << "m";
    } else {
        ss << "AUTO WP " << (target_wp.seq + 1) << "/" << waypoints_.size() 
           << " (d=" << std::fixed << std::setprecision(1) << dist_xy << "m)";
    }
    cmd.status_text = ss.str();

    return cmd;
}

} // namespace DroneControl

