#include "comm/mavlink_bridge.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <common/mavlink.h>

namespace DroneComm {

MavlinkBridge::MavlinkBridge() = default;

MavlinkBridge::~MavlinkBridge() {
    shutdown();
}

std::string MavlinkBridge::detect_wsl_host_ip() {
    // Đọc default gateway từ /proc/net/route
    std::ifstream route_file("/proc/net/route");
    if (!route_file.is_open()) return "";

    std::string line;
    // Bỏ dòng tiêu đề
    std::getline(route_file, line);

    while (std::getline(route_file, line)) {
        std::stringstream ss(line);
        std::string iface;
        unsigned long dest, gateway;
        if (ss >> iface >> std::hex >> dest >> gateway) {
            if (dest == 0 && gateway != 0) {
                // Đây là default gateway
                struct in_addr addr;
                addr.s_addr = gateway;
                return std::string(inet_ntoa(addr));
            }
        }
    }
    return "";
}

bool MavlinkBridge::init(DroneCore::VehicleState* state_ptr,
                         const std::string& qgc_ip,
                         int qgc_port,
                         int local_port) {
    state_ = state_ptr;
    if (!state_) {
        std::cerr << "[MavlinkBridge] [!] Lỗi: VehicleState con trỏ null!" << std::endl;
        return false;
    }

    system_id_ = state_->system_id.load();
    component_id_ = state_->component_id.load();

    // 1. Tạo UDP Socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "[MavlinkBridge] [!] Lỗi tạo UDP socket: " << strerror(errno) << std::endl;
        return false;
    }

    // Thiết lập non-blocking socket
    int flags = fcntl(sockfd_, F_GETFL, 0);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    // Cho phép tái sử dụng địa chỉ và broadcast
    int opt = 1;
    setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(sockfd_, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));

    // 2. Bind vào cổng local để lắng nghe các gói tin từ QGC (14540 hoặc auto)
    std::memset(&local_addr_, 0, sizeof(local_addr_));
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_addr.s_addr = INADDR_ANY;
    local_addr_.sin_port = htons(local_port);

    if (bind(sockfd_, (struct sockaddr*)&local_addr_, sizeof(local_addr_)) < 0) {
        // Nếu cổng 14540 bận, bind tự do sang cổng khác
        local_addr_.sin_port = htons(0);
        if (bind(sockfd_, (struct sockaddr*)&local_addr_, sizeof(local_addr_)) < 0) {
            std::cerr << "[MavlinkBridge] [!] Không thể bind socket: " << strerror(errno) << std::endl;
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }
    }

    // 3. Chuẩn bị danh sách địa chỉ đích của QGC
    target_addrs_.clear();

    auto add_target = [this, qgc_port](const std::string& ip_str) {
        if (ip_str.empty()) return;
        struct sockaddr_in target;
        std::memset(&target, 0, sizeof(target));
        target.sin_family = AF_INET;
        target.sin_port = htons(qgc_port);
        if (inet_pton(AF_INET, ip_str.c_str(), &target.sin_addr) > 0) {
            target_addrs_.push_back(target);
            std::cout << "[MavlinkBridge] Đã thêm địa chỉ QGC đích: " << ip_str << ":" << qgc_port << std::endl;
        }
    };

    if (!qgc_ip.empty()) {
        add_target(qgc_ip);
    } else {
        // Tự động thêm Localhost (127.0.0.1)
        add_target("127.0.0.1");

        // Tự động phát hiện IP Windows Host nếu đang chạy WSL2
        std::string wsl_host = detect_wsl_host_ip();
        if (!wsl_host.empty() && wsl_host != "127.0.0.1") {
            add_target(wsl_host);
        }

        // Thêm broadcast LAN 255.255.255.255
        add_target("255.255.255.255");
    }

    start_time_ = std::chrono::steady_clock::now();
    running_ = true;
    worker_thread_ = std::thread(&MavlinkBridge::thread_worker, this);

    std::cout << "[MavlinkBridge] [✓] Đã khởi động MAVLink Bridge! Sẵn sàng kết nối với QGroundControl." << std::endl;
    return true;
}

void MavlinkBridge::shutdown() {
    if (running_.exchange(false)) {
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        if (sockfd_ >= 0) {
            close(sockfd_);
            sockfd_ = -1;
        }
        std::cout << "[MavlinkBridge] Đã dừng kết nối MAVLink." << std::endl;
    }
}

void MavlinkBridge::set_navigator(DroneControl::WaypointNavigator* nav) {
    navigator_ = nav;
}

void MavlinkBridge::send_raw(const void* buf, size_t len) {
    if (sockfd_ < 0 || len == 0) return;

    // Nếu đã nhận gói từ QGC -> ưu tiên gửi duy nhất tới QGC đang hoạt động,
    // tránh gửi đúp (duplicate packets) gây lỗi MAVLink Mission Protocol transaction!
    if (has_active_qgc_.load()) {
        sendto(sockfd_, buf, len, 0, (struct sockaddr*)&qgc_active_addr_, sizeof(qgc_active_addr_));
    } else {
        // Nếu chưa bắt được QGC -> gửi tới các địa chỉ mục tiêu để thăm dò kết nối
        for (const auto& target : target_addrs_) {
            sendto(sockfd_, buf, len, 0, (struct sockaddr*)&target, sizeof(target));
        }
    }
}

void MavlinkBridge::send_heartbeat() {
    mavlink_message_t msg;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                        MAV_MODE_FLAG_STABILIZE_ENABLED |
                        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                        MAV_MODE_FLAG_GUIDED_ENABLED |
                        MAV_MODE_FLAG_AUTO_ENABLED;

    bool armed_or_auto = state_->is_armed.load() || auto_allowed_.load();
    if (armed_or_auto) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    uint8_t system_status = armed_or_auto ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

    // Mã hóa PX4 Custom Mode: main_mode ở byte 2 (shift 16), sub_mode ở byte 3 (shift 24)
    // Khi chạy --auto, để mặc định ở AUTO_LOITER / HOLD ((4 << 16) | (3 << 24)) để QGC luôn bật tính năng "Go to location" trên Map!
    uint32_t custom_mode = auto_allowed_.load() ? ((4 << 16) | (3 << 24)) : (3 << 16);
    if (navigator_ && navigator_->is_auto_active()) {
        auto nm = navigator_->get_nav_mode();
        if (nm == DroneControl::NavMode::MISSION) {
            custom_mode = (4 << 16) | (4 << 24); // AUTO_MISSION
        } else if (nm == DroneControl::NavMode::GUIDED) {
            custom_mode = (4 << 16) | (3 << 24); // AUTO_LOITER / HOLD
        } else if (nm == DroneControl::NavMode::AUTO_LAND) {
            custom_mode = (4 << 16) | (6 << 24); // AUTO_LAND
        }
    }

    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
                               MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_PX4,
                               base_mode,
                               custom_mode,
                               system_status);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_sys_status() {
    mavlink_message_t msg;
    // Bật đầy đủ các cờ cảm biến bắt buộc (GPS, AHRS, RC, Pre-arm check) để QGC luôn báo "Armed / Ready To Fly" (Xanh lá) thay vì "Not Ready"
    uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO |
                       MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                       MAV_SYS_STATUS_SENSOR_3D_MAG |
                       MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                       MAV_SYS_STATUS_SENSOR_GPS |
                       MAV_SYS_STATUS_AHRS |
                       MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                       MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                       MAV_SYS_STATUS_PREARM_CHECK;

    // Giả lập pin 4S LiPo đầy 16.8V, 100%
    uint16_t voltage_battery = 16800; // mV
    int16_t current_battery = 1200;   // 12A (10mA unit)
    int8_t battery_remaining = 98;    // %

    mavlink_msg_sys_status_pack(system_id_, component_id_, &msg,
                                sensors, sensors, sensors,
                                150, // cpu load 15%
                                voltage_battery,
                                current_battery,
                                battery_remaining,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_extended_sys_state() {
    mavlink_message_t msg;
    // Báo trạng thái IN_AIR khi đang bay hoặc đang ở chế độ --auto để QGroundControl luôn hiển thị menu "Go to location" khi click lên bản đồ!
    uint8_t landed_state = (state_->is_armed.load() || auto_allowed_.load())
                           ? MAV_LANDED_STATE_IN_AIR
                           : MAV_LANDED_STATE_ON_GROUND;

    mavlink_msg_extended_sys_state_pack(system_id_, component_id_, &msg,
                                        MAV_VTOL_STATE_UNDEFINED, landed_state);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_home_position() {
    mavlink_message_t msg;
    const double home_lat = navigator_ ? navigator_->get_home_lat() : 21.028511;
    const double home_lon = navigator_ ? navigator_->get_home_lon() : 105.854167;
    int32_t lat = static_cast<int32_t>(home_lat * 1e7);
    int32_t lon = static_cast<int32_t>(home_lon * 1e7);
    int32_t alt_mm = 15000; // 15m MSL
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    mavlink_msg_home_position_pack(system_id_, component_id_, &msg,
                                   lat, lon, alt_mm,
                                   0.0f, 0.0f, 0.0f,
                                   q, 0.0f, 0.0f, 0.0f, 0);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_attitude() {
    mavlink_message_t msg;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    float roll = state_->roll.load();
    float pitch = state_->pitch.load();
    float yaw = state_->yaw.load();
    float p = state_->p.load();
    float q = state_->q.load();
    float r = state_->r.load();

    mavlink_msg_attitude_pack(system_id_, component_id_, &msg,
                              static_cast<uint32_t>(now_ms),
                              roll, pitch, yaw,
                              p, q, r);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_local_position_ned() {
    mavlink_message_t msg;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    // Hệ tọa độ Gazebo FLU -> Hệ NED trong MAVLink:
    // X_ned = pos_x (North)
    // Y_ned = -pos_y (East)
    // Z_ned = -altitude (Down)
    float x = state_->pos_x.load();
    float y = state_->pos_y.load();
    float z = -state_->altitude.load();

    float vx = state_->vx.load();
    float vy = state_->vy.load();
    float vz = -state_->vz.load();

    mavlink_msg_local_position_ned_pack(system_id_, component_id_, &msg,
                                        static_cast<uint32_t>(now_ms),
                                        x, y, z,
                                        vx, vy, vz);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_global_position_int() {
    mavlink_message_t msg;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    // Tọa độ gốc giả lập (Hà Nội: 21.028511° N, 105.854167° E)
    const double home_lat = 21.028511;
    const double home_lon = 105.854167;
    const double meters_per_deg_lat = 111320.0;
    const double meters_per_deg_lon = 111320.0 * std::cos(home_lat * M_PI / 180.0);

    float px = state_->pos_x.load();
    float py = state_->pos_y.load();
    float alt = state_->altitude.load();

    int32_t lat = static_cast<int32_t>((home_lat + (px / meters_per_deg_lat)) * 1e7);
    int32_t lon = static_cast<int32_t>((home_lon + (py / meters_per_deg_lon)) * 1e7);
    int32_t alt_mm = static_cast<int32_t>((alt + 15.0f) * 1000.0f); // 15m MSL
    int32_t rel_alt_mm = static_cast<int32_t>(alt * 1000.0f);

    int16_t vx_cm = static_cast<int16_t>(state_->vx.load() * 100.0f);
    int16_t vy_cm = static_cast<int16_t>(state_->vy.load() * 100.0f);
    int16_t vz_cm = static_cast<int16_t>(-state_->vz.load() * 100.0f);

    float yaw_deg = state_->yaw.load() * 180.0f / static_cast<float>(M_PI);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;
    uint16_t hdg = static_cast<uint16_t>(yaw_deg * 100.0f);

    mavlink_msg_global_position_int_pack(system_id_, component_id_, &msg,
                                         static_cast<uint32_t>(now_ms),
                                         lat, lon, alt_mm, rel_alt_mm,
                                         vx_cm, vy_cm, vz_cm, hdg);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_vfr_hud() {
    mavlink_message_t msg;
    float vx = state_->vx.load();
    float vy = state_->vy.load();
    float speed = std::sqrt(vx * vx + vy * vy);

    float yaw_deg = state_->yaw.load() * 180.0f / static_cast<float>(M_PI);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;

    mavlink_msg_vfr_hud_pack(system_id_, component_id_, &msg,
                             speed, // airspeed
                             speed, // groundspeed
                             static_cast<int16_t>(yaw_deg),
                             static_cast<uint16_t>(state_->current_thrust.load() * 100.0f),
                             state_->altitude.load(),
                             state_->vz.load());

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_gps_raw_int() {
    mavlink_message_t msg;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    const double home_lat = navigator_ ? navigator_->get_home_lat() : 21.028511;
    const double home_lon = navigator_ ? navigator_->get_home_lon() : 105.854167;
    const double meters_per_deg_lat = 111320.0;
    const double meters_per_deg_lon = 111320.0 * std::cos(home_lat * M_PI / 180.0);

    float px = state_->pos_x.load();
    float py = state_->pos_y.load();
    float alt = state_->altitude.load();

    int32_t lat = static_cast<int32_t>((home_lat + (px / meters_per_deg_lat)) * 1e7);
    int32_t lon = static_cast<int32_t>((home_lon + (py / meters_per_deg_lon)) * 1e7);
    int32_t alt_mm = static_cast<int32_t>((alt + 15.0f) * 1000.0f); // 15m MSL

    float vx = state_->vx.load();
    float vy = state_->vy.load();
    uint16_t vel_cm = static_cast<uint16_t>(std::sqrt(vx * vx + vy * vy) * 100.0f);

    float yaw_deg = state_->yaw.load() * 180.0f / static_cast<float>(M_PI);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;
    uint16_t cog_cdeg = static_cast<uint16_t>(yaw_deg * 100.0f);

    mavlink_msg_gps_raw_int_pack(system_id_, component_id_, &msg,
                                 static_cast<uint64_t>(now_ms) * 1000ULL,
                                 3, // fix_type: 3 = 3D Fix
                                 lat, lon, alt_mm,
                                 100, // eph: HDOP 1.0m
                                 100, // epv: VDOP 1.0m
                                 vel_cm,
                                 cog_cdeg,
                                 16, // satellites_visible = 16
                                 alt_mm, // alt_ellipsoid
                                 100, // h_acc
                                 100, // v_acc
                                 10,  // vel_acc
                                 100, // hdg_acc
                                 cog_cdeg); // yaw

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_mission_current(uint16_t seq) {
    mavlink_message_t msg;
    uint16_t total = navigator_ ? static_cast<uint16_t>(navigator_->get_waypoint_count()) : 0;
    uint8_t mode = (navigator_ && navigator_->is_auto_active()) ? 1 : 0;
    mavlink_msg_mission_current_pack(system_id_, component_id_, &msg,
                                     seq, total, 0, mode, 0, 0, 0);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_mission_item_reached(uint16_t seq) {
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_pack(system_id_, component_id_, &msg, seq);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_mission_count(uint16_t count, uint8_t mission_type, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(system_id_, component_id_, &msg,
                                   target_sys, target_comp, count, mission_type, 0);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_mission_request_int(uint16_t seq, uint8_t mission_type, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t msg;
    mavlink_msg_mission_request_int_pack(system_id_, component_id_, &msg,
                                         target_sys, target_comp, seq, mission_type);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_mission_item_int(const DroneControl::WaypointItem& wp, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t msg;
    int32_t lat_int = static_cast<int32_t>(wp.lat * 1e7);
    int32_t lon_int = static_cast<int32_t>(wp.lon * 1e7);

    mavlink_msg_mission_item_int_pack(system_id_, component_id_, &msg,
                                      target_sys, target_comp,
                                      wp.seq,
                                      MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                      wp.command,
                                      (wp.seq == 0) ? 1 : 0,
                                      1, // autocontinue
                                      wp.hold_time,
                                      wp.accept_radius,
                                      0.0f,
                                      wp.yaw_deg,
                                      lat_int, lon_int, wp.z,
                                      MAV_MISSION_TYPE_MISSION);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_mission_ack(uint8_t result_type, uint8_t mission_type, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t msg;
    mavlink_msg_mission_ack_pack(system_id_, component_id_, &msg,
                                 target_sys, target_comp, result_type, mission_type, 0);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_statustext(const std::string& text, uint8_t severity) {
    mavlink_message_t msg;
    char text_buf[50];
    std::memset(text_buf, 0, sizeof(text_buf));
    std::strncpy(text_buf, text.c_str(), sizeof(text_buf) - 1);

    mavlink_msg_statustext_pack(system_id_, component_id_, &msg,
                                severity, text_buf, 0, 0);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::process_incoming_packets() {
    uint8_t recv_buf[2048];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    while (true) {
        ssize_t n = recvfrom(sockfd_, recv_buf, sizeof(recv_buf), 0,
                             (struct sockaddr*)&from_addr, &from_len);
        if (n <= 0) {
            break; // Không còn gói tin trong hàng đợi socket
        }

        // Cập nhật địa chỉ QGC đang hoạt động
        qgc_active_addr_ = from_addr;
        has_active_qgc_.store(true);
        qgc_connected_.store(true);
        last_heartbeat_rx_ = std::chrono::steady_clock::now();

        mavlink_message_t msg;
        mavlink_status_t status;

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, recv_buf[i], &msg, &status)) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                        mavlink_message_t pmsg;
                        mavlink_msg_param_value_pack(system_id_, component_id_, &pmsg,
                                                     "SYS_AUTOSTART", 4001.0f,
                                                     MAV_PARAM_TYPE_REAL32, 1, 0);
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                        uint16_t len = mavlink_msg_to_send_buffer(buf, &pmsg);
                        send_raw(buf, len);
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                        mavlink_param_request_read_t req;
                        mavlink_msg_param_request_read_decode(&msg, &req);
                        mavlink_message_t pmsg;
                        mavlink_msg_param_value_pack(system_id_, component_id_, &pmsg,
                                                     req.param_id, 1.0f,
                                                     MAV_PARAM_TYPE_REAL32, 1, 0);
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                        uint16_t len = mavlink_msg_to_send_buffer(buf, &pmsg);
                        send_raw(buf, len);
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_COUNT: {
                        mavlink_mission_count_t mc;
                        mavlink_msg_mission_count_decode(&msg, &mc);

                        uint8_t m_type = mc.mission_type;
                        std::cout << "[MavlinkBridge] QGC gửi MISSION_COUNT: type=" << (int)m_type 
                                  << ", count=" << mc.count << " từ sys=" << (int)msg.sysid << std::endl;

                        if (m_type == MAV_MISSION_TYPE_MISSION) {
                            mission_expected_count_ = mc.count;
                            mission_upload_buffer_.clear();
                            if (mission_expected_count_ == 0) {
                                if (navigator_) navigator_->clear_mission();
                                send_mission_ack(MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, msg.sysid, msg.compid);
                            } else {
                                mission_upload_buffer_.resize(mc.count);
                                mission_items_received_.assign(mc.count, false);
                                send_mission_request_int(0, MAV_MISSION_TYPE_MISSION, msg.sysid, msg.compid);
                            }
                        } else {
                            // GeoFence (1) hoặc Rally Points (2):
                            // Trả lời ACK ACCEPTED với đúng target sys/comp và đúng mission_type
                            // để QGC hoàn thành nạp Plan mà không bao giờ báo lỗi GeoFence / Rally Point failed!
                            send_mission_ack(MAV_MISSION_ACCEPTED, m_type, msg.sysid, msg.compid);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
                        mavlink_mission_item_int_t item;
                        mavlink_msg_mission_item_int_decode(&msg, &item);

                        if (item.mission_type != MAV_MISSION_TYPE_MISSION) {
                            send_mission_ack(MAV_MISSION_ACCEPTED, item.mission_type, msg.sysid, msg.compid);
                            break;
                        }

                        if (item.seq < mission_upload_buffer_.size()) {
                            DroneControl::WaypointItem wp;
                            wp.seq = item.seq;
                            wp.command = item.command;
                            wp.lat = static_cast<double>(item.x) / 1e7;
                            wp.lon = static_cast<double>(item.y) / 1e7;

                            // Nếu là lệnh Return To Launch (MAV_CMD_NAV_RETURN_TO_LAUNCH = 20) hoặc tọa độ (0,0):
                            // QGC gửi x=0, y=0 cho mục Return -> Gán về đúng tọa độ Home (Planned Home WP#0 hoặc Home gốc)
                            // để drone bay về đúng biểu tượng chữ H và dừng lại, thay vì bay tới (lat=0, lon=0) cách 11,000km!
                            if (item.command == MAV_CMD_NAV_RETURN_TO_LAUNCH ||
                                (std::abs(wp.lat) < 0.1 && std::abs(wp.lon) < 0.1)) {
                                if (!mission_upload_buffer_.empty() && item.seq > 0 &&
                                    (std::abs(mission_upload_buffer_[0].lat) > 0.1 || std::abs(mission_upload_buffer_[0].lon) > 0.1)) {
                                    wp.lat = mission_upload_buffer_[0].lat;
                                    wp.lon = mission_upload_buffer_[0].lon;
                                } else if (navigator_) {
                                    wp.lat = navigator_->get_home_lat();
                                    wp.lon = navigator_->get_home_lon();
                                }
                            }

                            // Giới hạn độ cao mục tiêu trong trần bay của ESP32 (1.5m -> 9.5m, tránh mặc định 50m của QGC làm kẹt trần 10m)
                            wp.z = (std::isnan(item.z) || item.z < 1.0f) ? 3.5f : std::min(item.z, 9.5f);
                            if (item.command == MAV_CMD_NAV_RETURN_TO_LAUNCH && item.seq > 0 && mission_upload_buffer_[item.seq - 1].z >= 1.5f) {
                                wp.z = mission_upload_buffer_[item.seq - 1].z;
                            }
                            // Lọc NaN từ QGC (nếu param1 là NaN thì phép so sánh <= 0.05f sẽ luôn false gây kẹt vĩnh viễn tại WP 1!)
                            wp.hold_time = (std::isnan(item.param1) || item.param1 < 0.0f) ? 0.0f : item.param1;
                            wp.accept_radius = (std::isnan(item.param2) || item.param2 < 0.5f) ? 2.5f : item.param2;
                            wp.yaw_deg = std::isnan(item.param4) ? -1.0f : item.param4;

                            if (navigator_) {
                                navigator_->gps_to_local(wp.lat, wp.lon, wp.z, wp.x, wp.y, wp.z);
                            }

                            mission_upload_buffer_[item.seq] = wp;
                            mission_items_received_[item.seq] = true;

                            std::cout << "[MavlinkBridge]   [+] WP #" << item.seq << "/" << mission_expected_count_
                                      << " (cmd=" << item.command << ", Alt=" << wp.z << "m, Hold=" << wp.hold_time << "s, X=" 
                                      << std::fixed << std::setprecision(1) << wp.x << ", Y=" << wp.y << ")" << std::endl;
                        }

                        // Tìm item tiếp theo còn thiếu
                        uint16_t next_seq = 0;
                        bool all_received = true;
                        for (size_t s = 0; s < mission_items_received_.size(); ++s) {
                            if (!mission_items_received_[s]) {
                                next_seq = static_cast<uint16_t>(s);
                                all_received = false;
                                break;
                            }
                        }

                        if (!all_received) {
                            send_mission_request_int(next_seq, MAV_MISSION_TYPE_MISSION, msg.sysid, msg.compid);
                        } else {
                            if (navigator_) {
                                navigator_->set_mission(mission_upload_buffer_);
                            }
                            send_mission_ack(MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, msg.sysid, msg.compid);
                            send_mission_current(0);
                            send_statustext("Mission Saved (" + std::to_string(mission_upload_buffer_.size()) + " items)", 6);
                            std::cout << "[MavlinkBridge] [✓] Đã lưu thành công toàn bộ "
                                      << mission_upload_buffer_.size() << " Waypoints từ QGC!" << std::endl;
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_ITEM: {
                        mavlink_mission_item_t item;
                        mavlink_msg_mission_item_decode(&msg, &item);

                        if (item.mission_type != MAV_MISSION_TYPE_MISSION) {
                            send_mission_ack(MAV_MISSION_ACCEPTED, item.mission_type, msg.sysid, msg.compid);
                            break;
                        }

                        if (item.seq < mission_upload_buffer_.size()) {
                            DroneControl::WaypointItem wp;
                            wp.seq = item.seq;
                            wp.command = item.command;
                            wp.lat = static_cast<double>(item.x);
                            wp.lon = static_cast<double>(item.y);

                            if (item.command == MAV_CMD_NAV_RETURN_TO_LAUNCH ||
                                (std::abs(wp.lat) < 0.1 && std::abs(wp.lon) < 0.1)) {
                                if (!mission_upload_buffer_.empty() && item.seq > 0 &&
                                    (std::abs(mission_upload_buffer_[0].lat) > 0.1 || std::abs(mission_upload_buffer_[0].lon) > 0.1)) {
                                    wp.lat = mission_upload_buffer_[0].lat;
                                    wp.lon = mission_upload_buffer_[0].lon;
                                } else if (navigator_) {
                                    wp.lat = navigator_->get_home_lat();
                                    wp.lon = navigator_->get_home_lon();
                                }
                            }

                            wp.z = (std::isnan(item.z) || item.z < 1.0f) ? 3.5f : std::min(item.z, 9.5f);
                            if (item.command == MAV_CMD_NAV_RETURN_TO_LAUNCH && item.seq > 0 && mission_upload_buffer_[item.seq - 1].z >= 1.5f) {
                                wp.z = mission_upload_buffer_[item.seq - 1].z;
                            }
                            wp.hold_time = (std::isnan(item.param1) || item.param1 < 0.0f) ? 0.0f : item.param1;
                            wp.accept_radius = (std::isnan(item.param2) || item.param2 < 0.5f) ? 2.5f : item.param2;
                            wp.yaw_deg = std::isnan(item.param4) ? -1.0f : item.param4;

                            if (navigator_) {
                                navigator_->gps_to_local(wp.lat, wp.lon, wp.z, wp.x, wp.y, wp.z);
                            }

                            mission_upload_buffer_[item.seq] = wp;
                            mission_items_received_[item.seq] = true;
                        }

                        uint16_t next_seq = 0;
                        bool all_received = true;
                        for (size_t s = 0; s < mission_items_received_.size(); ++s) {
                            if (!mission_items_received_[s]) {
                                next_seq = static_cast<uint16_t>(s);
                                all_received = false;
                                break;
                            }
                        }

                        if (!all_received) {
                            send_mission_request_int(next_seq, MAV_MISSION_TYPE_MISSION, msg.sysid, msg.compid);
                        } else {
                            if (navigator_) {
                                navigator_->set_mission(mission_upload_buffer_);
                            }
                            send_mission_ack(MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, msg.sysid, msg.compid);
                            send_mission_current(0);
                            send_statustext("Mission Saved (" + std::to_string(mission_upload_buffer_.size()) + " items)", 6);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
                        mavlink_mission_request_list_t req;
                        mavlink_msg_mission_request_list_decode(&msg, &req);
                        if (req.mission_type == MAV_MISSION_TYPE_MISSION) {
                            uint16_t count = navigator_ ? static_cast<uint16_t>(navigator_->get_waypoint_count()) : 0;
                            send_mission_count(count, MAV_MISSION_TYPE_MISSION, msg.sysid, msg.compid);
                        } else {
                            send_mission_count(0, req.mission_type, msg.sysid, msg.compid);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
                        mavlink_mission_request_int_t req;
                        mavlink_msg_mission_request_int_decode(&msg, &req);
                        if (navigator_) {
                            DroneControl::WaypointItem wp;
                            if (navigator_->get_waypoint(req.seq, wp)) {
                                send_mission_item_int(wp, msg.sysid, msg.compid);
                            }
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_REQUEST: {
                        mavlink_mission_request_t req;
                        mavlink_msg_mission_request_decode(&msg, &req);
                        if (navigator_) {
                            DroneControl::WaypointItem wp;
                            if (navigator_->get_waypoint(req.seq, wp)) {
                                send_mission_item_int(wp, msg.sysid, msg.compid);
                            }
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
                        mavlink_mission_clear_all_t mca;
                        mavlink_msg_mission_clear_all_decode(&msg, &mca);
                        if (mca.mission_type == MAV_MISSION_TYPE_MISSION || mca.mission_type == MAV_MISSION_TYPE_ALL) {
                            if (navigator_) {
                                navigator_->clear_mission();
                            }
                        }
                        send_mission_ack(MAV_MISSION_ACCEPTED, mca.mission_type, msg.sysid, msg.compid);
                        send_statustext("Mission Cleared", 6);
                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
                        mavlink_mission_set_current_t msc;
                        mavlink_msg_mission_set_current_decode(&msg, &msc);
                        if (navigator_) {
                            navigator_->set_current_seq(msc.seq);
                        }
                        send_mission_current(msc.seq);
                        break;
                    }
                    case MAVLINK_MSG_ID_COMMAND_LONG: {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);

                        uint8_t result = MAV_RESULT_ACCEPTED;

                        if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                            bool arm = (cmd.param1 == 1.0f);
                            std::cout << "[MavlinkBridge] Nhận lệnh từ QGC: "
                                      << (arm ? "ARM" : "DISARM") << std::endl;
                            if (arm_callback_) {
                                arm_callback_(arm);
                            }
                            send_statustext(arm ? "Drone Armed by QGC" : "Drone Disarmed by QGC", 6);
                        } else if (cmd.command == MAV_CMD_DO_REPOSITION || cmd.command == MAV_CMD_DO_SET_HOME) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Mode Disabled! Run ./run.sh --auto", 4);
                                std::cout << "[MavlinkBridge] [!] Từ chối Guided Reposition vì đang ở chế độ THỦ CÔNG (chạy ./run.sh --auto để bật)" << std::endl;
                            } else {
                                float yaw_deg = std::isnan(cmd.param4) ? -1.0f : cmd.param4;
                                double lat = static_cast<double>(cmd.param5);
                                double lon = static_cast<double>(cmd.param6);
                                float cur_alt = std::clamp(state_->altitude.load(), 3.5f, 9.5f);
                                float alt = (std::isnan(cmd.param7) || cmd.param7 < 1.0f || cmd.param7 > 9.5f) ? cur_alt : cmd.param7;
                                if (navigator_ && std::abs(lat) > 1.0 && std::abs(lon) > 1.0) {
                                    if (arm_callback_) arm_callback_(true);
                                    float lx, ly, lz;
                                    navigator_->gps_to_local(lat, lon, alt, lx, ly, lz);
                                    navigator_->start_guided_reposition(lx, ly, lz, yaw_deg);
                                    send_statustext("Guided: Flying to clicked location", 6);
                                    std::cout << "[MavlinkBridge] [🎯 CLICK & FLY] Bay tới tọa độ: X=" << lx << "m, Y=" << ly << "m, Z=" << lz << "m" << std::endl;
                                }
                            }
                        } else if (cmd.command == MAV_CMD_MISSION_START) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Mode Disabled! Run ./run.sh --auto", 4);
                                std::cout << "[MavlinkBridge] [!] Từ chối Start Mission vì đang ở chế độ THỦ CÔNG (chạy ./run.sh --auto để bật)" << std::endl;
                            } else {
                                if (navigator_) {
                                    navigator_->start_mission();
                                    send_statustext("Mission Started", 6);
                                }
                            }
                        } else if (cmd.command == MAV_CMD_NAV_TAKEOFF) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Takeoff Disabled! Run ./run.sh --auto", 4);
                            } else {
                                float alt = (cmd.param7 > 0.5f) ? cmd.param7 : 2.5f;
                                if (navigator_) {
                                    float cx = state_->pos_x.load();
                                    float cy = state_->pos_y.load();
                                    navigator_->start_guided_reposition(cx, cy, alt, -1.0f);
                                    send_statustext("Auto Takeoff", 6);
                                }
                            }
                        } else if (cmd.command == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("RTL Disabled! Run ./run.sh --auto", 4);
                            } else if (navigator_) {
                                float cur_alt = std::clamp(state_->altitude.load(), 3.5f, 9.5f);
                                float rx = 0.0f, ry = 0.0f;
                                DroneControl::WaypointItem wp0;
                                if (navigator_->get_waypoint(0, wp0) && (std::abs(wp0.lat) > 0.1 || std::abs(wp0.lon) > 0.1)) {
                                    rx = wp0.x;
                                    ry = wp0.y;
                                }
                                navigator_->start_guided_reposition(rx, ry, cur_alt, 0.0f);
                                send_statustext("RTL: Returning to Home", 6);
                            }
                        } else if (cmd.command == MAV_CMD_NAV_LAND) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Land Disabled! Run ./run.sh --auto", 4);
                            } else {
                                if (navigator_) {
                                    navigator_->start_auto_land();
                                    send_statustext("Auto Land", 6);
                                }
                            }
                        } else if (cmd.command == MAV_CMD_DO_SET_MODE) {
                            uint8_t main_mode = static_cast<uint8_t>(cmd.param2);
                            uint8_t sub_mode = static_cast<uint8_t>(cmd.param3);

                            if (main_mode == 4) { // PX4 AUTO MODE
                                if (!auto_allowed_.load()) {
                                    result = MAV_RESULT_DENIED;
                                    send_statustext("Auto Disabled! Run ./run.sh --auto", 4);
                                } else {
                                    if (sub_mode == 4) { // AUTO_MISSION
                                        if (navigator_) navigator_->start_mission();
                                        send_statustext("Mission Started", 6);
                                    } else if (sub_mode == 5) { // AUTO_RTL (Return To Launch)
                                        if (navigator_) {
                                            float cur_alt = std::clamp(state_->altitude.load(), 3.5f, 9.5f);
                                            float rx = 0.0f, ry = 0.0f;
                                            DroneControl::WaypointItem wp0;
                                            if (navigator_->get_waypoint(0, wp0) && (std::abs(wp0.lat) > 0.1 || std::abs(wp0.lon) > 0.1)) {
                                                rx = wp0.x;
                                                ry = wp0.y;
                                            }
                                            navigator_->start_guided_reposition(rx, ry, cur_alt, 0.0f);
                                        }
                                        send_statustext("RTL: Returning to Home", 6);
                                    } else if (sub_mode == 3 || sub_mode == 2) { // LOITER or TAKEOFF
                                        if (navigator_) {
                                            float cx = state_->pos_x.load();
                                            float cy = state_->pos_y.load();
                                            navigator_->start_guided_reposition(cx, cy, 2.5f, -1.0f);
                                        }
                                        send_statustext("Guided Mode Engaged", 6);
                                    } else if (sub_mode == 6) { // AUTO_LAND
                                        if (navigator_) navigator_->start_auto_land();
                                        send_statustext("Auto Land Engaged", 6);
                                    }
                                }
                            } else {
                                if (navigator_) navigator_->cancel_auto_mode("Manual Mode Requested");
                            }
                        } else if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ||
                                   (cmd.command == MAV_CMD_REQUEST_MESSAGE && 
                                    (static_cast<int>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION || static_cast<int>(cmd.param1) == 520))) {
                            mavlink_message_t cap_msg;
                            uint64_t caps = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                                            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
                            // Mã hóa version PX4 v1.14.0 (Major=1, Minor=14, Patch=0, Type=0xFF/Official)
                            uint32_t flight_sw_version = (1 << 24) | (14 << 16) | (0 << 8) | 0xFF;
                            mavlink_msg_autopilot_version_pack(
                                system_id_, component_id_, &cap_msg,
                                caps, flight_sw_version, flight_sw_version, flight_sw_version, 0, nullptr, nullptr, nullptr, 0, 0, 0, nullptr);
                            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                            uint16_t len = mavlink_msg_to_send_buffer(buf, &cap_msg);
                            send_raw(buf, len);
                            result = MAV_RESULT_ACCEPTED;
                        } else if (cmd.command == MAV_CMD_REQUEST_MESSAGE && static_cast<int>(cmd.param1) == MAVLINK_MSG_ID_HOME_POSITION) {
                            send_home_position();
                            result = MAV_RESULT_ACCEPTED;
                        } else if (cmd.command == MAV_CMD_REQUEST_MESSAGE && static_cast<int>(cmd.param1) == MAVLINK_MSG_ID_EXTENDED_SYS_STATE) {
                            send_extended_sys_state();
                            result = MAV_RESULT_ACCEPTED;
                        }

                        // Gửi COMMAND_ACK xác nhận cho QGC
                        mavlink_message_t ack_msg;
                        mavlink_msg_command_ack_pack(system_id_, component_id_, &ack_msg,
                                                     cmd.command, result, 255, 0,
                                                     msg.sysid, msg.compid);
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                        uint16_t len = mavlink_msg_to_send_buffer(buf, &ack_msg);
                        send_raw(buf, len);
                        break;
                    }
                    case MAVLINK_MSG_ID_COMMAND_INT: {
                        mavlink_command_int_t cmd;
                        mavlink_msg_command_int_decode(&msg, &cmd);
                        uint8_t result = MAV_RESULT_ACCEPTED;

                        if (cmd.command == MAV_CMD_DO_REPOSITION || cmd.command == MAV_CMD_DO_SET_HOME) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Mode Disabled! Run ./run.sh --auto", 4);
                                std::cout << "[MavlinkBridge] [!] Từ chối Guided Reposition vì đang ở chế độ THỦ CÔNG (chạy ./run.sh --auto để bật)" << std::endl;
                            } else {
                                float yaw_deg = std::isnan(cmd.param4) ? -1.0f : cmd.param4;
                                double lat = static_cast<double>(cmd.x) / 1e7;
                                double lon = static_cast<double>(cmd.y) / 1e7;
                                float cur_alt = std::clamp(state_->altitude.load(), 3.5f, 9.5f);
                                float alt = (std::isnan(cmd.z) || cmd.z < 1.0f || cmd.z > 9.5f) ? cur_alt : cmd.z;
                                if (navigator_ && std::abs(lat) > 1.0 && std::abs(lon) > 1.0) {
                                    if (arm_callback_) arm_callback_(true);
                                    float lx, ly, lz;
                                    navigator_->gps_to_local(lat, lon, alt, lx, ly, lz);
                                    navigator_->start_guided_reposition(lx, ly, lz, yaw_deg);
                                    send_statustext("Guided: Flying to clicked location", 6);
                                    std::cout << "[MavlinkBridge] [🎯 CLICK & FLY] Bay tới tọa độ: X=" << lx << "m, Y=" << ly << "m, Z=" << lz << "m" << std::endl;
                                }
                            }
                        } else if (cmd.command == MAV_CMD_MISSION_START) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Mode Disabled! Run ./run.sh --auto", 4);
                                std::cout << "[MavlinkBridge] [!] Từ chối Start Mission vì đang ở chế độ THỦ CÔNG (chạy ./run.sh --auto để bật)" << std::endl;
                            } else {
                                if (navigator_) {
                                    navigator_->start_mission();
                                    send_statustext("Mission Started", 6);
                                }
                            }
                        } else if (cmd.command == MAV_CMD_NAV_TAKEOFF) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Takeoff Disabled! Run ./run.sh --auto", 4);
                            } else {
                                float alt = (cmd.z > 0.5f) ? cmd.z : 2.5f;
                                if (navigator_) {
                                    float cx = state_->pos_x.load();
                                    float cy = state_->pos_y.load();
                                    navigator_->start_guided_reposition(cx, cy, alt, -1.0f);
                                    send_statustext("Auto Takeoff", 6);
                                }
                            }
                        } else if (cmd.command == MAV_CMD_NAV_LAND) {
                            if (!auto_allowed_.load()) {
                                result = MAV_RESULT_DENIED;
                                send_statustext("Auto Land Disabled! Run ./run.sh --auto", 4);
                            } else {
                                if (navigator_) {
                                    navigator_->start_auto_land();
                                    send_statustext("Auto Land", 6);
                                }
                            }
                        } else if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ||
                                   (cmd.command == MAV_CMD_REQUEST_MESSAGE && 
                                    (static_cast<int>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION || static_cast<int>(cmd.param1) == 520))) {
                            mavlink_message_t cap_msg;
                            uint64_t caps = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                                            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
                            // Mã hóa version PX4 v1.14.0 (Major=1, Minor=14, Patch=0, Type=0xFF/Official)
                            uint32_t flight_sw_version = (1 << 24) | (14 << 16) | (0 << 8) | 0xFF;
                            mavlink_msg_autopilot_version_pack(
                                system_id_, component_id_, &cap_msg,
                                caps, flight_sw_version, flight_sw_version, flight_sw_version, 0, nullptr, nullptr, nullptr, 0, 0, 0, nullptr);
                            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                            uint16_t len = mavlink_msg_to_send_buffer(buf, &cap_msg);
                            send_raw(buf, len);
                            result = MAV_RESULT_ACCEPTED;
                        }

                        mavlink_message_t ack_msg;
                        mavlink_msg_command_ack_pack(system_id_, component_id_, &ack_msg,
                                                     cmd.command, result, 255, 0,
                                                     msg.sysid, msg.compid);
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                        uint16_t len = mavlink_msg_to_send_buffer(buf, &ack_msg);
                        send_raw(buf, len);
                        break;
                    }
                    case MAVLINK_MSG_ID_SET_MODE: {
                        mavlink_set_mode_t sm;
                        mavlink_msg_set_mode_decode(&msg, &sm);
                        uint8_t main_mode = (sm.custom_mode >> 16) & 0xFF;
                        uint8_t sub_mode = (sm.custom_mode >> 24) & 0xFF;
                        if (main_mode == 4 && auto_allowed_.load()) {
                            if (sub_mode == 4 && navigator_) {
                                navigator_->start_mission();
                                send_statustext("Mission Started", 6);
                            } else if (navigator_) {
                                float cx = state_->pos_x.load();
                                float cy = state_->pos_y.load();
                                navigator_->start_guided_reposition(cx, cy, 2.5f, -1.0f);
                                send_statustext("Guided Mode", 6);
                            }
                        } else if (navigator_) {
                            navigator_->cancel_auto_mode("Manual Mode");
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {
                        mavlink_set_position_target_local_ned_t spt;
                        mavlink_msg_set_position_target_local_ned_decode(&msg, &spt);
                        if (auto_allowed_.load() && navigator_) {
                            // NED: x=North, y=East, z=Down -> Gazebo: x=North, y=-East, z=-Down
                            float gx = spt.x;
                            float gy = -spt.y;
                            float gz = -spt.z;
                            float yaw_deg = spt.yaw * 180.0f / static_cast<float>(M_PI);
                            navigator_->start_guided_reposition(gx, gy, gz, yaw_deg);
                            send_statustext("Guided to Target", 6);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
                        mavlink_set_position_target_global_int_t spt;
                        mavlink_msg_set_position_target_global_int_decode(&msg, &spt);
                        if (auto_allowed_.load() && navigator_) {
                            if (arm_callback_) arm_callback_(true);
                            double lat = static_cast<double>(spt.lat_int) / 1e7;
                            double lon = static_cast<double>(spt.lon_int) / 1e7;
                            float cur_alt = std::clamp(state_->altitude.load(), 3.5f, 9.5f);
                            float alt = (std::isnan(spt.alt) || spt.alt < 1.0f || spt.alt > 9.5f) ? cur_alt : spt.alt;
                            float gx, gy, gz;
                            navigator_->gps_to_local(lat, lon, alt, gx, gy, gz);
                            float yaw_deg = std::isnan(spt.yaw) ? -1.0f : (spt.yaw * 180.0f / static_cast<float>(M_PI));
                            navigator_->start_guided_reposition(gx, gy, gz, yaw_deg);
                            send_statustext("Guided to GPS Target", 6);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN: {
                        mavlink_set_gps_global_origin_t orig;
                        mavlink_msg_set_gps_global_origin_decode(&msg, &orig);
                        if (auto_allowed_.load() && navigator_) {
                            if (arm_callback_) arm_callback_(true);
                            double lat = static_cast<double>(orig.latitude) / 1e7;
                            double lon = static_cast<double>(orig.longitude) / 1e7;
                            float cur_alt = std::clamp(state_->altitude.load(), 3.5f, 9.5f);
                            float gx, gy, gz;
                            navigator_->gps_to_local(lat, lon, cur_alt, gx, gy, gz);
                            navigator_->start_guided_reposition(gx, gy, gz, -1.0f);
                            send_statustext("Guided: Flying to clicked location", 6);
                            std::cout << "[MavlinkBridge] [🎯 CLICK & FLY] Bay tới tọa độ: X=" << gx << "m, Y=" << gy << "m, Z=" << gz << "m" << std::endl;
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
        }
    }
}

void MavlinkBridge::thread_worker() {
    uint32_t loop_count = 0;

    send_statustext("Antigravity C++ Autopilot Online", 6);

    while (running_) {
        // 1. Gói tin tốc độ cao (50Hz - mỗi 20ms)
        send_attitude();
        send_local_position_ned();

        // 2. Gói tin tốc độ trung bình (10Hz - mỗi 100ms)
        if (loop_count % 5 == 0) {
            send_global_position_int();
            send_gps_raw_int();
            send_vfr_hud();
            send_extended_sys_state();
        }

        // 3. Gói tin tốc độ thấp (1Hz - mỗi 1000ms)
        if (loop_count % 50 == 0) {
            send_heartbeat();
            send_sys_status();
            send_home_position();
            if (navigator_ && navigator_->is_auto_active()) {
                send_mission_current(navigator_->get_current_seq());
            }
        }

        // 4. Nhận và xử lý lệnh đến từ QGC
        process_incoming_packets();

        ++loop_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

} // namespace DroneComm
