#include "comm/mavlink_bridge.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <fstream>
#include <sstream>
#include <cmath>

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

void MavlinkBridge::send_raw(const void* buf, size_t len) {
    if (sockfd_ < 0 || len == 0) return;

    // Nếu đã nhận gói từ QGC -> ưu tiên gửi trực tiếp tới QGC đang hoạt động
    if (has_active_qgc_.load()) {
        sendto(sockfd_, buf, len, 0, (struct sockaddr*)&qgc_active_addr_, sizeof(qgc_active_addr_));
    }

    // Đồng thời gửi tới các địa chỉ mục tiêu (localhost, WSL host, broadcast)
    for (const auto& target : target_addrs_) {
        sendto(sockfd_, buf, len, 0, (struct sockaddr*)&target, sizeof(target));
    }
}

void MavlinkBridge::send_heartbeat() {
    mavlink_message_t msg;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (state_->is_armed.load()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    uint8_t system_status = state_->is_connected.load() ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
                               MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC,
                               base_mode,
                               1, // custom mode
                               system_status);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    send_raw(buffer, len);
}

void MavlinkBridge::send_sys_status() {
    mavlink_message_t msg;
    uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO |
                       MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                       MAV_SYS_STATUS_SENSOR_3D_MAG |
                       MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                       MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

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
                        // Nhận heartbeat từ QGC
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                        // Trả lời param list nhanh chóng để QGC không bị xoay chờ parameter
                        std::cout << "[MavlinkBridge] Nhận yêu cầu danh sách tham số từ QGC..." << std::endl;
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
                        } else if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
                            mavlink_message_t cap_msg;
                            mavlink_msg_autopilot_version_pack(
                                system_id_, component_id_, &cap_msg,
                                MAV_PROTOCOL_CAPABILITY_COMMAND_INT | MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET,
                                1, 0, 0, 0, nullptr, nullptr, nullptr, 0, 0, 0, nullptr);
                            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                            uint16_t len = mavlink_msg_to_send_buffer(buf, &cap_msg);
                            send_raw(buf, len);
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
                    default:
                        break;
                }
            }
        }
    }
}

void MavlinkBridge::thread_worker() {
    uint32_t loop_count = 0;

    // Gửi thông báo chào mừng QGC
    send_statustext("Antigravity C++ Autopilot Online", 6);

    while (running_) {
        // 1. Gói tin tốc độ cao (50Hz - mỗi 20ms)
        send_attitude();
        send_local_position_ned();

        // 2. Gói tin tốc độ trung bình (10Hz - mỗi 100ms)
        if (loop_count % 5 == 0) {
            send_global_position_int();
            send_vfr_hud();
        }

        // 3. Gói tin tốc độ thấp (1Hz - mỗi 1000ms)
        if (loop_count % 50 == 0) {
            send_heartbeat();
            send_sys_status();
        }

        // 4. Nhận và xử lý lệnh đến từ QGC
        process_incoming_packets();

        ++loop_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

} // namespace DroneComm
