/**
 * @file mavlink_telemetry.hpp
 * @brief Module Phát Sóng Telemetry MAVLink v2 & Nhận Lệnh từ QGroundControl (QGC) qua WiFi UDP trên ESP32
 * 
 * Tính năng chuẩn GCS:
 * 1. Tự động phát mạng WiFi SoftAP độc lập: "ESP32_DRONE_GCS", pass "12345678"
 * 2. Mở cổng UDP 14550 chuẩn của QGroundControl (QGC tự động nhận diện kết nối).
 * 3. Phát định kỳ các gói tin MAVLink chuẩn:
 *    - HEARTBEAT (1 Hz): Báo danh tính Quadrotor, trạng thái ARM / DISARM.
 *    - ATTITUDE (30 Hz): Chân trời nhân tạo 3D (Roll, Pitch, Yaw).
 *    - LOCAL_POSITION_NED (10 Hz): Tọa độ không gian (X, Y, Z, Vx, Vy, Vz).
 *    - VFR_HUD (10 Hz): Độ cao, tốc độ bay, la bàn, mức ga.
 *    - GPS_RAW_INT (5 Hz): Tọa độ GPS 3D Fix 14 vệ tinh (hiển thị Drone trên bản đồ vệ tinh QGC).
 *    - SYS_STATUS (1 Hz): Điện áp pin 12.0V (3S LiPo), mức pin 92%.
 *    - STATUSTEXT: Thông báo trạng thái bằng giọng nói / chữ trên QGC.
 * 4. Tiếp nhận lệnh từ QGC (ARM, DISARM, TAKEOFF, LAND, PARAM_REQUEST).
 */

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <cmath>

#if __has_include(<common/mavlink.h>)
#include <common/mavlink.h>
#elif __has_include("mavlink/common/mavlink.h")
#include "mavlink/common/mavlink.h"
#elif __has_include("include/mavlink/common/mavlink.h")
#include "include/mavlink/common/mavlink.h"
#else
#error "MAVLink library not found! Please ensure mavlink folder is in the sketch or in Arduino libraries."
#endif

#include "flight_config.h"
#include "telemetry_packet.hpp"
#include "math_utils.hpp"

namespace DroneComm {

class MavlinkTelemetry {
public:
    MavlinkTelemetry()
        : system_id_(FlightConfig::GCS::SYSTEM_ID),
          component_id_(FlightConfig::GCS::COMPONENT_ID),
          last_heartbeat_ms_(0),
          last_attitude_ms_(0),
          last_position_ms_(0),
          last_gps_ms_(0),
          last_status_ms_(0),
          remote_port_(FlightConfig::GCS::UDP_PORT),
          has_client_(false),
          is_initialized_(false),
          was_armed_(false) {}

    /**
     * @brief Khởi tạo mạng WiFi SoftAP và mở cổng UDP 14550
     */
    bool init(const char* ssid = FlightConfig::GCS::WIFI_SSID,
              const char* password = FlightConfig::GCS::WIFI_PASSWORD,
              uint16_t local_port = FlightConfig::GCS::UDP_PORT) {
        
        if (!FlightConfig::GCS::ENABLE_WIFI_MAVLINK) {
            return false;
        }

        // Cấu hình SoftAP
        WiFi.disconnect(true);
        delay(40);
        WiFi.mode(WIFI_AP);

        // Thiết lập IP tĩnh cho AP: 192.168.4.1
        IPAddress local_ip(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 1);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_ip, gateway, subnet);

        bool ap_ok = WiFi.softAP(ssid, password);
        delay(60);

        // Mặc định địa chỉ phát broadcast đến toàn bộ mạng con
        remote_ip_ = IPAddress(192, 168, 4, 255);
        remote_port_ = local_port;

        bool udp_ok = udp_.begin(local_port);
        is_initialized_ = ap_ok && udp_ok;
        return is_initialized_;
    }

    /**
     * @brief Cập nhật định kỳ truyền nhận MAVLink (Gọi từ loop() hoặc Core 0 task)
     */
    void update(const SensorPacket& s,
                bool is_armed,
                float target_alt,
                bool& cmd_arm_req,
                bool& cmd_disarm_req,
                bool& cmd_takeoff_req,
                bool& cmd_land_req) {

        if (!is_initialized_) return;

        uint32_t now_ms = millis();

        // 1. Nhận và xử lý lệnh từ QGroundControl (Downlink)
        receive_and_handle_commands(cmd_arm_req, cmd_disarm_req, cmd_takeoff_req, cmd_land_req);

        // Thông báo Statustext khi chuyển trạng thái ARM / DISARM
        if (is_armed != was_armed_) {
            was_armed_ = is_armed;
            if (is_armed) {
                send_statustext(MAV_SEVERITY_INFO, "Drone HITL: ARMED - Motors spinning");
            } else {
                send_statustext(MAV_SEVERITY_INFO, "Drone HITL: DISARMED - Safe mode");
            }
        }

        // 2. Phát gói HEARTBEAT (1 Hz)
        if (now_ms - last_heartbeat_ms_ >= FlightConfig::GCS::HEARTBEAT_PERIOD_MS) {
            last_heartbeat_ms_ = now_ms;
            send_heartbeat(is_armed);
        }

        // 3. Phát gói ATTITUDE (30 Hz)
        if (now_ms - last_attitude_ms_ >= FlightConfig::GCS::ATTITUDE_PERIOD_MS) {
            last_attitude_ms_ = now_ms;
            send_attitude(s.roll, s.pitch, s.yaw, s.p, s.q, s.r);
        }

        // 4. Phát gói LOCAL_POSITION_NED & VFR_HUD (10 Hz)
        if (now_ms - last_position_ms_ >= FlightConfig::GCS::POSITION_PERIOD_MS) {
            last_position_ms_ = now_ms;
            send_local_position_ned(s.x, s.y, s.z, s.vx, s.vy, s.vz);
            send_vfr_hud(s.vx, s.vy, s.vz, s.yaw, s.z, is_armed);
        }

        // 5. Phát gói GPS_RAW_INT (5 Hz - Định vị giả lập để hiện trên bản đồ vệ tinh QGC)
        if (now_ms - last_gps_ms_ >= 200) {
            last_gps_ms_ = now_ms;
            send_gps_raw(s.x, s.y, s.z, s.vx, s.vy, s.yaw);
        }

        // 6. Phát gói SYS_STATUS (1 Hz)
        if (now_ms - last_status_ms_ >= FlightConfig::GCS::STATUS_PERIOD_MS) {
            last_status_ms_ = now_ms;
            send_sys_status(is_armed);
        }
    }

    bool is_initialized() const { return is_initialized_; }
    bool has_client() const { return has_client_; }
    IPAddress get_client_ip() const { return remote_ip_; }

private:
    uint8_t system_id_;
    uint8_t component_id_;
    WiFiUDP udp_;
    IPAddress remote_ip_;
    uint16_t remote_port_;

    uint32_t last_heartbeat_ms_;
    uint32_t last_attitude_ms_;
    uint32_t last_position_ms_;
    uint32_t last_gps_ms_;
    uint32_t last_status_ms_;

    bool has_client_;
    bool is_initialized_;
    bool was_armed_;

    /**
     * @brief Đóng gói và gửi một message MAVLink qua WiFi UDP
     */
    void send_message(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (len > 0) {
            udp_.beginPacket(remote_ip_, remote_port_);
            udp_.write(buf, len);
            udp_.endPacket();
        }
    }

    /**
     * @brief Phát gói HEARTBEAT (Báo danh tính Quadrotor và cờ ARM cho QGC)
     */
    void send_heartbeat(bool is_armed) {
        uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        if (is_armed) {
            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }

        uint32_t custom_mode = 0; // 0: STABILIZED / POSHOLD
        uint8_t system_status = is_armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
                                   MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC,
                                   base_mode,
                                   custom_mode,
                                   system_status);
        send_message(msg);
    }

    /**
     * @brief Phát gói ATTITUDE (Góc nghiêng Roll/Pitch/Yaw và tốc độ góc p, q, r)
     */
    void send_attitude(float roll, float pitch, float yaw, float p, float q, float r) {
        mavlink_message_t msg;
        mavlink_msg_attitude_pack(system_id_, component_id_, &msg,
                                  millis(),
                                  roll,
                                  pitch,
                                  yaw,
                                  p,
                                  q,
                                  r);
        send_message(msg);
    }

    /**
     * @brief Phát gói LOCAL_POSITION_NED (Hệ tọa độ NED: Z âm là bay lên)
     */
    void send_local_position_ned(float x, float y, float z, float vx, float vy, float vz) {
        mavlink_message_t msg;
        // Quy ước NED: Z hướng xuống đất -> Độ cao thực tế Z thế giới thành -Z trong NED
        mavlink_msg_local_position_ned_pack(system_id_, component_id_, &msg,
                                            millis(),
                                            x,
                                            y,
                                            -z,
                                            vx,
                                            vy,
                                            -vz);
        send_message(msg);
    }

    /**
     * @brief Phát gói VFR_HUD (Hiển thị tốc độ, độ cao, la bàn trên màn hình chính QGC)
     */
    void send_vfr_hud(float vx, float vy, float vz, float yaw, float z, bool is_armed) {
        float groundspeed = std::sqrt(vx * vx + vy * vy);
        float heading_deg = DroneMath::rad2deg(DroneMath::normalize_angle(yaw));
        if (heading_deg < 0.0f) heading_deg += 360.0f;
        uint16_t throttle = is_armed ? 50 : 0; // Hiển thị 50% ga khi armed

        mavlink_message_t msg;
        mavlink_msg_vfr_hud_pack(system_id_, component_id_, &msg,
                                 groundspeed,
                                 groundspeed,
                                 static_cast<int16_t>(heading_deg),
                                 throttle,
                                 z,
                                 vz);
        send_message(msg);
    }

    /**
     * @brief Phát gói GPS_RAW_INT (Định vị 3D Fix 14 vệ tinh để hiện icon trên map QGC)
     */
    void send_gps_raw(float x, float y, float z, float vx, float vy, float yaw) {
        // Tọa độ gốc giả lập (Hà Nội: 21.028511 N, 105.854167 E)
        constexpr double HOME_LAT = 21.028511;
        constexpr double HOME_LON = 105.854167;
        // 1 độ kinh/vĩ ~ 111319.5 mét
        double lat_deg = HOME_LAT + (static_cast<double>(y) / 111319.5);
        double lon_deg = HOME_LON + (static_cast<double>(x) / (111319.5 * 0.9334));

        int32_t lat_e7 = static_cast<int32_t>(lat_deg * 1e7);
        int32_t lon_e7 = static_cast<int32_t>(lon_deg * 1e7);
        int32_t alt_mm = static_cast<int32_t>((10.0f + z) * 1000.0f); // 10m MSL + độ cao z

        float groundspeed = std::sqrt(vx * vx + vy * vy);
        uint16_t vel_cm = static_cast<uint16_t>(groundspeed * 100.0f);
        float heading_deg = DroneMath::rad2deg(DroneMath::normalize_angle(yaw));
        if (heading_deg < 0.0f) heading_deg += 360.0f;
        uint16_t cog_cdeg = static_cast<uint16_t>(heading_deg * 100.0f);

        mavlink_message_t msg;
        mavlink_msg_gps_raw_int_pack(
            system_id_, component_id_, &msg,
            static_cast<uint64_t>(millis()) * 1000ULL,
            3,               // fix_type: 3D Fix
            lat_e7,
            lon_e7,
            alt_mm,
            120,             // eph: HDOP 1.2
            150,             // epv: VDOP 1.5
            vel_cm,
            cog_cdeg,
            14,              // satellites_visible: 14 vệ tinh
            alt_mm,
            0, 0, 0, 0,
            cog_cdeg
        );
        send_message(msg);
    }

    /**
     * @brief Phát gói SYS_STATUS (Báo pin và tình trạng hệ thống)
     */
    void send_sys_status(bool is_armed) {
        mavlink_message_t msg;
        mavlink_msg_sys_status_pack(
            system_id_, component_id_, &msg,
            0, 0, 0, // Cảm biến tích hợp
            500,     // Tải CPU: 50%
            12000,   // Điện áp pin: 12.0V (Pin 3S LiPo)
            is_armed ? 1500 : 200, // Dòng điện: 15A khi bay, 2A khi chờ
            92,      // Phần trăm pin còn lại: 92%
            0, 0,    // drop_rate_comm, errors_comm
            0, 0, 0, 0, // errors_count1..4
            0, 0, 0  // onboard_control_sensors_present/enabled/health_extended
        );
        send_message(msg);
    }

    /**
     * @brief Gửi thông báo chữ / âm thanh lên màn hình QGC
     */
    void send_statustext(uint8_t severity, const char* text) {
        mavlink_message_t msg;
        mavlink_msg_statustext_pack(system_id_, component_id_, &msg,
                                    severity,
                                    text,
                                    0, 0);
        send_message(msg);
    }

    /**
     * @brief Phản hồi thông tin phiên bản Autopilot khi QGC hỏi
     */
    void send_autopilot_version() {
        mavlink_message_t msg;
        mavlink_msg_autopilot_version_pack(
            system_id_, component_id_, &msg,
            0, 0, 0, 0, 0,
            nullptr, nullptr, nullptr,
            0, 0, 0,
            nullptr
        );
        send_message(msg);
    }

    /**
     * @brief Tiếp nhận và giải mã các lệnh từ QGroundControl
     */
    void receive_and_handle_commands(bool& cmd_arm, bool& cmd_disarm, bool& cmd_takeoff, bool& cmd_land) {
        int packet_size = udp_.parsePacket();
        if (packet_size <= 0) return;

        // Lưu lại địa chỉ IP của thiết bị QGC vừa gửi tới
        remote_ip_ = udp_.remoteIP();
        remote_port_ = udp_.remotePort();
        if (remote_port_ == 0) remote_port_ = FlightConfig::GCS::UDP_PORT;
        has_client_ = true;

        uint8_t rx_buf[256];
        int len = udp_.read(rx_buf, sizeof(rx_buf));

        for (int i = 0; i < len; ++i) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)) {
                
                // 1. Lệnh COMMAND_LONG (ARM, DISARM, TAKEOFF, LAND)
                if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                    mavlink_command_long_t cmd;
                    mavlink_msg_command_long_decode(&msg, &cmd);

                    uint8_t result = MAV_RESULT_ACCEPTED;

                    switch (cmd.command) {
                        case MAV_CMD_COMPONENT_ARM_DISARM:
                            if (cmd.param1 == 1.0f) {
                                cmd_arm = true;
                                send_statustext(MAV_SEVERITY_INFO, "QGC Command: ARM accepted");
                            } else if (cmd.param1 == 0.0f) {
                                cmd_disarm = true;
                                send_statustext(MAV_SEVERITY_INFO, "QGC Command: DISARM accepted");
                            }
                            break;

                        case MAV_CMD_NAV_TAKEOFF:
                            cmd_takeoff = true;
                            cmd_arm = true;
                            send_statustext(MAV_SEVERITY_INFO, "QGC Command: TAKEOFF accepted");
                            break;

                        case MAV_CMD_NAV_LAND:
                            cmd_land = true;
                            send_statustext(MAV_SEVERITY_INFO, "QGC Command: LAND accepted");
                            break;

                        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            send_autopilot_version();
                            break;

                        default:
                            result = MAV_RESULT_UNSUPPORTED;
                            break;
                    }

                    // Phản hồi gói COMMAND_ACK xác nhận thực thi lệnh cho QGC
                    mavlink_message_t ack_msg;
                    mavlink_msg_command_ack_pack(
                        system_id_, component_id_, &ack_msg,
                        cmd.command, result,
                        255, 0,
                        msg.sysid, msg.compid
                    );
                    send_message(ack_msg);
                }
                // 2. Phản hồi PARAM_REQUEST_LIST để QGC hoàn thành đồng bộ thông số lập tức
                else if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST || msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
                    mavlink_message_t pmsg;
                    char param_id[17] = "SYSID_THISMAV";
                    mavlink_msg_param_value_pack(
                        system_id_, component_id_, &pmsg,
                        param_id,
                        1.0f,
                        MAV_PARAM_TYPE_REAL32,
                        1, // param_count
                        0  // param_index
                    );
                    send_message(pmsg);
                }
            }
        }
    }
};

} // namespace DroneComm
