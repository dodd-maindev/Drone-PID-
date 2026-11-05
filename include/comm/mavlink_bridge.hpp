#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>
#include <vector>
#include <netinet/in.h>
#include "core/vehicle_state.hpp"
#include "control/waypoint_navigator.hpp"

namespace DroneComm {

/**
 * @brief Cầu nối phát & nhận MAVLink qua UDP kết nối trực tiếp với QGroundControl (QGC)
 */
class MavlinkBridge {
public:
    MavlinkBridge();
    ~MavlinkBridge();

    /**
     * @brief Khởi động luồng MAVLink kết nối QGC
     * @param state_ptr Con trỏ trạng thái máy bay VehicleState
     * @param qgc_ip IP của QGC (mặc định để trống sẽ tự động phát tới 127.0.0.1 và Windows WSL Gateway)
     * @param qgc_port Cổng UDP của QGC (mặc định 14550)
     * @param local_port Cổng local lắng nghe (mặc định 14540)
     */
    bool init(DroneCore::VehicleState* state_ptr,
              const std::string& qgc_ip = "",
              int qgc_port = 14550,
              int local_port = 14540);

    /**
     * @brief Dừng luồng MAVLink
     */
    void shutdown();

    /**
     * @brief Gán bộ điều hướng Waypoints
     */
    void set_navigator(DroneControl::WaypointNavigator* nav);

    /**
     * @brief Gửi thông điệp chữ (Pop-up/Voice) hiển thị trên QGroundControl
     * @param text Nội dung thông báo (tối đa 50 ký tự)
     * @param severity Mức độ nghiêm trọng (MAV_SEVERITY: 6=INFO, 4=WARNING, 3=ERROR)
     */
    void send_statustext(const std::string& text, uint8_t severity = 6);

    /**
     * @brief Gửi sự kiện Waypoint tới QGC
     */
    void send_mission_current(uint16_t seq);
    void send_mission_item_reached(uint16_t seq);

    /**
     * @brief Đăng ký callback khi nhận lệnh Arm/Disarm từ QGC
     */
    void set_arm_callback(std::function<void(bool arm)> cb) {
        arm_callback_ = cb;
    }

    bool is_running() const { return running_.load(); }
    bool is_qgc_connected() const { return qgc_connected_.load(); }
    void set_auto_allowed(bool allowed) { auto_allowed_.store(allowed); }
    bool is_auto_allowed() const { return auto_allowed_.load(); }

private:
    void thread_worker();
    void process_incoming_packets();
    void send_heartbeat();
    void send_sys_status();
    void send_extended_sys_state();
    void send_home_position();
    void send_attitude();
    void send_local_position_ned();
    void send_global_position_int();
    void send_gps_raw_int();
    void send_vfr_hud();
    void send_mission_count(uint16_t count, uint8_t mission_type = 0);
    void send_mission_item_int(const DroneControl::WaypointItem& wp);
    void send_mission_request_int(uint16_t seq, uint8_t mission_type = 0);
    void send_mission_ack(uint8_t result_type, uint8_t mission_type = 0); // 0 = MAV_MISSION_TYPE_MISSION
    void send_mission_count(uint16_t count, uint8_t mission_type = 0, uint8_t target_sys = 255, uint8_t target_comp = 0);
    void send_mission_item_int(const DroneControl::WaypointItem& wp, uint8_t target_sys = 255, uint8_t target_comp = 0);
    void send_mission_request_int(uint16_t seq, uint8_t mission_type = 0, uint8_t target_sys = 255, uint8_t target_comp = 0);
    void send_mission_ack(uint8_t result_type, uint8_t mission_type = 0, uint8_t target_sys = 255, uint8_t target_comp = 0); // 0 = MAV_MISSION_TYPE_MISSION
    void send_raw(const void* buf, size_t len);

    std::string detect_wsl_host_ip();

    DroneCore::VehicleState* state_{nullptr};
    DroneControl::WaypointNavigator* navigator_{nullptr};
    std::atomic<bool> running_{false};
    std::atomic<bool> auto_allowed_{false};
    std::thread worker_thread_;

    int sockfd_{-1};
    struct sockaddr_in local_addr_;
    std::vector<struct sockaddr_in> target_addrs_;
    struct sockaddr_in qgc_active_addr_;
    std::atomic<bool> has_active_qgc_{false};
    std::atomic<bool> qgc_connected_{false};
    std::chrono::steady_clock::time_point last_heartbeat_rx_;

    std::function<void(bool)> arm_callback_{nullptr};

    // Buffer nhận Waypoint từ QGC
    uint16_t mission_expected_count_{0};
    std::vector<DroneControl::WaypointItem> mission_upload_buffer_;
    std::vector<bool> mission_items_received_;

    uint8_t system_id_{1};
    uint8_t component_id_{1}; // MAV_COMP_ID_AUTOPILOT1
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace DroneComm
