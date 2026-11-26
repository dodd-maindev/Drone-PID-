/**
 * @file 07_hitl_formation_bridge.cpp
 * @brief Cầu nối HITL Đội hình phân tán 4 Drone (Distributed Formation + ESP32 HITL)
 *
 * - Drone 1 (x500_1): Chạy HITL trực tiếp trên mạch ESP32 thật qua UART 921600 baud (hoặc SMC+HGO nếu --no-esp32).
 * - Drone 2, 3, 4 (x500_2..4): Chạy thuật toán Distributed Consensus + SMC + HGO (a_4_drone.m) trên PC.
 * - Hỗ trợ cả Sensor Model (4x StateEstimator) và Physics trực tiếp (--world-odom / --no-estimator).
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <string>
#include <cstring>
#include <vector>
#include <array>
#include <mutex>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/actuators.pb.h>

#include "comm/telemetry_packet.hpp"
#include "comm/mavlink_bridge.hpp"
#include "core/vehicle_state.hpp"
#include "estimation/state_estimator.hpp"
#include "formation/formation_controller.hpp"
#include "math/math_utils.hpp"

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

struct TerminalRawGuard {
    bool active{false};
    struct termios orig{};
    TerminalRawGuard() {
        if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &orig) == 0) {
            struct termios raw = orig;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
                active = true;
            }
        }
    }
    ~TerminalRawGuard() {
        if (active) tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    }
};

static char read_key_nonblocking() {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    struct timeval tv{0, 0};
    if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) > 0) {
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) == 1) {
            return ch;
        }
    }
    return 0;
}

enum class FlightMode { STANDBY, FLYING, LANDING };

static int open_serial_port(const std::string& port_name, int baud_rate = B921600) {
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    // Giải phóng RTS/DTR để ESP32 thoát khỏi Reset/Bootloader an toàn (khớp 06_hitl_esp32_bridge.cpp)
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
        status |= TIOCM_RTS;
        status &= ~TIOCM_DTR;
        ioctl(fd, TIOCMSET, &status);
        usleep(50000);

        status &= ~TIOCM_RTS;
        status &= ~TIOCM_DTR;
        ioctl(fd, TIOCMSET, &status);
        usleep(250000);
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static std::string auto_detect_serial_port() {
    const std::vector<std::string> candidates = {
        "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"
    };
    for (const auto& p : candidates) {
        int fd = open(p.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd >= 0) { close(fd); return p; }
    }
    return "";
}

static ControlAlgoMode parse_algo_arg(const std::string& val) {
    if (val == "2" || val == "rl" || val == "RL" || val == "critic") return ALGO_RL_CRITIC;
    if (val == "3" || val == "pid" || val == "PID") return ALGO_PID;
    return ALGO_ITSMC;
}

struct DroneSlot {
    std::mutex mtx;
    DroneEstimation::StateEstimator estimator;
    DroneFormation::DroneState state;
    bool connected{false};
};

static std::array<DroneSlot, 4> g_drones;
static bool g_use_estimator = true;

static void sync_from_estimator(int idx) {
    auto st = g_drones[idx].estimator.get_state();
    std::lock_guard<std::mutex> lk(g_drones[idx].mtx);
    g_drones[idx].state.x     = st.x;
    g_drones[idx].state.y     = st.y;
    g_drones[idx].state.z     = st.z;
    g_drones[idx].state.vx_w  = st.vx_world;
    g_drones[idx].state.vy_w  = st.vy_world;
    g_drones[idx].state.vz_w  = st.vz_world;
    g_drones[idx].state.roll  = st.roll;
    g_drones[idx].state.pitch = st.pitch;
    g_drones[idx].state.yaw   = st.yaw;
    g_drones[idx].state.p     = st.p;
    g_drones[idx].state.q     = st.q;
    g_drones[idx].state.r     = st.r;
    if (st.is_healthy()) g_drones[idx].connected = true;
}

static void on_world_odom(int idx, const gz::msgs::Odometry& msg) {
    float x = static_cast<float>(msg.pose().position().x());
    float y = static_cast<float>(msg.pose().position().y());
    float z = static_cast<float>(msg.pose().position().z());
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    DroneMath::quaternion_to_euler(
        msg.pose().orientation().w(), msg.pose().orientation().x(),
        msg.pose().orientation().y(), msg.pose().orientation().z(),
        roll, pitch, yaw);

    float vx_b = static_cast<float>(msg.twist().linear().x());
    float vy_b = static_cast<float>(msg.twist().linear().y());
    float vz_b = static_cast<float>(msg.twist().linear().z());
    float cr = std::cos(roll), sr = std::sin(roll);
    float cp = std::cos(pitch), sp = std::sin(pitch);
    float cy = std::cos(yaw), sy = std::sin(yaw);

    float vx_w = (cy * cp) * vx_b + (cy * sp * sr - sy * cr) * vy_b + (cy * sp * cr + sy * sr) * vz_b;
    float vy_w = (sy * cp) * vx_b + (sy * sp * sr + cy * cr) * vy_b + (sy * sp * cr - cy * sr) * vz_b;
    float vz_w = (-sp) * vx_b + (cp * sr) * vy_b + (cp * cr) * vz_b;

    std::lock_guard<std::mutex> lk(g_drones[idx].mtx);
    g_drones[idx].state.x = x;
    g_drones[idx].state.y = y;
    g_drones[idx].state.z = z;
    g_drones[idx].state.vx_w = vx_w;
    g_drones[idx].state.vy_w = vy_w;
    g_drones[idx].state.vz_w = vz_w;
    g_drones[idx].state.roll = roll;
    g_drones[idx].state.pitch = pitch;
    g_drones[idx].state.yaw = yaw;
    g_drones[idx].state.p = static_cast<float>(msg.twist().angular().x());
    g_drones[idx].state.q = static_cast<float>(msg.twist().angular().y());
    g_drones[idx].state.r = static_cast<float>(msg.twist().angular().z());
    g_drones[idx].connected = true;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);
    TerminalRawGuard term_guard;

    ControlAlgoMode selected_algo = ALGO_ITSMC;
    std::string serial_port;
    bool use_esp32 = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--world-odom" || arg == "--no-estimator") {
            g_use_estimator = false;
        } else if (arg == "--no-esp32") {
            use_esp32 = false;
        } else if (arg == "--algo" && i + 1 < argc) {
            selected_algo = parse_algo_arg(argv[++i]);
        } else if (arg.rfind("--algo=", 0) == 0) {
            selected_algo = parse_algo_arg(arg.substr(7));
        } else if (arg.rfind("/dev/", 0) == 0) {
            serial_port = arg;
        }
    }

    int serial_fd = -1;
    if (use_esp32) {
        if (serial_port.empty()) serial_port = auto_detect_serial_port();
        if (!serial_port.empty()) {
            serial_fd = open_serial_port(serial_port, B921600);
        }
        if (serial_fd >= 0) {
            std::cout << "[✓] Drone 1 (x500_1) chạy HITL trên ESP32 qua " << serial_port << " @ 921600 baud" << std::endl;
        } else {
            std::cout << "[!] Không tìm thấy ESP32 Serial -> Chuyển toàn bộ 4 Drone sang chạy SMC+HGO trên PC!" << std::endl;
            use_esp32 = false;
        }
    } else {
        std::cout << "[★] Chạy mô phỏng đội hình 4 Drone thuần C++ (--no-esp32)" << std::endl;
    }

    std::cout << "[★] Nguồn trạng thái: "
              << (g_use_estimator ? "4 Cảm biến thô + 4x StateEstimator (Kalman/Complementary)"
                                  : "Đọc thẳng Gazebo Odometry (/model/x500_k/odometry)")
              << std::endl;

    gz::transport::Node gz_node;
    std::array<gz::transport::Node::Publisher, 4> motor_pubs;
    std::array<gz::transport::Node::Publisher, 4> motor_pubs_alt;
    for (int k = 0; k < 4; ++k) {
        std::string model_name = "x500_" + std::to_string(k + 1);
        motor_pubs[k]     = gz_node.Advertise<gz::msgs::Actuators>("/" + model_name + "/command/motor_speed");
        motor_pubs_alt[k] = gz_node.Advertise<gz::msgs::Actuators>("/model/" + model_name + "/command/motor_speed");

        if (g_use_estimator) {
            g_drones[k].estimator.set_gps_origin(47.397971057728974, 8.546163739800146, 0.0);
            g_drones[k].estimator.set_mag_reference(6e-6f, 2.3e-5f, -4.2e-5f);

            std::string prefix = "/world/default/model/" + model_name + "/link/base_link/sensor/";
            gz_node.Subscribe(prefix + "imu_sensor/imu", std::function<void(const gz::msgs::IMU&)>(
                [k](const gz::msgs::IMU& msg) {
                    double ts = msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9;
                    g_drones[k].estimator.on_imu(ts,
                        msg.angular_velocity().x(), msg.angular_velocity().y(), msg.angular_velocity().z(),
                        msg.linear_acceleration().x(), msg.linear_acceleration().y(), msg.linear_acceleration().z());
                    sync_from_estimator(k);
                }));
            gz_node.Subscribe(prefix + "navsat_sensor/navsat", std::function<void(const gz::msgs::NavSat&)>(
                [k](const gz::msgs::NavSat& msg) {
                    g_drones[k].estimator.on_gps(msg.latitude_deg(), msg.longitude_deg(), msg.altitude(),
                        msg.velocity_east(), msg.velocity_north(), msg.velocity_up());
                    sync_from_estimator(k);
                }));
            gz_node.Subscribe(prefix + "air_pressure_sensor/air_pressure", std::function<void(const gz::msgs::FluidPressure&)>(
                [k](const gz::msgs::FluidPressure& msg) {
                    g_drones[k].estimator.on_baro(static_cast<float>(msg.pressure()));
                    sync_from_estimator(k);
                }));
            gz_node.Subscribe(prefix + "magnetometer_sensor/magnetometer", std::function<void(const gz::msgs::Magnetometer&)>(
                [k](const gz::msgs::Magnetometer& msg) {
                    g_drones[k].estimator.on_mag(msg.field_tesla().x(), msg.field_tesla().y(), msg.field_tesla().z());
                }));
        } else {
            gz_node.Subscribe("/model/" + model_name + "/odometry", std::function<void(const gz::msgs::Odometry&)>(
                [k](const gz::msgs::Odometry& msg) { on_world_odom(k, msg); }));
        }
    }

    std::cout << "[*] Đang chờ kết nối 4 Drone từ Gazebo..." << std::endl;
    while (g_running) {
        bool all_ok = true;
        for (int k = 0; k < 4; ++k) {
            std::lock_guard<std::mutex> lk(g_drones[k].mtx);
            if (!g_drones[k].connected) all_ok = false;
        }
        if (all_ok) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!g_running) return 0;
    std::cout << "[✓] Đã nhận đủ trạng thái 4 Drone từ Gazebo!" << std::endl;

    // MAVLink cho Drone 1 -> QGC
    DroneCore::VehicleState vstate1;
    DroneComm::MavlinkBridge mavlink_bridge;
    mavlink_bridge.init(&vstate1, "", 14550, 14540);

    // UDP Web Oscilloscope (9098)
    int osc_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in osc_addr{};
    osc_addr.sin_family = AF_INET;
    osc_addr.sin_port = htons(9098);
    inet_pton(AF_INET, "127.0.0.1", &osc_addr.sin_addr);

    // Log Formation CSV
    std::filesystem::create_directories("logs/formation/trajectory");
    std::filesystem::create_directories("DroneHITL/logs/formation/trajectory");
    std::string log_path = "logs/formation/trajectory/latest_formation.csv";
    std::ofstream log_file(log_path, std::ios::out | std::ios::trunc);
    log_file << "time_s,p0_x,p0_y,p0_z,"
             << "x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,"
             << "pd1_x,pd1_y,pd1_z,pd2_x,pd2_y,pd2_z,pd3_x,pd3_y,pd3_z,pd4_x,pd4_y,pd4_z\n";

    DroneFormation::DistributedFormationCoordinator formation;
    std::vector<uint8_t> rx_buf;
    std::array<float, 4> esp_motors{0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t esp_rx_count = 0;
    float last_esp_r = 0.0f, last_esp_p = 0.0f;
    float esp_virtual_alt = 0.12f;
    bool esp_virtual_alt_init = false;
    bool fallback_warned = false;
    FlightMode mode = FlightMode::STANDBY;

    if (use_esp32 && serial_fd >= 0) {
        std::cout << "[*] Đang chờ ESP32 khởi động xong sau khi mở cổng Serial (1.5s)..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        tcflush(serial_fd, TCIOFLUSH);
    }

    std::cout << "\n[⏸ STANDBY] Cả 4 Drone đang nằm yên trên bãi đáp. Nhấn phím [1] để cất cánh đội hình Kim cương | [0] Hạ cánh | [Ctrl+C] Thoát.\n"
              << std::endl;

    auto t_start = std::chrono::steady_clock::now();
    auto last_loop_time = t_start;
    auto last_hud = t_start;
    auto last_osc = t_start;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        float real_dt = std::clamp(std::chrono::duration<float>(now - last_loop_time).count(), 0.002f, 0.05f);
        last_loop_time = now;

        std::array<DroneFormation::DroneState, 4> cur_st{};
        for (int k = 0; k < 4; ++k) {
            std::lock_guard<std::mutex> lk(g_drones[k].mtx);
            cur_st[k] = g_drones[k].state;
        }

        // Đọc phím điều khiển không chặn (hỗ trợ nhấn thẳng '1' hoặc '1' + Enter)
        char key = read_key_nonblocking();
        if (key == '1' && mode != FlightMode::FLYING) {
            formation.sync_start_positions(cur_st);
            esp_virtual_alt = cur_st[0].z;
            esp_virtual_alt_init = false;
            fallback_warned = false;
            mode = FlightMode::FLYING;
            std::cout << "\n[🚀 TAKEOFF] Đã nhận phím 1 -> Cả 4 Drone cất cánh xếp đội hình Kim cương!" << std::endl;
        } else if (key == '0' && mode == FlightMode::FLYING) {
            mode = FlightMode::LANDING;
            std::cout << "\n[🛬 LAND] Đã nhận phím 0 -> Cả 4 Drone hạ cánh an toàn!" << std::endl;
        }

        if (mode == FlightMode::FLYING) {
            formation.step_consensus(0.010f);
        } else if (mode == FlightMode::LANDING) {
            formation.step_landing(0.010f);
        }

        // Cập nhật VehicleState Drone 1 cho QGC
        {
            float cy = std::cos(cur_st[0].yaw), sy = std::sin(cur_st[0].yaw);
            vstate1.pos_x.store(cur_st[0].x);
            vstate1.pos_y.store(cur_st[0].y);
            vstate1.altitude.store(cur_st[0].z);
            vstate1.vx.store(cy * cur_st[0].vx_w + sy * cur_st[0].vy_w);
            vstate1.vy.store(-sy * cur_st[0].vx_w + cy * cur_st[0].vy_w);
            vstate1.vz.store(cur_st[0].vz_w);
            vstate1.roll.store(cur_st[0].roll);
            vstate1.pitch.store(cur_st[0].pitch);
            vstate1.yaw.store(cur_st[0].yaw);
            vstate1.p.store(cur_st[0].p);
            vstate1.q.store(cur_st[0].q);
            vstate1.r.store(cur_st[0].r);
            vstate1.is_armed.store(mode != FlightMode::STANDBY);
            vstate1.is_connected.store(true);
        }

        // Drone 1: ESP32 HITL hoặc SMC+HGO
        std::array<float, 4> m1{0.0f, 0.0f, 0.0f, 0.0f};
        if (use_esp32 && serial_fd >= 0) {
            SensorPacket pkt{};
            pkt.header[0] = 0xAA;
            pkt.header[1] = 0x55;
            pkt.algo_mode = static_cast<uint8_t>(selected_algo);
            pkt.x = cur_st[0].x; pkt.y = cur_st[0].y; pkt.z = cur_st[0].z;
            float cy = std::cos(cur_st[0].yaw), sy = std::sin(cur_st[0].yaw);
            pkt.vx =  cy * cur_st[0].vx_w + sy * cur_st[0].vy_w;
            pkt.vy = -sy * cur_st[0].vx_w + cy * cur_st[0].vy_w;
            pkt.vz = cur_st[0].vz_w;
            pkt.roll = cur_st[0].roll; pkt.pitch = cur_st[0].pitch; pkt.yaw = cur_st[0].yaw;
            pkt.p = cur_st[0].p; pkt.q = cur_st[0].q; pkt.r = cur_st[0].r;
            pkt.cmd_yaw_rate = 0.0f;

            float sim_t = formation.get_sim_time();
            const auto& pd0 = formation.get_p_des(0);
            const auto& dpd0 = formation.get_dp_des(0);

            if (mode == FlightMode::STANDBY) {
                pkt.flags = 0;
                pkt.cmd_vx = 0.0f;
                pkt.cmd_vy = 0.0f;
                pkt.cmd_alt_vel = 0.0f;
                esp_virtual_alt = cur_st[0].z;
                esp_virtual_alt_init = false;
            } else if (mode == FlightMode::LANDING) {
                pkt.flags = CTRL_FLAG_ARMED | CTRL_FLAG_LAND;
                pkt.cmd_vx = 0.0f;
                pkt.cmd_vy = 0.0f;
                pkt.cmd_alt_vel = -0.60f;
            } else {
                float raw_vz = 0.0f;
                formation.compute_esp32_velocity_cmds(cur_st[0], pkt.cmd_vx, pkt.cmd_vy, raw_vz);

                if (sim_t < 0.25f) {
                    pkt.flags = 0; // Đảm bảo cạnh lên DISARMED -> ARMED sạch trên ESP32
                    pkt.cmd_vx = 0.0f;
                    pkt.cmd_vy = 0.0f;
                    pkt.cmd_alt_vel = 0.0f;
                    esp_virtual_alt = cur_st[0].z;
                    esp_virtual_alt_init = false;
                } else if (sim_t < 0.60f) {
                    // Kích hoạt chuyển trạng thái DISARMED -> GROUND_IDLE -> FLYING trên ESP32
                    pkt.flags = CTRL_FLAG_ARMED | CTRL_FLAG_ALT_ACTIVE | CTRL_FLAG_TAKEOFF;
                    pkt.cmd_vx = 0.0f;
                    pkt.cmd_vy = 0.0f;
                    pkt.cmd_alt_vel = 0.80f;
                    if (!esp_virtual_alt_init) {
                        esp_virtual_alt = std::max(cur_st[0].z + 0.10f, 0.25f);
                        esp_virtual_alt_init = true;
                    } else {
                        esp_virtual_alt = std::min(2.0f, esp_virtual_alt + 0.80f * real_dt);
                    }
                } else {
                    pkt.flags = CTRL_FLAG_ARMED | CTRL_FLAG_ALT_ACTIVE;
                    float cmd_vz_ff = DroneMath::clamp(dpd0.z + 1.8f * (pd0.z - esp_virtual_alt), -1.8f, 1.8f);
                    esp_virtual_alt = DroneMath::clamp(esp_virtual_alt + cmd_vz_ff * real_dt, 0.05f, 10.0f);
                    float alt_trim = DroneMath::clamp(0.18f * (pd0.z - cur_st[0].z) - 0.12f * cur_st[0].vz_w, -0.25f, 0.25f);
                    pkt.cmd_alt_vel = DroneMath::clamp(cmd_vz_ff + alt_trim, -2.0f, 2.0f);

                    if (sim_t >= 4.0f && cur_st[0].z >= 1.0f) {
                        pkt.flags |= CTRL_FLAG_DRIVING_X | CTRL_FLAG_DRIVING_Y;
                    } else {
                        pkt.cmd_vx = 0.0f;
                        pkt.cmd_vy = 0.0f;
                    }
                }
            }

            pkt.checksum = compute_packet_checksum(reinterpret_cast<const uint8_t*>(&pkt) + 2, sizeof(SensorPacket) - 3);
            (void)write(serial_fd, &pkt, sizeof(SensorPacket));

            uint8_t tmp[256];
            ssize_t nr = 0;
            while ((nr = read(serial_fd, tmp, sizeof(tmp))) > 0) {
                rx_buf.insert(rx_buf.end(), tmp, tmp + nr);
            }
            while (rx_buf.size() >= sizeof(ActuatorPacket)) {
                if (rx_buf[0] == 0x55 && rx_buf[1] == 0xAA) {
                    ActuatorPacket ack{};
                    std::memcpy(&ack, rx_buf.data(), sizeof(ActuatorPacket));
                    if (compute_packet_checksum(reinterpret_cast<const uint8_t*>(&ack) + 2, sizeof(ActuatorPacket) - 3) == ack.checksum) {
                        esp_motors = {ack.w0, ack.w1, ack.w2, ack.w3};
                        last_esp_r = ack.target_roll_deg;
                        last_esp_p = ack.target_pitch_deg;
                        esp_rx_count++;
                        rx_buf.erase(rx_buf.begin(), rx_buf.begin() + sizeof(ActuatorPacket));
                    } else {
                        rx_buf.erase(rx_buf.begin());
                    }
                } else {
                    rx_buf.erase(rx_buf.begin());
                }
            }

            if (mode == FlightMode::STANDBY) {
                m1 = {0.0f, 0.0f, 0.0f, 0.0f};
            } else if (esp_rx_count > 0) {
                m1 = esp_motors;
            } else if (sim_t > 3.0f) {
                if (!fallback_warned) {
                    fallback_warned = true;
                    std::cout << "\n[!] Chưa nhận được phản hồi hợp lệ từ ESP32 sau 3.0s -> Tạm chuyển Drone 1 sang SMC+HGO dự phòng!" << std::endl;
                }
                m1 = formation.compute_drone_motors(0, cur_st[0], 0.010f, true);
            } else {
                m1 = {200.0f, 200.0f, 200.0f, 200.0f};
            }
        } else if (mode != FlightMode::STANDBY) {
            if (mode == FlightMode::LANDING && cur_st[0].z < 0.22f) {
                m1 = {0.0f, 0.0f, 0.0f, 0.0f};
            } else {
                m1 = formation.compute_drone_motors(0, cur_st[0], 0.010f, true);
            }
        }

        auto pub_motors = [&](int idx, const std::array<float, 4>& w) {
            gz::msgs::Actuators msg;
            for (int i = 0; i < 4; ++i) msg.add_velocity(w[i]);
            motor_pubs[idx].Publish(msg);
            motor_pubs_alt[idx].Publish(msg);
        };
        pub_motors(0, m1);
        for (int k = 1; k < 4; ++k) {
            std::array<float, 4> mk{0.0f, 0.0f, 0.0f, 0.0f};
            if (mode == FlightMode::FLYING) {
                mk = formation.compute_drone_motors(k, cur_st[k], 0.010f, true);
            } else if (mode == FlightMode::LANDING && cur_st[k].z >= 0.22f) {
                mk = formation.compute_drone_motors(k, cur_st[k], 0.010f, true);
            }
            pub_motors(k, mk);
        }

        if (mode == FlightMode::LANDING) {
            bool all_down = true;
            for (int k = 0; k < 4; ++k) {
                if (cur_st[k].z >= 0.25f) {
                    all_down = false;
                    break;
                }
            }
            if (all_down) {
                mode = FlightMode::STANDBY;
                std::cout << "\n[✓] Cả 4 Drone đã hạ cánh an toàn (STANDBY). Nhấn [1] để bay lại." << std::endl;
            }
        }

        // Ghi log CSV 4 Drone + Leader (chỉ ghi khi đang bay / hạ cánh)
        const auto& p0 = formation.get_leader_pos();
        if (mode != FlightMode::STANDBY) {
            log_file << std::fixed << std::setprecision(4)
                     << formation.get_sim_time() << "," << p0.x << "," << p0.y << "," << p0.z;
            for (int k = 0; k < 4; ++k) log_file << "," << cur_st[k].x << "," << cur_st[k].y << "," << cur_st[k].z;
            for (int k = 0; k < 4; ++k) {
                const auto& pd = formation.get_p_des(k);
                log_file << "," << pd.x << "," << pd.y << "," << pd.z;
            }
            log_file << "\n";
        }

        const char* mode_str = (mode == FlightMode::STANDBY) ? "STANDBY"
                             : (mode == FlightMode::LANDING) ? "LANDING"
                                                             : "FLYING";

        // Web Oscilloscope (Drone 1 @ 50Hz)
        if (osc_sock >= 0 && std::chrono::duration<double>(now - last_osc).count() >= 0.020) {
            last_osc = now;
            char jbuf[512];
            int n = snprintf(jbuf, sizeof(jbuf),
                "{\"t\":%.2f,\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"tr\":%.2f,\"tp\":%.2f,"
                "\"z\":%.3f,\"tz\":%.2f,\"vz\":%.2f,\"vx\":%.2f,\"vy\":%.2f,"
                "\"w\":[%.1f,%.1f,%.1f,%.1f],\"arm\":%d,\"land\":%d,\"algo\":\"4-Drone Formation\",\"status\":\"%s\"}",
                formation.get_sim_time(),
                DroneMath::rad2deg(cur_st[0].roll), DroneMath::rad2deg(cur_st[0].pitch), DroneMath::rad2deg(cur_st[0].yaw),
                last_esp_r, last_esp_p, cur_st[0].z, p0.z, cur_st[0].vz_w, cur_st[0].vx_w, cur_st[0].vy_w,
                m1[0], m1[1], m1[2], m1[3],
                (mode != FlightMode::STANDBY) ? 1 : 0,
                (mode == FlightMode::LANDING) ? 1 : 0,
                mode_str);
            if (n > 0) sendto(osc_sock, jbuf, n, 0, (sockaddr*)&osc_addr, sizeof(osc_addr));
        }

        if (std::chrono::duration<double>(now - last_hud).count() >= 0.15) {
            last_hud = now;
            std::cout << "\r[🛸 " << mode_str << " t=" << std::fixed << std::setprecision(1) << formation.get_sim_time()
                      << "s | ESP_Rx=" << esp_rx_count << " w1=" << static_cast<int>(m1[0]) << "] "
                      << "D1(" << cur_st[0].x << "," << cur_st[0].y << "," << cur_st[0].z << ") "
                      << "D2(" << cur_st[1].x << "," << cur_st[1].y << "," << cur_st[1].z << ") "
                      << "D3(" << cur_st[2].x << "," << cur_st[2].y << "," << cur_st[2].z << ") "
                      << "D4(" << cur_st[3].x << "," << cur_st[3].y << "," << cur_st[3].z << ")   " << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    log_file.close();
    try {
        std::filesystem::copy_file(log_path, "DroneHITL/logs/formation/trajectory/latest_formation.csv",
                                   std::filesystem::copy_options::overwrite_existing);
    } catch (...) {}

    mavlink_bridge.shutdown();
    for (int k = 0; k < 4; ++k) {
        gz::msgs::Actuators stop_msg;
        for (int i = 0; i < 4; ++i) stop_msg.add_velocity(0.0f);
        motor_pubs[k].Publish(stop_msg);
        motor_pubs_alt[k].Publish(stop_msg);
    }
    if (osc_sock >= 0) close(osc_sock);
    if (serial_fd >= 0) close(serial_fd);
    std::cout << "\n[✓] Đã lưu log đội hình tại: " << log_path << std::endl;
    return 0;
}
