/**
 * @file 06_hitl_esp32_bridge.cpp
 * @brief Chương trình Cầu nối Phần cứng trong vòng lặp (Hardware-In-The-Loop - HITL Gateway)
 * 
 * Vai trò:
 * 1. Chạy trên máy tính (Linux / WSL2): Nhận phím lái từ người dùng và dữ liệu Odometry từ Gazebo Sim.
 * 2. Đóng gói SensorPacket (68 bytes) gửi sang ESP32 qua cáp USB Serial 921600 baud ở tần số 100-200Hz.
 * 3. Đọc ActuatorPacket (28 bytes) từ ESP32 trả về, bơm vào topic /x500/command/motor_speed của Gazebo Sim.
 * 4. Hiển thị HUD giám sát thời gian thực: Tần số truyền nhận, độ trễ vi điều khiển, góc nghiêng và trạng thái.
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <cstring>
#include <vector>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/actuators.pb.h>

#include "comm/telemetry_packet.hpp"
#include "math/math_utils.hpp"

// Cấu hình bàn phím Non-blocking
void set_nonblocking_terminal(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

// Đọc phím thời gian thực (hỗ trợ phím mũi tên)
char read_key() {
    char latest_key = 0;
    char c = 0;
    while (read(STDIN_FILENO, &c, 1) > 0) {
        if (c == 27) {
            char seq[2] = {0, 0};
            int tries = 0;
            while (tries++ < 20 && read(STDIN_FILENO, &seq[0], 1) <= 0) usleep(100);
            tries = 0;
            while (tries++ < 20 && read(STDIN_FILENO, &seq[1], 1) <= 0) usleep(100);
            if (seq[0] == '[') {
                switch (seq[1]) {
                    case 'A': latest_key = 'U'; break; // Lên
                    case 'B': latest_key = 'N'; break; // Xuống
                    case 'C': latest_key = 'R'; break; // Phải
                    case 'D': latest_key = 'L'; break; // Trái
                    default: break;
                }
            }
        } else {
            latest_key = c;
        }
    }
    return latest_key;
}

// Cấu hình cổng Serial POSIX 921600 baud
int open_serial_port(const std::string& port_name, int baud_rate = B921600) {
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    tty.c_cflag |= (CLOCAL | CREAD); // Cho phép nhận dữ liệu
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;              // 8 data bits
    tty.c_cflag &= ~PARENB;          // No parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;         // No hardware flow control

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0; // Non-blocking read

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    return fd;
}

// Biến lưu trạng thái Odometry từ Gazebo
static std::mutex g_odom_mutex;
static float g_pos_x = 0, g_pos_y = 0, g_pos_z = 0;
static float g_vx = 0, g_vy = 0, g_vz = 0;
static float g_roll = 0, g_pitch = 0, g_yaw = 0;
static float g_p = 0, g_q = 0, g_r = 0;
static bool g_odom_connected = false;

void on_odometry(const gz::msgs::Odometry& msg) {
    std::lock_guard<std::mutex> lock(g_odom_mutex);
    g_pos_x = msg.pose().position().x();
    g_pos_y = msg.pose().position().y();
    g_pos_z = msg.pose().position().z();

    g_vx = msg.twist().linear().x();
    g_vy = msg.twist().linear().y();
    g_vz = msg.twist().linear().z();

    float w = msg.pose().orientation().w();
    float x = msg.pose().orientation().x();
    float y = msg.pose().orientation().y();
    float z = msg.pose().orientation().z();
    DroneMath::quaternion_to_euler(w, x, y, z, g_roll, g_pitch, g_yaw);

    g_p = msg.twist().angular().x();
    g_q = msg.twist().angular().y();
    g_r = msg.twist().angular().z();

    g_odom_connected = true;
}

int main(int argc, char** argv) {
    std::string serial_port = "/dev/ttyUSB0";
    if (argc > 1) {
        serial_port = argv[1];
    }

    std::cout << "============================================================" << std::endl;
    std::cout << " CẦU NỐI HARDWARE-IN-THE-LOOP (HITL) ESP32 <-> GAZEBO SIM" << std::endl;
    std::cout << " Điều khiển Quadrotor x500 bằng Lõi Firmware trên ESP32" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << " Cổng Serial: " << serial_port << " (Tốc độ: 921600 baud)" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << " [A]            : Hạ cánh an toàn (Safe Land)" << std::endl;
    std::cout << " [Q]            : Arm & Cất cánh lên 2.0m (Takeoff to 2.0m)" << std::endl;
    std::cout << " [Phím Mũi tên] : Lái Tiến / Lùi / Trái / Phải (ESP32 tự phanh)" << std::endl;
    std::cout << " [W / S]        : Tăng / Giảm độ cao" << std::endl;
    std::cout << " [Z / C]        : Xoay đầu (Yaw) Trái / Phải" << std::endl;
    std::cout << " [SPACE]        : Phanh khẩn cấp" << std::endl;
    std::cout << " [X]            : Thoát chương trình" << std::endl;
    std::cout << "============================================================\n" << std::endl;

    // 1. Mở cổng Serial kết nối ESP32
    int serial_fd = open_serial_port(serial_port);
    if (serial_fd < 0) {
        std::cerr << "[!] KHÔNG THỂ MỞ CỔNG SERIAL: " << serial_port << std::endl;
        std::cerr << "    Gợi ý:" << std::endl;
        std::cerr << "    - Cắm ESP32 vào cổng USB máy tính." << std::endl;
        std::cerr << "    - Nếu dùng WSL2, hãy dùng 'usbipd' để attach cổng USB vào WSL." << std::endl;
        std::cerr << "    - Kiểm tra quyền truy cập: sudo chmod 666 " << serial_port << std::endl;
        std::cerr << "    - Hoặc chạy với cú pháp: " << argv[0] << " /dev/ttyACM0 (hoặc cổng tương ứng)" << std::endl;
        return -1;
    }
    std::cout << "[✓] Đã mở thành công cổng Serial " << serial_port << " ở tốc độ 921600 baud!" << std::endl;

    // 2. Khởi tạo Gazebo Transport IPC
    gz::transport::Node gz_node;
    if (!gz_node.Subscribe("/model/x500/odometry", on_odometry)) {
        std::cerr << "[!] Không thể subscribe topic /model/x500/odometry từ Gazebo!" << std::endl;
        close(serial_fd);
        return -1;
    }
    auto motor_pub = gz_node.Advertise<gz::msgs::Actuators>("/x500/command/motor_speed");
    auto motor_pub_alt = gz_node.Advertise<gz::msgs::Actuators>("/model/x500/command/motor_speed");
    if (!motor_pub) {
        std::cerr << "[!] Không thể advertise topic /x500/command/motor_speed!" << std::endl;
        close(serial_fd);
        return -1;
    }
    std::cout << "[*] Đang chờ Gazebo Sim xuất tín hiệu Odometry..." << std::endl;
    while (!g_odom_connected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[✓] Đã nhận tín hiệu Odometry 100Hz từ Gazebo Sim!" << std::endl;

    set_nonblocking_terminal(true);

    // Trạng thái lệnh người lái
    bool is_armed = false;
    float max_speed_mps = 2.5f;
    float target_vx_cmd = 0.0f;
    float target_vy_cmd = 0.0f;
    float alt_vel_cmd = 0.0f;
    float yaw_rate_cmd = 0.0f;

    auto last_pitch_key_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto last_roll_key_time  = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto last_alt_key_time   = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto last_yaw_key_time   = std::chrono::steady_clock::now() - std::chrono::seconds(1);

    int pitch_hits = 0;
    int roll_hits = 0;
    bool req_takeoff = false;
    bool req_land = false;
    bool running = true;

    // Bộ đệm nhận UART từ ESP32
    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(256);

    uint32_t tx_packets = 0;
    uint32_t rx_packets = 0;
    float last_esp_pitch = 0.0f;
    float last_esp_roll = 0.0f;

    auto loop_start = std::chrono::steady_clock::now();
    auto last_hud_time = std::chrono::steady_clock::now();

    while (running) {
        auto now = std::chrono::steady_clock::now();

        // 1. Đọc phím điều khiển từ bàn phím
        char key = read_key();
        if (key != 0) {
            switch (key) {
                case 'q': case 'Q':
                    is_armed = true;
                    req_takeoff = true;
                    req_land = false;
                    std::cout << "\n[🚀 TAKEOFF] Đã nhận phím Q -> ARM ĐỘNG CƠ & CẤT CÁNH LÊN 2.0M!" << std::endl;
                    break;
                case 'a': case 'A':
                    req_land = true;
                    req_takeoff = false;
                    std::cout << "\n[🛑 LAND] Đã nhận phím A -> HẠ CÁNH AN TOÀN!" << std::endl;
                    break;
                case 'w': case 'W':
                    alt_vel_cmd = 0.85f;
                    last_alt_key_time = now;
                    break;
                case 's': case 'S':
                    alt_vel_cmd = -0.70f;
                    last_alt_key_time = now;
                    break;
                case 'U': case 'i': case 'I':
                    target_vx_cmd = max_speed_mps;
                    last_pitch_key_time = now;
                    pitch_hits++;
                    break;
                case 'N': case 'k': case 'K':
                    target_vx_cmd = -max_speed_mps;
                    last_pitch_key_time = now;
                    pitch_hits++;
                    break;
                case 'L': case 'j': case 'J':
                    target_vy_cmd = max_speed_mps;
                    last_roll_key_time = now;
                    roll_hits++;
                    break;
                case 'R': case 'l':
                    target_vy_cmd = -max_speed_mps;
                    last_roll_key_time = now;
                    roll_hits++;
                    break;
                case 'z': case 'Z':
                    yaw_rate_cmd = 0.50f;
                    last_yaw_key_time = now;
                    break;
                case 'c': case 'C': case 'e': case 'E': case 'd': case 'D':
                    yaw_rate_cmd = -0.50f;
                    last_yaw_key_time = now;
                    break;
                case '1':
                    max_speed_mps = 1.0f;
                    std::cout << "\n[⚡ CHẾ ĐỘ] 1 - Chậm (1.0 m/s)" << std::endl;
                    break;
                case '2':
                    max_speed_mps = 2.5f;
                    std::cout << "\n[⚡ CHẾ ĐỘ] 2 - Tiêu chuẩn (2.5 m/s)" << std::endl;
                    break;
                case '3':
                    max_speed_mps = 5.0f;
                    std::cout << "\n[⚡ CHẾ ĐỘ] 3 - Tốc độ cao (5.0 m/s)" << std::endl;
                    break;
                case ' ':
                    is_armed = false;
                    req_takeoff = false;
                    req_land = false;
                    std::cout << "\n[⚠️ EMERGENCY] Phanh khẩn cấp / DISARM!" << std::endl;
                    break;
                case 'x': case 'X':
                    running = false;
                    break;
            }
        }

        // Kiểm tra thời gian nhả phím thích ứng (120ms khi đang giữ phím)
        double time_pitch = std::chrono::duration<double>(now - last_pitch_key_time).count();
        double time_roll  = std::chrono::duration<double>(now - last_roll_key_time).count();
        double time_alt   = std::chrono::duration<double>(now - last_alt_key_time).count();
        double time_yaw   = std::chrono::duration<double>(now - last_yaw_key_time).count();

        double pitch_to = (pitch_hits >= 2) ? 0.12 : 0.25;
        double roll_to  = (roll_hits >= 2)  ? 0.12 : 0.25;

        bool pitch_active = (time_pitch <= pitch_to);
        if (!pitch_active) pitch_hits = 0;

        bool roll_active  = (time_roll  <= roll_to);
        if (!roll_active)  roll_hits = 0;

        bool alt_active = (time_alt <= 0.40);
        bool yaw_active = (time_yaw <= 0.25);

        // 2. Đóng gói SensorPacket (68 bytes) gửi sang ESP32
        SensorPacket send_pkt;
        memset(&send_pkt, 0, sizeof(send_pkt));
        send_pkt.header[0] = 0xAA;
        send_pkt.header[1] = 0x55;

        {
            std::lock_guard<std::mutex> lock(g_odom_mutex);
            send_pkt.x = g_pos_x;
            send_pkt.y = g_pos_y;
            send_pkt.z = g_pos_z;
            send_pkt.vx = g_vx;
            send_pkt.vy = g_vy;
            send_pkt.vz = g_vz;
            send_pkt.roll = g_roll;
            send_pkt.pitch = g_pitch;
            send_pkt.yaw = g_yaw;
            send_pkt.p = g_p;
            send_pkt.q = g_q;
            send_pkt.r = g_r;
        }

        send_pkt.cmd_vx = pitch_active ? target_vx_cmd : 0.0f;
        send_pkt.cmd_vy = roll_active  ? target_vy_cmd : 0.0f;
        send_pkt.cmd_alt_vel = alt_active ? alt_vel_cmd : 0.0f;
        send_pkt.cmd_yaw_rate = yaw_active ? yaw_rate_cmd : 0.0f;

        send_pkt.flags = 0;
        if (is_armed) send_pkt.flags |= CTRL_FLAG_ARMED;
        if (pitch_active) send_pkt.flags |= CTRL_FLAG_DRIVING_X;
        if (roll_active)  send_pkt.flags |= CTRL_FLAG_DRIVING_Y;
        if (alt_active)   send_pkt.flags |= CTRL_FLAG_ALT_ACTIVE;
        if (req_takeoff)  send_pkt.flags |= CTRL_FLAG_TAKEOFF;
        if (req_land)     send_pkt.flags |= CTRL_FLAG_LAND;

        send_pkt.checksum = compute_packet_checksum((uint8_t*)&send_pkt + 2, sizeof(send_pkt) - 3);

        write(serial_fd, (uint8_t*)&send_pkt, sizeof(send_pkt));
        tx_packets++;

        // 3. Đọc phản hồi ActuatorPacket từ ESP32
        uint8_t read_buf[64];
        int bytes_read = read(serial_fd, read_buf, sizeof(read_buf));
        static float last_motor_w0 = 0.0f;
        if (bytes_read > 0) {
            rx_buffer.insert(rx_buffer.end(), read_buf, read_buf + bytes_read);

            // Tìm và giải mã gói tin ActuatorPacket (28 bytes)
            while (rx_buffer.size() >= sizeof(ActuatorPacket)) {
                if (rx_buffer[0] == 0x55 && rx_buffer[1] == 0xAA) {
                    ActuatorPacket in_pkt;
                    memcpy(&in_pkt, rx_buffer.data(), sizeof(ActuatorPacket));

                    uint8_t cs = compute_packet_checksum((uint8_t*)&in_pkt + 2, sizeof(ActuatorPacket) - 3);
                    if (cs == in_pkt.checksum) {
                        // Nhận gói tin hợp lệ từ ESP32! Bơm sang Gazebo Sim
                        gz::msgs::Actuators motor_msg;
                        motor_msg.add_velocity(in_pkt.w0);
                        motor_msg.add_velocity(in_pkt.w1);
                        motor_msg.add_velocity(in_pkt.w2);
                        motor_msg.add_velocity(in_pkt.w3);
                        motor_pub.Publish(motor_msg);
                        motor_pub_alt.Publish(motor_msg);

                        rx_packets++;
                        last_esp_pitch = in_pkt.target_pitch_deg;
                        last_esp_roll  = in_pkt.target_roll_deg;
                        last_motor_w0  = in_pkt.w0;

                        // Xóa gói đã xử lý khỏi buffer
                        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sizeof(ActuatorPacket));
                    } else {
                        // Checksum sai, dịch 1 byte để tìm lại đồng bộ
                        rx_buffer.erase(rx_buffer.begin());
                    }
                } else {
                    rx_buffer.erase(rx_buffer.begin());
                }
            }
        }

        // 4. In thông tin HUD giám sát (30Hz)
        if (std::chrono::duration<double>(now - last_hud_time).count() >= 0.033) {
            last_hud_time = now;
            std::string status_str;
            if (!is_armed) {
                status_str = "DISARMED (Bấm Q để bay)";
            } else if (req_land) {
                status_str = "HẠ CÁNH...";
            } else {
                status_str = "ĐANG BAY";
            }

            std::cout << "\r[⚡ HITL-ESP32] "
                      << "Alt: " << std::fixed << std::setprecision(2) << g_pos_z << "m | "
                      << "Roll: " << std::setw(5) << std::setprecision(1) << DroneMath::rad2deg(g_roll) << "° | "
                      << "Pitch: " << std::setw(5) << DroneMath::rad2deg(g_pitch) << "° | "
                      << "Mot: " << std::setw(4) << static_cast<int>(last_motor_w0) << " rad/s | "
                      << "Tx/Rx: " << tx_packets << "/" << rx_packets << " | "
                      << status_str << "   " << std::flush;
        }

        // Tần số gửi 100Hz (10ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    set_nonblocking_terminal(false);

    // Gửi lệnh dừng khẩn cấp trước khi thoát
    gz::msgs::Actuators stop_msg;
    for (int i = 0; i < 4; ++i) stop_msg.add_velocity(0.0f);
    motor_pub.Publish(stop_msg);
    motor_pub_alt.Publish(stop_msg);

    close(serial_fd);
    std::cout << "\n[!] Đã dừng cầu nối HITL an toàn." << std::endl;
    return 0;
}
