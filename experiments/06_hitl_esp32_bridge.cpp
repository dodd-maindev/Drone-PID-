/**
 * @file 06_hitl_esp32_bridge.cpp
 * @brief Chương trình Cầu nối Phần cứng trong vòng lặp (Hardware-In-The-Loop - HITL Gateway)
 * 
 * Vai trò:
 * 1. Chạy trên máy tính (Linux / WSL2): Nhận lệnh lái từ Tay cầm Xbox 360 / Bàn phím và dữ liệu Odometry từ Gazebo Sim.
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/actuators.pb.h>

#include "comm/telemetry_packet.hpp"
#include "math/math_utils.hpp"

// Cấu trúc gói tin UDP truyền từ Tay cầm Xbox 360
#pragma pack(push, 1)
struct XboxUdpPacket {
    uint32_t magic;          // 0x58424F58 ('XBOX')
    uint16_t buttons;        // wButtons bitmask
    uint8_t left_trigger;    // bLeftTrigger
    uint8_t right_trigger;   // bRightTrigger
    int16_t thumb_lx;        // Con lăn trái trục ngang
    int16_t thumb_ly;        // Con lăn trái trục đứng (Độ cao)
    int16_t thumb_rx;        // Con lăn phải trục ngang (Trái / Phải)
    int16_t thumb_ry;        // Con lăn phải trục đứng (Tiến / Lùi)
    uint8_t is_connected;    // 1: Đã kết nối, 0: Ngắt kết nối
};
#pragma pack(pop)

// Mã phím chuẩn XInput Xbox 360
constexpr uint16_t XBOX_BTN_DPAD_UP    = 0x0001;
constexpr uint16_t XBOX_BTN_DPAD_DOWN  = 0x0002;
constexpr uint16_t XBOX_BTN_DPAD_LEFT  = 0x0004;
constexpr uint16_t XBOX_BTN_DPAD_RIGHT = 0x0008;
constexpr uint16_t XBOX_BTN_START      = 0x0010;
constexpr uint16_t XBOX_BTN_BACK       = 0x0020;
constexpr uint16_t XBOX_BTN_LTHUMB     = 0x0040;
constexpr uint16_t XBOX_BTN_RTHUMB     = 0x0080;
constexpr uint16_t XBOX_BTN_LB         = 0x0100;
constexpr uint16_t XBOX_BTN_RB         = 0x0200;
constexpr uint16_t XBOX_BTN_A          = 0x1000;
constexpr uint16_t XBOX_BTN_B          = 0x2000;
constexpr uint16_t XBOX_BTN_X          = 0x4000;
constexpr uint16_t XBOX_BTN_Y          = 0x8000;

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

// Mở UDP socket non-blocking nhận dữ liệu từ Tay cầm
int create_udp_receiver_socket(int port = 9099) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;

    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }
    return sock;
}

// Lấy IP của WSL2
std::string get_wsl_ip() {
    FILE* fp = popen("hostname -I 2>/dev/null", "r");
    if (!fp) return "127.0.0.1";
    char buf[128];
    std::string ip = "127.0.0.1";
    if (fgets(buf, sizeof(buf), fp)) {
        char* token = strtok(buf, " \t\r\n");
        if (token) ip = token;
    }
    pclose(fp);
    return ip;
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
    std::string serial_port = "";
    if (argc > 1) {
        serial_port = argv[1];
    } else {
        // Tự động quét tìm cổng ESP32 đang hoạt động
        const std::vector<std::string> candidates = {"/dev/ttyUSB1", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyACM1"};
        for (const auto& dev : candidates) {
            int test_fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (test_fd >= 0) {
                close(test_fd);
                serial_port = dev;
                break;
            }
        }
        if (serial_port.empty()) {
            serial_port = "/dev/ttyUSB0";
        }
    }

    std::cout << "============================================================" << std::endl;
    std::cout << " CẦU NỐI HARDWARE-IN-THE-LOOP (HITL) ESP32 <-> GAZEBO SIM" << std::endl;
    std::cout << " Điều khiển Quadrotor x500 bằng Lõi Firmware trên ESP32" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << " Cổng Serial: " << serial_port << " (Tốc độ: 921600 baud)" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << " [🎮 HƯỚNG DẪN ĐIỀU KHIỂN TAY CẦM XBOX 360]:" << std::endl;
    std::cout << "   • [Nút Y]           : CẤT CÁNH lên 2.0m (Takeoff)" << std::endl;
    std::cout << "   • [Nút A]           : HẠ CÁNH an toàn (Safe Land)" << std::endl;
    std::cout << "   • [Con lăn trái ↑↓] : ĐIỀU KHIỂN ĐỘ CAO (Bay lên / Hạ xuống)" << std::endl;
    std::cout << "   • [Con lăn phải ↑↓] : TIẾN / LÙI (Pitch)" << std::endl;
    std::cout << "   • [Con lăn phải ←→] : TRÁI / PHẢI (Roll)" << std::endl;
    std::cout << "   • [Nút X]           : XOAY QUANH TRỤC SANG TRÁI (Yaw Left)" << std::endl;
    std::cout << "   • [Nút B]           : XOAY QUANH TRỤC SANG PHẢI (Yaw Right)" << std::endl;
    std::cout << "   • [Nút BACK]        : Phanh dừng khẩn cấp / DISARM" << std::endl;
    std::cout << "   • [Nút LB / RB]     : Giảm / Tăng tốc độ tối đa (1.0 - 5.0 m/s)" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << " [⌨️ BÀN PHÍM DỰ PHÒNG]:" << std::endl;
    std::cout << "   • [Q] Cất cánh | [A] Hạ cánh | [W / S] Độ cao | [Mũi tên] Lái" << std::endl;
    std::cout << "   • [Z / C] Xoay Yaw | [SPACE] Phanh khẩn | [X] Thoát" << std::endl;
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

    // 2. Khởi tạo UDP Socket và Cầu nối Tay Cầm Xbox 360
    int udp_sock = create_udp_receiver_socket(9099);
    std::string wsl_ip = get_wsl_ip();
    if (udp_sock >= 0) {
        std::cout << "[✓] Đã mở cổng UDP 9099 nhận tín hiệu tay cầm (IP WSL: " << wsl_ip << ")" << std::endl;
    } else {
        std::cerr << "[!] Cảnh báo: Không thể mở socket UDP 9099 cho tay cầm!" << std::endl;
    }

    // Tự động khởi chạy cầu nối Xbox Windows bridge nếu có
    system("pkill -f xbox_controller_bridge.exe 2>/dev/null");
    std::string bridge_exe = "/home/do/drone_control_cpp/tools/xbox_controller_bridge.exe";
    if (access(bridge_exe.c_str(), X_OK) == 0) {
        std::string launch_cmd = bridge_exe + " " + wsl_ip + " 9099 > /dev/null 2>&1 &";
        system(launch_cmd.c_str());
        std::cout << "[✓] Đã tự động kích hoạt Cầu nối Tay cầm Xbox 360 trên Windows!" << std::endl;
    }

    // 3. Khởi tạo Gazebo Transport IPC
    gz::transport::Node gz_node;
    if (!gz_node.Subscribe("/model/x500/odometry", on_odometry)) {
        std::cerr << "[!] Không thể subscribe topic /model/x500/odometry từ Gazebo!" << std::endl;
        close(serial_fd);
        if (udp_sock >= 0) close(udp_sock);
        system("pkill -f xbox_controller_bridge.exe 2>/dev/null");
        return -1;
    }
    auto motor_pub = gz_node.Advertise<gz::msgs::Actuators>("/x500/command/motor_speed");
    auto motor_pub_alt = gz_node.Advertise<gz::msgs::Actuators>("/model/x500/command/motor_speed");
    if (!motor_pub) {
        std::cerr << "[!] Không thể advertise topic /x500/command/motor_speed!" << std::endl;
        close(serial_fd);
        if (udp_sock >= 0) close(udp_sock);
        system("pkill -f xbox_controller_bridge.exe 2>/dev/null");
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

    // Trạng thái Tay cầm Xbox 360
    XboxUdpPacket latest_xbox_pkt;
    memset(&latest_xbox_pkt, 0, sizeof(latest_xbox_pkt));
    uint16_t last_xbox_buttons = 0;
    bool xbox_connected = false;
    auto last_xbox_pkt_time = std::chrono::steady_clock::now() - std::chrono::seconds(5);

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

        // 1. Đọc dữ liệu từ Tay cầm Xbox 360 qua UDP (nếu có)
        if (udp_sock >= 0) {
            XboxUdpPacket temp_pkt;
            while (recv(udp_sock, &temp_pkt, sizeof(temp_pkt), 0) == sizeof(temp_pkt)) {
                if (temp_pkt.magic == 0x58424F58) {
                    latest_xbox_pkt = temp_pkt;
                    last_xbox_pkt_time = now;
                    xbox_connected = (temp_pkt.is_connected != 0);
                }
            }
        }

        // Kiểm tra timeout kết nối tay cầm (mất tín hiệu > 500ms)
        if (std::chrono::duration<double>(now - last_xbox_pkt_time).count() > 0.5) {
            xbox_connected = false;
        }

        // Xử lý các nút bấm và con lăn trên Tay Cầm Xbox 360
        float gp_vx = 0.0f;
        float gp_vy = 0.0f;
        float gp_alt_vel = 0.0f;
        float gp_yaw_rate = 0.0f;
        bool gp_pitch_active = false;
        bool gp_roll_active = false;
        bool gp_alt_active = false;
        bool gp_yaw_active = false;

        if (xbox_connected) {
            uint16_t btns = latest_xbox_pkt.buttons;

            // Phím Y: Cất cánh
            if ((btns & XBOX_BTN_Y) && !(last_xbox_buttons & XBOX_BTN_Y)) {
                is_armed = true;
                req_takeoff = true;
                req_land = false;
                std::cout << "\n[🚀 TAKEOFF] Nút Y Xbox 360 -> ARM & CẤT CÁNH LÊN 2.0M!" << std::endl;
            }

            // Phím A: Hạ cánh
            if ((btns & XBOX_BTN_A) && !(last_xbox_buttons & XBOX_BTN_A)) {
                req_land = true;
                req_takeoff = false;
                std::cout << "\n[🛑 LAND] Nút A Xbox 360 -> HẠ CÁNH AN TOÀN!" << std::endl;
            }

            // Phím BACK: Phanh khẩn cấp / Disarm
            if ((btns & XBOX_BTN_BACK) && !(last_xbox_buttons & XBOX_BTN_BACK)) {
                is_armed = false;
                req_takeoff = false;
                req_land = false;
                std::cout << "\n[⚠️ EMERGENCY] Nút BACK Xbox 360 -> Phanh khẩn cấp / DISARM!" << std::endl;
            }

            // Phím LB / RB: Giảm / Tăng tốc độ tối đa
            if ((btns & XBOX_BTN_LB) && !(last_xbox_buttons & XBOX_BTN_LB)) {
                max_speed_mps = std::max(1.0f, max_speed_mps - 1.0f);
                std::cout << "\n[⚡ CHẾ ĐỘ] Giảm tốc độ tối đa: " << max_speed_mps << " m/s" << std::endl;
            }
            if ((btns & XBOX_BTN_RB) && !(last_xbox_buttons & XBOX_BTN_RB)) {
                max_speed_mps = std::min(5.0f, max_speed_mps + 1.0f);
                std::cout << "\n[⚡ CHẾ ĐỘ] Tăng tốc độ tối đa: " << max_speed_mps << " m/s" << std::endl;
            }

            // Phím X: Xoay quanh trục sang trái (Yaw Left CCW)
            if (btns & XBOX_BTN_X) {
                gp_yaw_rate = 0.70f;
                gp_yaw_active = true;
            }
            // Phím B: Xoay quanh trục sang phải (Yaw Right CW)
            else if (btns & XBOX_BTN_B) {
                gp_yaw_rate = -0.70f;
                gp_yaw_active = true;
            }

            constexpr int DEADZONE = 4000;

            // Con lăn trái (Left Stick Y): Điều khiển ĐỘ CAO (Bay lên / Hạ xuống)
            if (latest_xbox_pkt.thumb_ly > DEADZONE) {
                gp_alt_vel = static_cast<float>(latest_xbox_pkt.thumb_ly - DEADZONE) / (32767.0f - DEADZONE) * 1.5f;
                gp_alt_active = true;
                req_takeoff = false; // Ngắt cờ cất cánh tự động khi người lái chủ động chỉnh độ cao
            } else if (latest_xbox_pkt.thumb_ly < -DEADZONE) {
                gp_alt_vel = static_cast<float>(latest_xbox_pkt.thumb_ly + DEADZONE) / (32768.0f - DEADZONE) * 1.2f;
                gp_alt_active = true;
                req_takeoff = false; // Ngắt cờ cất cánh tự động
            }

            // Con lăn phải (Right Stick Y): Tiến / Lùi (Pitch)
            if (latest_xbox_pkt.thumb_ry > DEADZONE) {
                gp_vx = static_cast<float>(latest_xbox_pkt.thumb_ry - DEADZONE) / (32767.0f - DEADZONE) * max_speed_mps;
                gp_pitch_active = true;
            } else if (latest_xbox_pkt.thumb_ry < -DEADZONE) {
                gp_vx = static_cast<float>(latest_xbox_pkt.thumb_ry + DEADZONE) / (32768.0f - DEADZONE) * max_speed_mps;
                gp_pitch_active = true;
            }

            // Con lăn phải (Right Stick X): Phải / Trái (Roll)
            // Trong hệ tọa độ thân: Trái là +Vy, Phải là -Vy
            if (latest_xbox_pkt.thumb_rx > DEADZONE) {
                gp_vy = -static_cast<float>(latest_xbox_pkt.thumb_rx - DEADZONE) / (32767.0f - DEADZONE) * max_speed_mps;
                gp_roll_active = true;
            } else if (latest_xbox_pkt.thumb_rx < -DEADZONE) {
                gp_vy = static_cast<float>(-latest_xbox_pkt.thumb_rx - DEADZONE) / (32768.0f - DEADZONE) * max_speed_mps;
                gp_roll_active = true;
            }

            // Tuyệt đối không xoay Yaw bằng con lăn trái. Xoay Yaw chỉ dùng nút X (Trái) và nút B (Phải)

            last_xbox_buttons = btns;
        }

        // Tự động kết thúc cất cánh khi đã lên độ cao ổn định (~1.85m)
        if (req_takeoff && g_pos_z >= 1.85f) {
            req_takeoff = false;
        }

        // 2. Đọc phím điều khiển từ bàn phím (Dự phòng song song)
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
                    req_takeoff = false;
                    break;
                case 's': case 'S':
                    alt_vel_cmd = -0.70f;
                    last_alt_key_time = now;
                    req_takeoff = false;
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

        bool kb_pitch_active = (time_pitch <= pitch_to);
        if (!kb_pitch_active) pitch_hits = 0;

        bool kb_roll_active  = (time_roll  <= roll_to);
        if (!kb_roll_active)  roll_hits = 0;

        bool kb_alt_active = (time_alt <= 0.40);
        bool kb_yaw_active = (time_yaw <= 0.25);

        // Hợp nhất lệnh: Ưu tiên Tay cầm Xbox 360, sau đó đến Bàn phím
        bool pitch_active = gp_pitch_active || kb_pitch_active;
        float send_cmd_vx = gp_pitch_active ? gp_vx : (kb_pitch_active ? target_vx_cmd : 0.0f);

        bool roll_active  = gp_roll_active || kb_roll_active;
        float send_cmd_vy = gp_roll_active ? gp_vy : (kb_roll_active ? target_vy_cmd : 0.0f);

        bool alt_active   = gp_alt_active || kb_alt_active;
        float send_cmd_alt = gp_alt_active ? gp_alt_vel : (kb_alt_active ? alt_vel_cmd : 0.0f);

        bool yaw_active   = gp_yaw_active || kb_yaw_active;
        float send_cmd_yaw = gp_yaw_active ? gp_yaw_rate : (kb_yaw_active ? yaw_rate_cmd : 0.0f);

        // 3. Đóng gói SensorPacket (68 bytes) gửi sang ESP32
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

        send_pkt.cmd_vx = pitch_active ? send_cmd_vx : 0.0f;
        send_pkt.cmd_vy = roll_active  ? send_cmd_vy : 0.0f;
        send_pkt.cmd_alt_vel = alt_active ? send_cmd_alt : 0.0f;
        send_pkt.cmd_yaw_rate = yaw_active ? send_cmd_yaw : 0.0f;

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

        // 4. Đọc phản hồi ActuatorPacket từ ESP32
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

        // 5. In thông tin HUD giám sát (30Hz)
        if (std::chrono::duration<double>(now - last_hud_time).count() >= 0.033) {
            last_hud_time = now;
            std::string status_str;
            if (!is_armed) {
                status_str = "DISARMED (Bấm Y để bay)";
            } else if (req_land) {
                status_str = "HẠ CÁNH...";
            } else {
                status_str = "ĐANG BAY";
            }

            std::string pad_status = xbox_connected ? "🎮 Xbox: ON" : "⌨️ Phím: ON";

            std::cout << "\r[⚡ HITL-ESP32] "
                      << pad_status << " | "
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
    if (udp_sock >= 0) close(udp_sock);
    system("pkill -f xbox_controller_bridge.exe 2>/dev/null");

    std::cout << "\n[!] Đã dừng cầu nối HITL an toàn." << std::endl;
    return 0;
}
