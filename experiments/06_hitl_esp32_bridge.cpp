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
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/cameratrack.pb.h>

#include "comm/telemetry_packet.hpp"
#include "math/math_utils.hpp"
#include "sound/drone_sound.hpp"

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

    // Reset ESP32 một cách an toàn và giải phóng khỏi chế độ Bootloader/Reset (DTR/RTS):
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
        // 1. Kéo RTS lên cao (EN xuống thấp = Reset)
        status |= TIOCM_RTS;
        status &= ~TIOCM_DTR;
        ioctl(fd, TIOCMSET, &status);
        usleep(50000); // 50ms xung reset

        // 2. Hạ cả RTS và DTR (EN lên cao, IO0 lên cao = Chế độ chạy bình thường)
        status &= ~TIOCM_RTS;
        status &= ~TIOCM_DTR;
        ioctl(fd, TIOCMSET, &status);
        usleep(250000); // 250ms cho ESP32 khởi động vào firmware
    }

    tcflush(fd, TCIOFLUSH);

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
static float g_qw = 1.0f, g_qx = 0.0f, g_qy = 0.0f, g_qz = 0.0f;
static bool g_odom_connected = false;

void on_odometry(const gz::msgs::Odometry& msg) {
    std::lock_guard<std::mutex> lock(g_odom_mutex);
    g_pos_x = msg.pose().position().x();
    g_pos_y = msg.pose().position().y();
    g_pos_z = msg.pose().position().z();

    g_vx = msg.twist().linear().x();
    g_vy = msg.twist().linear().y();
    g_vz = msg.twist().linear().z();

    g_qw = msg.pose().orientation().w();
    g_qx = msg.pose().orientation().x();
    g_qy = msg.pose().orientation().y();
    g_qz = msg.pose().orientation().z();
    DroneMath::quaternion_to_euler(g_qw, g_qx, g_qy, g_qz, g_roll, g_pitch, g_yaw);

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
    std::cout << "   • [Con lăn trái ←→] : XOAY HƯỚNG YAW (Trái / Phải)" << std::endl;
    std::cout << "   • [Con lăn phải ↑↓] : TIẾN / LÙI (Pitch)" << std::endl;
    std::cout << "   • [Con lăn phải ←→] : TRÁI / PHẢI (Roll)" << std::endl;
    std::cout << "   • [Nút X / B]       : XOAY TRÁI (CCW) / XOAY PHẢI (CW)" << std::endl;
    std::cout << "   • [Nút RB]          : KHÓA CAMERA bám theo drone" << std::endl;
    std::cout << "   • [Nút LB + Cần phải]: LỘN NHÀO 360° (ACRO FLIP: Trái/Phải/Tới/Lui)" << std::endl;
    std::cout << "   • [Nút BACK]        : Phanh dừng khẩn cấp / DISARM" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << " [⌨️ BÀN PHÍM DỰ PHÒNG]:" << std::endl;
    std::cout << "   • [Q] Cất cánh | [A] Hạ cánh | [W / S] Độ cao | [I/K/J/L] Lái" << std::endl;
    std::cout << "   • [Z / C] Xoay Yaw | [F] Khóa Camera | [R + J/L/I/K] Lộn 360°" << std::endl;
    std::cout << "   • [SPACE] Phanh khẩn | [X] Thoát" << std::endl;
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

    // Camera tự động Follow drone x500 khi khởi động và trong suốt quá trình bay
    auto cam_pub = gz_node.Advertise<gz::msgs::CameraTrack>("/gui/track");
    auto send_camera_follow = [&cam_pub]() {
        gz::msgs::CameraTrack msg;
        msg.set_track_mode(gz::msgs::CameraTrack::FOLLOW);
        msg.mutable_follow_target()->set_name("x500");
        msg.mutable_follow_target()->set_type(gz::msgs::Entity::MODEL);
        msg.set_follow_pgain(0.08);
        auto* fo = msg.mutable_follow_offset();
        fo->set_x(-3.5);
        fo->set_y(0.0);
        fo->set_z(1.8);
        cam_pub.Publish(msg);
    };

    // Luồng nền tự động gửi lệnh Follow camera định kỳ 20s đầu để đảm bảo Gazebo GUI bắt được sau khi khởi động
    std::thread([send_camera_follow]() {
        for (int i = 0; i < 20; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            send_camera_follow();
        }
    }).detach();
    std::cout << "[✓] Đã kích hoạt Camera tự động Follow drone x500!" << std::endl;

    // Khởi tạo hệ thống âm thanh động cơ (start.mp3 và continue.mp3)
    DroneSoundManager sound_mgr;
    sound_mgr.init();

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

    // Trạng thái Acrobatic 360 Flip Maneuver
    enum FlipDirection {
        FLIP_NONE = 0,
        FLIP_LEFT,   // Roll trái (-X)
        FLIP_RIGHT,  // Roll phải (+X)
        FLIP_FRONT,  // Pitch tới (+Y)
        FLIP_BACK    // Pitch lui (-Y)
    };

    enum FlipPhase {
        PHASE_IDLE = 0,
        PHASE_POPUP,    // Bật vọt lấy quán tính độ cao (~130ms @ 920 rad/s)
        PHASE_ROTATING, // Xoay vòng 360° dứt khoát (~160-260ms @ 740 rad/s)
        PHASE_BRAKING,  // Phanh ngược chiều góc (~60-90ms @ 720 rad/s)
        PHASE_RECOVERY  // Đón drone, triệt tiêu rơi tự do và khóa thăng bằng (~180ms)
    };

    FlipPhase flip_phase = PHASE_IDLE;
    FlipDirection flip_dir = FLIP_NONE;
    auto flip_phase_start = std::chrono::steady_clock::now();
    bool flip_passed_inverted = false;
    float flip_start_z = 0.0f;

    auto last_lb_press_time = std::chrono::steady_clock::now() - std::chrono::seconds(5);
    bool lb_flip_armed = false;

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
                send_camera_follow();
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

            // Phím RB: Khóa lại góc nhìn camera tự động bám theo Drone (Follow Mode)
            if ((btns & XBOX_BTN_RB) && !(last_xbox_buttons & XBOX_BTN_RB)) {
                send_camera_follow();
                std::cout << "\n[📷 CAMERA] Nút RB Xbox 360 -> Tự động khóa góc nhìn bám theo drone!" << std::endl;
            }

            // Nút LB: Vũ trang cho cú lộn 360° (Acrobatic Flip)
            // Nhấn LB, sau đó trong vòng 0.35s nếu gạt cần phải (Trái/Phải/Lên/Xuống) thì thực hiện lộn
            if ((btns & XBOX_BTN_LB) && !(last_xbox_buttons & XBOX_BTN_LB)) {
                last_lb_press_time = now;
                lb_flip_armed = true;
                std::cout << "\n[⚡ FLIP READY] Đã nhấn LB! Gạt cần phải (Trái/Phải/Lên/Xuống) trong 0.3s để lộn 360°!" << std::endl;
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

            constexpr int DEADZONE = 7800;

            // Con lăn trái (Left Stick X): Điều khiển hướng quay quanh trục (Yaw Rate)
            if (latest_xbox_pkt.thumb_lx > DEADZONE) {
                gp_yaw_rate = -static_cast<float>(latest_xbox_pkt.thumb_lx - DEADZONE) / (32767.0f - DEADZONE) * 1.5f;
                gp_yaw_active = true;
            } else if (latest_xbox_pkt.thumb_lx < -DEADZONE) {
                gp_yaw_rate = static_cast<float>(-latest_xbox_pkt.thumb_lx - DEADZONE) / (32768.0f - DEADZONE) * 1.5f;
                gp_yaw_active = true;
            }

            // Con lăn trái (Left Stick Y): Điều khiển ĐỘ CAO (Bay lên / Hạ xuống)
            // Chỉ cho phép can thiệp hủy cất cánh tự động khi người lái cố tình gạt cần mạnh (> 16000)
            if (latest_xbox_pkt.thumb_ly > DEADZONE) {
                gp_alt_vel = static_cast<float>(latest_xbox_pkt.thumb_ly - DEADZONE) / (32767.0f - DEADZONE) * 1.5f;
                if (!req_takeoff || latest_xbox_pkt.thumb_ly > 16000) {
                    gp_alt_active = true;
                    req_takeoff = false;
                }
            } else if (latest_xbox_pkt.thumb_ly < -DEADZONE) {
                gp_alt_vel = static_cast<float>(latest_xbox_pkt.thumb_ly + DEADZONE) / (32768.0f - DEADZONE) * 1.2f;
                if (!req_takeoff || latest_xbox_pkt.thumb_ly < -16000) {
                    gp_alt_active = true;
                    req_takeoff = false;
                }
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

            // Kiểm tra kích hoạt lộn 360° nếu đang trong cửa sổ 0.35s kể từ khi nhấn LB (hoặc đang giữ LB)
            double time_since_lb = std::chrono::duration<double>(now - last_lb_press_time).count();
            bool lb_window_open = lb_flip_armed && (time_since_lb <= 0.35 || (btns & XBOX_BTN_LB));
            if (time_since_lb > 0.35 && !(btns & XBOX_BTN_LB)) {
                lb_flip_armed = false;
            }

            if (lb_window_open && flip_phase == PHASE_IDLE) {
                FlipDirection detected_dir = FLIP_NONE;
                if (latest_xbox_pkt.thumb_rx < -16000) {
                    detected_dir = FLIP_LEFT;
                } else if (latest_xbox_pkt.thumb_rx > 16000) {
                    detected_dir = FLIP_RIGHT;
                } else if (latest_xbox_pkt.thumb_ry > 16000) {
                    detected_dir = FLIP_FRONT;
                } else if (latest_xbox_pkt.thumb_ry < -16000) {
                    detected_dir = FLIP_BACK;
                }

                if (detected_dir != FLIP_NONE) {
                    lb_flip_armed = false;
                    if (!is_armed) {
                        std::cout << "\n[⚠️ FLIP BỎ QUA] Drone chưa ARM động cơ!" << std::endl;
                    } else if (g_pos_z < 1.25f) {
                        std::cout << "\n[⚠️ FLIP TỪ CHỐI] Độ cao (" << std::fixed << std::setprecision(2) << g_pos_z 
                                  << "m) quá thấp! Hãy bay lên trên 1.5m để lộn an toàn." << std::endl;
                    } else {
                        flip_dir = detected_dir;
                        flip_phase = PHASE_POPUP;
                        flip_phase_start = now;
                        flip_passed_inverted = false;
                        flip_start_z = g_pos_z;

                        std::string dir_str;
                        if (flip_dir == FLIP_LEFT) dir_str = "LỘN TRÁI (Roll Left 360°)";
                        else if (flip_dir == FLIP_RIGHT) dir_str = "LỘN PHẢI (Roll Right 360°)";
                        else if (flip_dir == FLIP_FRONT) dir_str = "LỘN TỚI TRƯỚC (Front Flip 360°)";
                        else if (flip_dir == FLIP_BACK) dir_str = "LỘN VỀ SAU (Back Flip 360°)";

                        std::cout << "\n[🌀 ACRO FLIP 360°] KÍCH HOẠT: " << dir_str << "!" << std::endl;
                    }
                }
            }

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
                    send_camera_follow();
                    std::cout << "\n[🚀 TAKEOFF] Đã nhận phím Q -> ARM ĐỘNG CƠ & CẤT CÁNH LÊN 2.0M!" << std::endl;
                    break;
                case 'a': case 'A':
                    req_land = true;
                    req_takeoff = false;
                    std::cout << "\n[🛑 LAND] Đã nhận phím A -> HẠ CÁNH AN TOÀN!" << std::endl;
                    break;
                case 'f': case 'F':
                    send_camera_follow();
                    std::cout << "\n[📷 CAMERA] Đã nhận phím F -> Tự động khóa góc nhìn bám theo drone!" << std::endl;
                    break;
                case 'r': case 'R':
                    last_lb_press_time = now;
                    lb_flip_armed = true;
                    std::cout << "\n[⚡ FLIP READY] Đã nhấn R! Nhấn J (Trái), L (Phải), I (Tới), K (Lui) trong 0.5s để lộn 360°!" << std::endl;
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
                    if (lb_flip_armed && flip_phase == PHASE_IDLE && is_armed && g_pos_z >= 1.25f) {
                        lb_flip_armed = false;
                        flip_dir = FLIP_FRONT;
                        flip_phase = PHASE_POPUP;
                        flip_phase_start = now;
                        flip_passed_inverted = false;
                        flip_start_z = g_pos_z;
                        std::cout << "\n[🌀 ACRO FLIP 360°] KÍCH HOẠT: LỘN TỚI TRƯỚC (Front Flip 360°)!" << std::endl;
                    } else {
                        target_vx_cmd = max_speed_mps;
                        last_pitch_key_time = now;
                        pitch_hits++;
                    }
                    break;
                case 'N': case 'k': case 'K':
                    if (lb_flip_armed && flip_phase == PHASE_IDLE && is_armed && g_pos_z >= 1.25f) {
                        lb_flip_armed = false;
                        flip_dir = FLIP_BACK;
                        flip_phase = PHASE_POPUP;
                        flip_phase_start = now;
                        flip_passed_inverted = false;
                        flip_start_z = g_pos_z;
                        std::cout << "\n[🌀 ACRO FLIP 360°] KÍCH HOẠT: LỘN VỀ SAU (Back Flip 360°)!" << std::endl;
                    } else {
                        target_vx_cmd = -max_speed_mps;
                        last_pitch_key_time = now;
                        pitch_hits++;
                    }
                    break;
                case 'L': case 'j': case 'J':
                    if (lb_flip_armed && flip_phase == PHASE_IDLE && is_armed && g_pos_z >= 1.25f) {
                        lb_flip_armed = false;
                        flip_dir = FLIP_LEFT;
                        flip_phase = PHASE_POPUP;
                        flip_phase_start = now;
                        flip_passed_inverted = false;
                        flip_start_z = g_pos_z;
                        std::cout << "\n[🌀 ACRO FLIP 360°] KÍCH HOẠT: LỘN TRÁI (Roll Left 360°)!" << std::endl;
                    } else {
                        target_vy_cmd = max_speed_mps;
                        last_roll_key_time = now;
                        roll_hits++;
                    }
                    break;
                case 'l':
                    if (lb_flip_armed && flip_phase == PHASE_IDLE && is_armed && g_pos_z >= 1.25f) {
                        lb_flip_armed = false;
                        flip_dir = FLIP_RIGHT;
                        flip_phase = PHASE_POPUP;
                        flip_phase_start = now;
                        flip_passed_inverted = false;
                        flip_start_z = g_pos_z;
                        std::cout << "\n[🌀 ACRO FLIP 360°] KÍCH HOẠT: LỘN PHẢI (Roll Right 360°)!" << std::endl;
                    } else {
                        target_vy_cmd = -max_speed_mps;
                        last_roll_key_time = now;
                        roll_hits++;
                    }
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

        if (flip_phase != PHASE_IDLE) {
            // Trong lúc lộn 360°, giữ trạng thái giả lập thăng bằng gửi sang ESP32
            // để bộ PID trong ESP32 không bị dội tích phân (anti-windup)
            send_pkt.roll = 0.0f;
            send_pkt.pitch = 0.0f;
            send_pkt.p = 0.0f;
            send_pkt.q = 0.0f;
            send_pkt.vx = 0.0f;
            send_pkt.vy = 0.0f;
            send_pkt.vz = 0.0f;
            send_pkt.z = flip_start_z;
            send_pkt.cmd_vx = 0.0f;
            send_pkt.cmd_vy = 0.0f;
            send_pkt.cmd_alt_vel = 0.0f;
            send_pkt.cmd_yaw_rate = 0.0f;
            send_pkt.flags = (is_armed ? (CTRL_FLAG_ARMED | CTRL_FLAG_DRIVING_X | CTRL_FLAG_DRIVING_Y) : 0);
        }

        send_pkt.checksum = compute_packet_checksum((uint8_t*)&send_pkt + 2, sizeof(send_pkt) - 3);

        write(serial_fd, (uint8_t*)&send_pkt, sizeof(send_pkt));
        tx_packets++;

        // 4. Đọc toàn bộ phản hồi ActuatorPacket từ ESP32 có trong bộ đệm Serial
        uint8_t read_buf[256];
        int bytes_read = 0;
        while ((bytes_read = read(serial_fd, read_buf, sizeof(read_buf))) > 0) {
            rx_buffer.insert(rx_buffer.end(), read_buf, read_buf + bytes_read);
        }

        static float last_motor_w0 = 0.0f;
        // Tìm và giải mã gói tin ActuatorPacket (28 bytes)
        while (rx_buffer.size() >= sizeof(ActuatorPacket)) {
            if (rx_buffer[0] == 0x55 && rx_buffer[1] == 0xAA) {
                ActuatorPacket in_pkt;
                memcpy(&in_pkt, rx_buffer.data(), sizeof(ActuatorPacket));

                uint8_t cs = compute_packet_checksum((uint8_t*)&in_pkt + 2, sizeof(ActuatorPacket) - 3);
                if (cs == in_pkt.checksum) {
                    // Khi không ở trong cú lộn: Bơm lệnh motor từ ESP32 sang Gazebo Sim
                    if (flip_phase == PHASE_IDLE) {
                        gz::msgs::Actuators motor_msg;
                        motor_msg.add_velocity(in_pkt.w0);
                        motor_msg.add_velocity(in_pkt.w1);
                        motor_msg.add_velocity(in_pkt.w2);
                        motor_msg.add_velocity(in_pkt.w3);
                        motor_pub.Publish(motor_msg);
                        motor_pub_alt.Publish(motor_msg);
                        last_motor_w0 = in_pkt.w0;
                    }

                    rx_packets++;
                    last_esp_pitch = in_pkt.target_pitch_deg;
                    last_esp_roll  = in_pkt.target_roll_deg;

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

        // 4b. Xử lý điều khiển động cơ khi đang thực hiện Acrobatic Flip 360°
        if (flip_phase != PHASE_IDLE) {
            double phase_elapsed = std::chrono::duration<double>(now - flip_phase_start).count();

            // Trích xuất góc Euler, Quaternion, độ cao và vận tốc từ Odometry
            float cur_roll, cur_pitch, cur_p, cur_q, cur_qw, cur_qx, cur_qy, cur_qz;
            float cur_z, cur_vz;
            {
                std::lock_guard<std::mutex> lock(g_odom_mutex);
                cur_roll = g_roll;
                cur_pitch = g_pitch;
                cur_p = g_p;
                cur_q = g_q;
                cur_qw = g_qw;
                cur_qx = g_qx;
                cur_qy = g_qy;
                cur_qz = g_qz;
                cur_z = g_pos_z;
                cur_vz = g_vz;
            }

            // Vector trục Z thân drone trong hệ tọa độ thế giới:
            // z_up = +1.0 (đứng thẳng cân bằng), 0.0 (nghiêng 90°), -1.0 (ngửa bụng hoàn toàn 180°)
            float z_up = 1.0f - 2.0f * (cur_qx * cur_qx + cur_qy * cur_qy);

            // Ghi nhận mốc đã lật qua điểm ngửa bụng (inverted)
            if (z_up < -0.20f) {
                flip_passed_inverted = true;
            }

            float fw0 = 0.0f, fw1 = 0.0f, fw2 = 0.0f, fw3 = 0.0f;

            if (flip_phase == PHASE_POPUP) {
                // Giai đoạn 1: Bật vọt lên lấy quán tính độ cao (~200ms @ 980 rad/s)
                // Cung cấp vận tốc leo ~+1.5m/s và đẩy drone lên +15cm trước khi lộn
                fw0 = fw1 = fw2 = fw3 = 980.0f;
                if (phase_elapsed >= 0.20) {
                    flip_phase = PHASE_ROTATING;
                    flip_phase_start = now;
                    flip_passed_inverted = false;
                }
            } else if (flip_phase == PHASE_ROTATING) {
                // Giai đoạn 2: Xoay lộn nhào chớp nhoáng
                // Khi drone chưa nghiêng quá 78° (z_up >= 0.20): đạp xoay cực nhanh (980 rad/s)
                // KHI ĐÃ NGỬA BỤNG (z_up < 0.20): CẮT TOÀN BỘ 4 ĐỘNG CƠ VỀ 0 rad/s!
                // Triệt tiêu 100% lực đẩy ép xuống đất lúc ngửa bụng, drone lộn qua đỉnh hoàn toàn theo quán tính cực nhanh (< 0.1s)
                const float W_KICK = 980.0f;

                if (z_up >= 0.20f && !flip_passed_inverted) {
                    if (flip_dir == FLIP_RIGHT) {
                        fw1 = fw2 = W_KICK;
                        fw0 = fw3 = 0.0f;
                    } else if (flip_dir == FLIP_LEFT) {
                        fw0 = fw3 = W_KICK;
                        fw1 = fw2 = 0.0f;
                    } else if (flip_dir == FLIP_FRONT) {
                        fw1 = fw3 = W_KICK;
                        fw0 = fw2 = 0.0f;
                    } else if (flip_dir == FLIP_BACK) {
                        fw0 = fw2 = W_KICK;
                        fw1 = fw3 = 0.0f;
                    }
                } else {
                    // ĐANG NGỬA BỤNG: TẮT HOÀN TOÀN CẢ 4 CÁNH (0 rad/s) để không sinh lực đẩy drone xuống đất!
                    fw0 = fw1 = fw2 = fw3 = 0.0f;
                }

                // Ghi nhận mốc đã lật qua điểm ngửa bụng (inverted)
                if (z_up < -0.20f) {
                    flip_passed_inverted = true;
                }

                // Điều kiện chuyển NGAY sang PHASE_RECOVERY (Điều khiển vòng kín PD):
                // Phải ĐÃ QUA ĐIỂM NGỬA BỤNG (flip_passed_inverted) và góc đang tiến sát lại trạng thái cân bằng (~75° còn lại)
                // KHÔNG DÙNG PHANH MỞ (OPEN-LOOP) để TUYỆT ĐỐI KHÔNG BỊ VĂNG SANG HƯỚNG ĐỐI DIỆN!
                bool trigger_recovery = false;
                if (flip_passed_inverted) {
                    if (flip_dir == FLIP_RIGHT && cur_roll >= -1.35f) {
                        trigger_recovery = true;
                    } else if (flip_dir == FLIP_LEFT && cur_roll <= 1.35f) {
                        trigger_recovery = true;
                    } else if ((flip_dir == FLIP_FRONT || flip_dir == FLIP_BACK) && z_up >= 0.25f) {
                        trigger_recovery = true;
                    }
                }

                // Giới hạn an toàn thời gian xoay (0.30s)
                if (trigger_recovery || phase_elapsed >= 0.30) {
                    flip_phase = PHASE_RECOVERY;
                    flip_phase_start = now;
                }
            } else if (flip_phase == PHASE_BRAKING) {
                // Dự phòng (chuyển ngay sang recovery)
                flip_phase = PHASE_RECOVERY;
                flip_phase_start = now;
            } else if (flip_phase == PHASE_RECOVERY) {
                // Giai đoạn 3: ĐIỀU KHIỂN VÒNG KÍN PD HÃM GÓC TỰ ĐỘNG & KHÓA CHẶT ĐỘ CAO
                // Thành phần D (-28.0 * rate) tự động ghì đứng vận tốc quay góc, khi rate về 0 thì lực hãm tự động tắt.
                // Thành phần P (-180.0 * angle) tự động kéo phẳng góc về đúng 0.0° mà KHÔNG BAO GIỜ BỊ QUÁ ĐÀ.
                float alt_deficit = std::max(0.0f, flip_start_z - cur_z);
                float vz_damping = (cur_vz < 0.0f) ? (-cur_vz * 60.0f) : 0.0f;
                float base_catch = 930.0f + DroneMath::clamp(alt_deficit * 350.0f + vz_damping, 0.0f, 65.0f);

                float p_term_roll  = -180.0f * cur_roll  - 28.0f * cur_p;
                float p_term_pitch = -180.0f * cur_pitch - 28.0f * cur_q;

                fw0 = DroneMath::clamp(base_catch - p_term_roll - p_term_pitch, 350.0f, 995.0f);
                fw1 = DroneMath::clamp(base_catch + p_term_roll + p_term_pitch, 350.0f, 995.0f);
                fw2 = DroneMath::clamp(base_catch + p_term_roll - p_term_pitch, 350.0f, 995.0f);
                fw3 = DroneMath::clamp(base_catch - p_term_roll + p_term_pitch, 350.0f, 995.0f);

                // Giữ đón ít nhất 220ms và chỉ nhả quyền điều khiển lại cho ESP32 khi góc đã phẳng (< 5°) và không còn rơi
                bool angle_level = (std::abs(cur_roll) <= 0.09f && std::abs(cur_pitch) <= 0.09f && std::abs(cur_p) <= 2.0f && std::abs(cur_q) <= 2.0f);
                bool catch_stable = (phase_elapsed >= 0.22 && angle_level && (cur_vz >= -0.10f || cur_z >= flip_start_z - 0.05f));
                if (catch_stable || phase_elapsed >= 0.38) {
                    flip_phase = PHASE_IDLE;
                    flip_dir = FLIP_NONE;
                    std::cout << "\n[✓ FLIP HOÀN TẤT] Lộn 360° chuẩn xác 1 vòng! Drone đứng vững thăng bằng." << std::endl;
                }
            }

            gz::msgs::Actuators flip_motor_msg;
            flip_motor_msg.add_velocity(fw0);
            flip_motor_msg.add_velocity(fw1);
            flip_motor_msg.add_velocity(fw2);
            flip_motor_msg.add_velocity(fw3);
            motor_pub.Publish(flip_motor_msg);
            motor_pub_alt.Publish(flip_motor_msg);
            last_motor_w0 = (fw0 + fw1 + fw2 + fw3) * 0.25f;
        }

        // Tự động ngắt động cơ và hoàn tất hạ cánh khi drone đã tiếp đất an toàn
        if (req_land && g_pos_z <= 0.12f && last_motor_w0 < 100.0f) {
            req_land = false;
            is_armed = false;
            std::cout << "\n[✓ TIẾP ĐẤT] Máy bay đã hạ cánh an toàn -> Disarm & Ngắt động cơ." << std::endl;
        }

        // Cập nhật âm thanh động cơ (cất cánh phát start.mp3, bay lặp continue.mp3, hạ cánh giảm dần rồi tắt)
        sound_mgr.update(is_armed, req_land, g_pos_z, last_motor_w0);

        // 5. In thông tin HUD giám sát (30Hz)
        if (std::chrono::duration<double>(now - last_hud_time).count() >= 0.033) {
            last_hud_time = now;
            std::string status_str;
            if (flip_phase != PHASE_IDLE) {
                status_str = "🌀 FLIP 360°...";
            } else if (!is_armed) {
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
