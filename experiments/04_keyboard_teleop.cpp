#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include "core/vehicle_commander.hpp"
#include "controllers/altitude_controller.hpp"
#include "math/math_utils.hpp"

// Cấu hình bàn phím Non-blocking (nhận phím thời gian thực không cần nhấn Enter)
void set_nonblocking_terminal(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // Tắt canonical mode và tắt echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

// Đọc phím bấm phi công
char read_key() {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) > 0) {
        if (c == 27) { // Ký tự escape của phím mũi tên ESC [ A/B/C/D
            char seq[2];
            if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0) {
                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A': return 'U'; // Mũi tên LÊN (Tiến)
                        case 'B': return 'N'; // Mũi tên XUỐNG (Lùi)
                        case 'C': return 'R'; // Mũi tên PHẢI (Nghiêng Phải)
                        case 'D': return 'L'; // Mũi tên TRÁI (Nghiêng Trái)
                    }
                }
            }
        }
        return c;
    }
    return 0;
}

enum class FlightState {
    FLYING,
    LANDING,
    LANDED,
    TAKEOFF
};

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " BÀN PHÍM ĐIỀU KHIỂN TAY CẦM RC ẢO (VIRTUAL RC TELEOP - PRO)" << std::endl;
    std::cout << " (Hạ cánh [A] - Khởi động bay lên tiếp [Q] - Mỏ neo tự động)" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << " [A]            : HẠ CÁNH AN TOÀN (Tự đáp đất & ngắt động cơ)" << std::endl;
    std::cout << " [Q]            : KHỞI ĐỘNG DRONE BAY LÊN TIẾP (Cất cánh lên 2.0m)" << std::endl;
    std::cout << " [Phím Mũi tên] : Lái Tiến / Lùi / Trái / Phải (Nhả ra tự khóa mỏ neo)" << std::endl;
    std::cout << " [W / S]        : Tăng / Giảm độ cao (mỗi nấc 0.15m)" << std::endl;
    std::cout << " [Z / C / E / D]: Xoay đầu (Yaw) Trái / Phải" << std::endl;
    std::cout << " [1 / 2 / 3]    : Đổi chế độ tốc độ: 1-Chậm (4.5°), 2-Vừa (8°), 3-Nhanh (14°)" << std::endl;
    std::cout << " [SPACE]        : Phanh đứng khẩn cấp (Emergency Hover)" << std::endl;
    std::cout << " [X]            : Thoát chương trình" << std::endl;
    std::cout << "============================================================" << std::endl;

    DroneCore::VehicleCommander drone;
    DroneControllers::AltitudeController alt_controller(0.59, 0.35, 0.08, 0.22);

    if (!drone.start()) {
        std::cerr << "[!] Không thể kết nối GZ-Transport!" << std::endl;
        return -1;
    }

    if (!drone.wait_for_connection(15)) {
        std::cerr << "[!] Quá thời gian chờ Gazebo Sim!" << std::endl;
        return -1;
    }

    // 1. Khởi động và Arm động cơ
    std::cout << "\n[*] Bơm tín hiệu mồi 30Hz..." << std::endl;
    for (int i = 0; i < 30; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, drone.state().yaw.load(), 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    drone.arm(true);
    drone.set_offboard_mode();

    // 2. Tự động cất cánh ban đầu lên độ cao 2.0m ổn định
    std::cout << "[🚀] Tự động cất cánh ban đầu lên 2.0m..." << std::endl;
    double target_alt = 2.0;
    float current_target_yaw_rad = drone.state().yaw.load();
    auto takeoff_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - takeoff_start).count() < 4.0) {
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        float thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);
        drone.send_attitude_target(0.0f, 0.0f, current_target_yaw_rad, thrust);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    std::cout << "\n[🎮] SẴN SÀNG LÁI! Bấm mũi tên để bay, nhả ra tự phanh khóa vị trí.\n"
              << "     Ấn [A] để Hạ cánh an toàn | Ấn [Q] để Khởi động bay lên tiếp!\n" << std::endl;

    set_nonblocking_terminal(true);

    FlightState state = FlightState::FLYING;
    float target_roll_deg = 0.0f;
    float target_pitch_deg = 0.0f;
    float max_tilt_deg = 4.5f;
    std::string speed_mode_name = "CHẬM (4.5°)";

    float anchor_x = drone.state().pos_x.load();
    float anchor_y = drone.state().pos_y.load();
    bool anchor_locked = true;

    float land_x = 0.0f;
    float land_y = 0.0f;

    float takeoff_x = 0.0f;
    float takeoff_y = 0.0f;

    auto last_tilt_key_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    bool running = true;

    while (running) {
        char key = read_key();
        auto now = std::chrono::steady_clock::now();

        if (key != 0) {
            switch (key) {
                // [PHÍM A]: HẠ CÁNH AN TOÀN
                case 'a': case 'A':
                    if (state == FlightState::FLYING || state == FlightState::TAKEOFF) {
                        std::cout << "\n\n[🛬] ĐÃ NHẬN PHÍM 'A': KÍCH HOẠT HẠ CÁNH AN TOÀN..." << std::endl;
                        land_x = drone.state().pos_x.load();
                        land_y = drone.state().pos_y.load();
                        target_alt = drone.state().altitude.load();
                        state = FlightState::LANDING;
                    }
                    break;

                // [PHÍM Q]: KHỞI ĐỘNG VÀ BAY LÊN TIẾP
                case 'q': case 'Q':
                    if (state == FlightState::LANDED) {
                        std::cout << "\n\n[🚀] ĐÃ NHẬN PHÍM 'Q': KHỞI ĐỘNG ĐỘNG CƠ VÀ CẤT CÁNH BAY LÊN TIẾP..." << std::endl;
                        drone.arm(true);
                        drone.set_offboard_mode();
                        alt_controller.reset();
                        takeoff_x = drone.state().pos_x.load();
                        takeoff_y = drone.state().pos_y.load();
                        target_alt = 0.15;
                        current_target_yaw_rad = drone.state().yaw.load();
                        state = FlightState::TAKEOFF;
                    } else if (state == FlightState::FLYING) {
                        target_alt = std::max(2.0, target_alt);
                        std::cout << "\n[🚀] PHÍM 'Q': Tự động đưa drone lên độ cao chuẩn " << target_alt << "m!" << std::endl;
                    }
                    break;

                // Điều khiển độ cao (khi đang bay)
                case 'w': case 'W':
                    if (state == FlightState::FLYING) {
                        target_alt = std::min(10.0, target_alt + 0.15);
                    }
                    break;
                case 's': case 'S':
                    if (state == FlightState::FLYING) {
                        target_alt = std::max(0.3, target_alt - 0.15);
                    }
                    break;

                // Điều khiển Tiến / Lùi (Pitch)
                case 'U': // Mũi tên LÊN: Tiến
                    if (state == FlightState::FLYING) {
                        target_pitch_deg = std::min(max_tilt_deg, target_pitch_deg + 1.5f);
                        last_tilt_key_time = now;
                    }
                    break;
                case 'N': // Mũi tên XUỐNG: Lùi
                    if (state == FlightState::FLYING) {
                        target_pitch_deg = std::max(-max_tilt_deg, target_pitch_deg - 1.5f);
                        last_tilt_key_time = now;
                    }
                    break;

                // Điều khiển Trái / Phải (Roll)
                case 'L': // Mũi tên TRÁI: Nghiêng sang trái
                    if (state == FlightState::FLYING) {
                        target_roll_deg = std::max(-max_tilt_deg, target_roll_deg - 1.5f);
                        last_tilt_key_time = now;
                    }
                    break;
                case 'R': // Mũi tên PHẢI: Nghiêng sang phải
                    if (state == FlightState::FLYING) {
                        target_roll_deg = std::min(max_tilt_deg, target_roll_deg + 1.5f);
                        last_tilt_key_time = now;
                    }
                    break;

                // Xoay đầu (Yaw): Z (Trái) / C hoặc E hoặc D (Phải)
                case 'z': case 'Z':
                    current_target_yaw_rad = DroneMath::normalize_angle(current_target_yaw_rad + DroneMath::deg2rad(5.0f));
                    break;
                case 'c': case 'C': case 'e': case 'E': case 'd': case 'D':
                    current_target_yaw_rad = DroneMath::normalize_angle(current_target_yaw_rad - DroneMath::deg2rad(5.0f));
                    break;

                // Đổi chế độ tốc độ
                case '1':
                    max_tilt_deg = 4.5f;
                    speed_mode_name = "CHẬM (4.5°)";
                    break;
                case '2':
                    max_tilt_deg = 8.0f;
                    speed_mode_name = "VỪA (8.0°)";
                    break;
                case '3':
                    max_tilt_deg = 14.0f;
                    speed_mode_name = "NHANH (14.0°)";
                    break;

                // Phím cách: Phanh đứng khẩn cấp
                case ' ':
                    if (state == FlightState::FLYING) {
                        target_roll_deg = 0.0f;
                        target_pitch_deg = 0.0f;
                        target_alt = drone.state().altitude.load();
                        current_target_yaw_rad = drone.state().yaw.load();
                    }
                    break;

                // Thoát khẩn cấp
                case 'x': case 'X':
                    running = false;
                    break;
            }
        }

        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();

        if (state == FlightState::FLYING) {
            // [CHẾ ĐỘ MỎ NEO TỌA ĐỘ ẢO (VIRTUAL ANCHOR)]:
            // Khi người dùng thả tay khỏi phím điều khiển (sau 120ms):
            // 1. Phanh triệt tiêu toàn bộ vận tốc trôi.
            // 2. Ghim cứng tọa độ (anchor_x, anchor_y).
            double time_since_last_tilt = std::chrono::duration<double>(now - last_tilt_key_time).count();
            bool is_hover_mode = (time_since_last_tilt > 0.12);

            if (is_hover_mode) {
                float vx_cur = drone.state().vx.load();
                float vy_cur = drone.state().vy.load();

                if (!anchor_locked) {
                    if (std::abs(vx_cur) < 0.08f && std::abs(vy_cur) < 0.08f) {
                        anchor_x = drone.state().pos_x.load();
                        anchor_y = drone.state().pos_y.load();
                        anchor_locked = true;
                    } else {
                        const float kv_brake = 3.2f;
                        target_pitch_deg = DroneMath::clamp(-kv_brake * vx_cur, -5.0f, 5.0f);
                        target_roll_deg  = DroneMath::clamp( kv_brake * vy_cur, -5.0f, 5.0f);
                    }
                }

                if (anchor_locked) {
                    float err_x = anchor_x - drone.state().pos_x.load();
                    float err_y = anchor_y - drone.state().pos_y.load();

                    target_pitch_deg = DroneMath::clamp(1.2f * err_x - 1.5f * vx_cur, -3.0f, 3.0f);
                    target_roll_deg  = DroneMath::clamp(-1.2f * err_y + 1.5f * vy_cur, -3.0f, 3.0f);
                }
            } else {
                anchor_locked = false;
            }

            float cos_tilt = std::cos(DroneMath::deg2rad(target_roll_deg)) * std::cos(DroneMath::deg2rad(target_pitch_deg));
            if (cos_tilt < 0.7f) cos_tilt = 0.7f;

            float base_thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);
            float total_thrust = base_thrust / cos_tilt;

            drone.send_attitude_target(DroneMath::deg2rad(target_roll_deg),
                                       DroneMath::deg2rad(target_pitch_deg),
                                       current_target_yaw_rad,
                                       total_thrust);

            std::string mode_tag = is_hover_mode ? "⚓ HOVER (BÀI 2)" : ("🎮 LÁI " + speed_mode_name);
            std::cout << "\r[" << std::setw(16) << std::left << mode_tag << "] "
                      << "Alt: " << std::fixed << std::setprecision(2) << current_alt << "m (Đặt: " << target_alt << "m) | "
                      << "Roll: " << std::setw(5) << std::setprecision(1) << target_roll_deg << "° | "
                      << "Pitch: " << std::setw(5) << target_pitch_deg << "° | "
                      << "Yaw: " << std::setw(5) << DroneMath::rad2deg(drone.state().yaw.load()) << "° | "
                      << "Vx: " << std::setw(5) << drone.state().vx.load() << "m/s | "
                      << "Vy: " << std::setw(5) << drone.state().vy.load() << "m/s   " << std::flush;

        } else if (state == FlightState::LANDING) {
            target_alt = std::max(0.05, target_alt - 0.025); // Hạ cánh mượt mà ~0.75 m/s
            float thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);

            // Khóa trục X-Y thẳng tắp trong suốt lúc hạ cánh
            float err_x = land_x - drone.state().pos_x.load();
            float err_y = land_y - drone.state().pos_y.load();
            float vx = drone.state().vx.load();
            float vy = drone.state().vy.load();

            float corr_pitch = DroneMath::clamp(1.2f * err_x - 1.5f * vx, -DroneMath::deg2rad(2.5f), DroneMath::deg2rad(2.5f));
            float corr_roll  = DroneMath::clamp(-1.2f * err_y + 1.5f * vy, -DroneMath::deg2rad(2.5f), DroneMath::deg2rad(2.5f));

            drone.send_attitude_target(corr_roll, corr_pitch, current_target_yaw_rad, thrust);

            std::cout << "\r[🛬 HẠ CÁNH AN TOÀN] Độ cao: " << std::fixed << std::setprecision(2) << current_alt 
                      << "m | Vận tốc rơi: " << vz << "m/s   " << std::flush;

            // Kiểm tra chạm đất
            if (current_alt <= 0.10 && target_alt <= 0.12) {
                for (int i = 0; i < 10; ++i) {
                    drone.send_attitude_target(0.0f, 0.0f, current_target_yaw_rad, 0.0f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                drone.disarm();
                state = FlightState::LANDED;
                std::cout << "\n\n[✓] ĐÃ HẠ CÁNH AN TOÀN VÀ TẮT ĐỘNG CƠ!" << std::endl;
                std::cout << ">>> Nhấn [Q] để khởi động Drone bay lên tiếp | Nhấn [X] để thoát <<<\n" << std::endl;
            }

        } else if (state == FlightState::LANDED) {
            drone.send_attitude_target(0.0f, 0.0f, current_target_yaw_rad, 0.0f);
            std::cout << "\r[💤 ĐÃ TIẾP ĐẤT - ĐỘNG CƠ TẮT] Nhấn 'Q' để khởi động bay lên | Nhấn 'X' để thoát      " << std::flush;

        } else if (state == FlightState::TAKEOFF) {
            target_alt = std::min(2.0, target_alt + 0.03); // Leo lên mượt mà
            float thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);

            // Khóa vị trí cất cánh thẳng đứng
            float err_x = takeoff_x - drone.state().pos_x.load();
            float err_y = takeoff_y - drone.state().pos_y.load();
            float vx = drone.state().vx.load();
            float vy = drone.state().vy.load();

            float corr_pitch = DroneMath::clamp(1.2f * err_x - 1.5f * vx, -DroneMath::deg2rad(2.5f), DroneMath::deg2rad(2.5f));
            float corr_roll  = DroneMath::clamp(-1.2f * err_y + 1.5f * vy, -DroneMath::deg2rad(2.5f), DroneMath::deg2rad(2.5f));

            drone.send_attitude_target(corr_roll, corr_pitch, current_target_yaw_rad, thrust);

            std::cout << "\r[🚀 ĐANG CẤT CÁNH LÊN 2.0M] Độ cao: " << std::fixed << std::setprecision(2) << current_alt 
                      << "m / 2.00m | Vz: " << vz << "m/s   " << std::flush;

            if (target_alt >= 2.0 && current_alt >= 1.90) {
                anchor_x = drone.state().pos_x.load();
                anchor_y = drone.state().pos_y.load();
                anchor_locked = true;
                target_roll_deg = 0.0f;
                target_pitch_deg = 0.0f;
                state = FlightState::FLYING;
                std::cout << "\n\n[✓] ĐÃ LÊN ĐỘ CAO 2.0M ỔN ĐỊNH! Sẵn sàng điều khiển:\n" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    set_nonblocking_terminal(false);

    // Dọn dẹp an toàn khi thoát
    if (state == FlightState::FLYING || state == FlightState::TAKEOFF || state == FlightState::LANDING) {
        std::cout << "\n[!] Đang hạ cánh khẩn cấp trước khi thoát..." << std::endl;
        while (drone.state().altitude.load() > 0.10) {
            target_alt = std::max(0.05, target_alt - 0.03);
            double current_alt = drone.state().altitude.load();
            double vz = drone.state().vz.load();
            float thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);
            drone.send_attitude_target(0.0f, 0.0f, current_target_yaw_rad, thrust);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }

    for (int i = 0; i < 15; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, current_target_yaw_rad, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    drone.disarm();
    drone.stop();
    std::cout << "\n[✓] Đã ngắt động cơ hoàn toàn và thoát chương trình an toàn!" << std::endl;
    return 0;
}
