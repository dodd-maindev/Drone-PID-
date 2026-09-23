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
#include "controllers/position_controller.hpp"
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

// Đọc phím bấm thời gian thực: quét sạch toàn bộ hàng đợi STDIN và trả về phím mới nhất
char read_key() {
    char latest_key = 0;
    char c = 0;

    // Đọc cạn toàn bộ các ký tự đang tồn đọng trong buffer bàn phím
    while (read(STDIN_FILENO, &c, 1) > 0) {
        if (c == 27) { // Ký tự escape ESC của phím mũi tên ESC [ A/B/C/D
            char seq[2] = {0, 0};
            int tries = 0;
            while (tries++ < 20 && read(STDIN_FILENO, &seq[0], 1) <= 0) {
                usleep(100);
            }
            tries = 0;
            while (tries++ < 20 && read(STDIN_FILENO, &seq[1], 1) <= 0) {
                usleep(100);
            }
            if (seq[0] == '[') {
                switch (seq[1]) {
                    case 'A': latest_key = 'U'; break; // Mũi tên LÊN (Tiến)
                    case 'B': latest_key = 'N'; break; // Mũi tên XUỐNG (Lùi)
                    case 'C': latest_key = 'R'; break; // Mũi tên PHẢI (Phải)
                    case 'D': latest_key = 'L'; break; // Mũi tên TRÁI (Trái)
                    default: break;
                }
            }
        } else {
            latest_key = c;
        }
    }
    return latest_key;
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
    std::cout << " (Chống trôi gió tự nhiên - Khóa mỏ neo - Phanh tức thì)" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << " [A]            : HẠ CÁNH AN TOÀN (Giữ vị trí đáp thẳng đứng)" << std::endl;
    std::cout << " [Q]            : KHỞI ĐỘNG DRONE BAY LÊN TIẾP (Cất cánh lên 2.0m)" << std::endl;
    std::cout << " [Phím Mũi tên] : Lái Tiến / Lùi / Trái / Phải (Nhả ra tự phanh khóa neo)" << std::endl;
    std::cout << " [W / S]        : Tăng / Giảm độ cao (Bấm giữ để leo/hạ, nhả ra đứng yên)" << std::endl;
    std::cout << " [Z / C / E / D]: Xoay đầu (Yaw) Trái / Phải (Tự xoay véc-tơ bù gió)" << std::endl;
    std::cout << " [1 / 2 / 3]    : Đổi chế độ tốc độ: 1-Chậm (1m/s), 2-Vừa (2.5m/s), 3-Nhanh (5m/s)" << std::endl;
    std::cout << " [SPACE]        : Phanh đứng khẩn cấp (Emergency Hover Lock)" << std::endl;
    std::cout << " [X]            : Thoát chương trình" << std::endl;
    std::cout << "============================================================" << std::endl;

    DroneCore::VehicleCommander drone;
    DroneControllers::AltitudeController alt_controller(0.59, 0.35, 0.08, 0.22);
    DroneControllers::PositionController pos_controller;

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
    std::cout << "[🚀] Tự động cất cánh ban đầu lên 2.0m (Tự khóa vị trí chống gió)..." << std::endl;
    double target_alt = 0.0;
    float current_target_yaw_rad = drone.state().yaw.load();
    pos_controller.set_anchor(drone.state().pos_x.load(), drone.state().pos_y.load());

    auto takeoff_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - takeoff_start).count() < 5.0) {
        target_alt = std::min(2.0, target_alt + 0.025);
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        float thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);

        float r_target = 0.0f;
        float p_target = 0.0f;
        if (current_alt > 0.08) {
            auto pos_out = pos_controller.update(
                drone.state().pos_x.load(), drone.state().pos_y.load(),
                drone.state().vx.load(), drone.state().vy.load(),
                drone.state().yaw.load(),
                0.0f, 0.0f, false, false, 10.0f, 0.033f
            );
            r_target = pos_out.roll_deg;
            p_target = pos_out.pitch_deg;
        }

        drone.send_attitude_target(DroneMath::deg2rad(r_target), DroneMath::deg2rad(p_target), current_target_yaw_rad, thrust);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    std::cout << "\n[🎮] SẴN SÀNG LÁI! Bấm mũi tên để bay, nhả ra tự phanh khóa vị trí.\n"
              << "     Ấn [A] để Hạ cánh an toàn | Ấn [Q] để Khởi động bay lên tiếp!\n" << std::endl;

    set_nonblocking_terminal(true);

    FlightState state = FlightState::FLYING;
    float target_roll_deg = 0.0f;
    float target_pitch_deg = 0.0f;
    float max_speed_mps = 2.5f;
    float max_tilt_deg = 12.0f;
    std::string speed_mode_name = "VỪA (2.5m/s | 12.0°)";

    float target_vx_cmd = 0.0f;
    float target_vy_cmd = 0.0f;
    int pitch_key_hits = 0;
    int roll_key_hits = 0;
    auto last_pitch_key_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto last_roll_key_time  = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto last_alt_key_time   = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    float alt_vel_cmd = 0.0f;
    bool alt_was_active = false;

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
                        pos_controller.set_anchor(drone.state().pos_x.load(), drone.state().pos_y.load());
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
                        pos_controller.set_anchor(drone.state().pos_x.load(), drone.state().pos_y.load());
                        target_alt = 0.15;
                        current_target_yaw_rad = drone.state().yaw.load();
                        state = FlightState::TAKEOFF;
                    } else if (state == FlightState::FLYING) {
                        target_alt = std::max(2.0, target_alt);
                        std::cout << "\n[🚀] PHÍM 'Q': Tự động đưa drone lên độ cao chuẩn " << target_alt << "m!" << std::endl;
                    }
                    break;

                // Điều khiển độ cao (khi đang bay: bấm giữ để tăng/giảm, nhả ra dừng ngay lập tức)
                case 'w': case 'W':
                    if (state == FlightState::FLYING) {
                        alt_vel_cmd = 0.85f; // leo lên tốc độ 0.85 m/s
                        last_alt_key_time = now;
                    }
                    break;
                case 's': case 'S':
                    if (state == FlightState::FLYING) {
                        alt_vel_cmd = -0.70f; // hạ xuống tốc độ 0.70 m/s
                        last_alt_key_time = now;
                    }
                    break;

                // Điều khiển Tiến / Lùi (Pitch): Mũi tên Lên / Xuống hoặc phím I / K
                case 'U': case 'i': case 'I':
                    if (state == FlightState::FLYING) {
                        target_vx_cmd = max_speed_mps;
                        last_pitch_key_time = now;
                        pitch_key_hits++;
                    }
                    break;
                case 'N': case 'k': case 'K':
                    if (state == FlightState::FLYING) {
                        target_vx_cmd = -max_speed_mps;
                        last_pitch_key_time = now;
                        pitch_key_hits++;
                    }
                    break;

                // Điều khiển Trái / Phải (Roll): Mũi tên Trái / Phải hoặc phím J / L
                case 'L': case 'j': case 'J':
                    if (state == FlightState::FLYING) {
                        target_vy_cmd = max_speed_mps;
                        last_roll_key_time = now;
                        roll_key_hits++;
                    }
                    break;
                case 'R': case 'l':
                    if (state == FlightState::FLYING) {
                        target_vy_cmd = -max_speed_mps;
                        last_roll_key_time = now;
                        roll_key_hits++;
                    }
                    break;

                // Xoay đầu (Yaw): Z (Trái) / C hoặc E hoặc D (Phải)
                case 'z': case 'Z':
                    current_target_yaw_rad = DroneMath::normalize_angle(current_target_yaw_rad + DroneMath::deg2rad(5.0f));
                    break;
                case 'c': case 'C': case 'e': case 'E': case 'd': case 'D':
                    current_target_yaw_rad = DroneMath::normalize_angle(current_target_yaw_rad - DroneMath::deg2rad(5.0f));
                    break;

                // Đổi chế độ tốc độ giới hạn
                case '1':
                    max_speed_mps = 1.0f;
                    max_tilt_deg = 8.0f;
                    speed_mode_name = "CHẬM (1.0m/s | 8.0°)";
                    break;
                case '2':
                    max_speed_mps = 2.5f;
                    max_tilt_deg = 12.0f;
                    speed_mode_name = "VỪA (2.5m/s | 12.0°)";
                    break;
                case '3':
                    max_speed_mps = 5.0f;
                    max_tilt_deg = 18.0f;
                    speed_mode_name = "NHANH (5.0m/s | 18.0°)";
                    break;

                // Phím cách: Phanh đứng khẩn cấp
                case ' ':
                    if (state == FlightState::FLYING) {
                        target_roll_deg = 0.0f;
                        target_pitch_deg = 0.0f;
                        target_vx_cmd = 0.0f;
                        target_vy_cmd = 0.0f;
                        target_alt = drone.state().altitude.load();
                        current_target_yaw_rad = drone.state().yaw.load();
                        pos_controller.set_anchor(drone.state().pos_x.load(), drone.state().pos_y.load());
                        pitch_key_hits = 0;
                        roll_key_hits = 0;
                        last_pitch_key_time = now - std::chrono::seconds(1);
                        last_roll_key_time = now - std::chrono::seconds(1);
                        last_alt_key_time = now - std::chrono::seconds(1);
                        alt_controller.reset();
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
            // 1. Quản lý độ cao: khi buông phím W/S, chốt độ cao hiện tại êm dịu (không xóa tích phân ga)
            bool alt_active = (std::chrono::duration<double>(now - last_alt_key_time).count() <= 0.45);
            if (alt_active) {
                target_alt += alt_vel_cmd * 0.033;
                target_alt = DroneMath::clamp(target_alt, 0.3, 10.0);
            } else {
                if (alt_was_active) {
                    target_alt = current_alt;
                    // Giữ nguyên tích phân của alt_controller để không làm sụt ga đột ngột!
                }
            }
            alt_was_active = alt_active;

            // 2. Quản lý di chuyển ngang & chống gió tự nhiên
            double time_since_pitch = std::chrono::duration<double>(now - last_pitch_key_time).count();
            double time_since_roll  = std::chrono::duration<double>(now - last_roll_key_time).count();

            // Độ trễ nhả phím thích ứng (Adaptive Key Release Timeout):
            // - Khi đang giữ phím (repeat đang chạy, hits >= 2): chỉ cần 0.12s (120ms) không nhận thêm phím
            //   là xác nhận người lái ĐÃ BUÔNG TAY, kích hoạt phanh lập tức (nhanh gấp 4 lần mức 0.45s cũ).
            // - Khi mới gõ 1 phím (hits == 1): cho phép 0.25s để chờ phím lặp đầu tiên.
            double pitch_timeout = (pitch_key_hits >= 2) ? 0.12 : 0.25;
            bool pitch_active = (time_since_pitch <= pitch_timeout);
            if (!pitch_active) {
                pitch_key_hits = 0;
            }

            double roll_timeout = (roll_key_hits >= 2) ? 0.12 : 0.25;
            bool roll_active  = (time_since_roll  <= roll_timeout);
            if (!roll_active) {
                roll_key_hits = 0;
            }

            float current_yaw = drone.state().yaw.load();
            float vx_body = drone.state().vx.load();
            float vy_body = drone.state().vy.load();

            auto pos_out = pos_controller.update(
                drone.state().pos_x.load(), drone.state().pos_y.load(),
                vx_body, vy_body,
                current_yaw,
                pitch_active ? target_vx_cmd : 0.0f,
                roll_active  ? target_vy_cmd : 0.0f,
                pitch_active, roll_active,
                max_tilt_deg,
                0.033f
            );

            target_pitch_deg = pos_out.pitch_deg;
            target_roll_deg  = pos_out.roll_deg;

            // Ga nâng từ AltitudeController (vehicle_commander.cpp đã tự động bù tilt compensation trên tầng attitude)
            float total_thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);

            drone.send_attitude_target(DroneMath::deg2rad(target_roll_deg),
                                       DroneMath::deg2rad(target_pitch_deg),
                                       current_target_yaw_rad,
                                       total_thrust);

            std::string mode_tag;
            if (pos_out.is_braking) {
                mode_tag = "🛑 PHANH TỨC THÌ";
            } else if (pos_out.is_hover) {
                mode_tag = "⚓ HOVER (KHÓA NEO)";
            } else {
                mode_tag = "🎮 LÁI " + speed_mode_name;
            }

            std::cout << "\r[" << std::setw(18) << std::left << mode_tag << "] "
                      << "Alt: " << std::fixed << std::setprecision(2) << current_alt << "m (Đặt: " << target_alt << "m) | "
                      << "Roll: " << std::setw(5) << std::setprecision(1) << target_roll_deg << "° | "
                      << "Pitch: " << std::setw(5) << target_pitch_deg << "° | "
                      << "Yaw: " << std::setw(5) << DroneMath::rad2deg(current_yaw) << "° | "
                      << "Vx: " << std::setw(5) << vx_body << "m/s | "
                      << "Vy: " << std::setw(5) << vy_body << "m/s | "
                      << "BùGió(P/R): " << std::setw(4) << pos_out.wind_pitch_trim_deg << "°/" << pos_out.wind_roll_trim_deg << "°   " << std::flush;

        } else if (state == FlightState::LANDING) {
            target_alt = std::max(0.05, target_alt - 0.02); // Hạ cánh từ từ ~0.6 m/s
            float thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);

            float r_target = 0.0f;
            float p_target = 0.0f;
            // Khi còn trên cao (> 0.20m), duy trì khóa mỏ neo để gió không thổi trôi!
            if (current_alt > 0.20) {
                auto pos_out = pos_controller.update(
                    drone.state().pos_x.load(), drone.state().pos_y.load(),
                    drone.state().vx.load(), drone.state().vy.load(),
                    drone.state().yaw.load(),
                    0.0f, 0.0f, false, false, 8.0f, 0.033f
                );
                r_target = pos_out.roll_deg;
                p_target = pos_out.pitch_deg;
            }

            drone.send_attitude_target(DroneMath::deg2rad(r_target), DroneMath::deg2rad(p_target), current_target_yaw_rad, thrust);

            std::cout << "\r[🛬 HẠ CÁNH AN TOÀN] Độ cao: " << std::fixed << std::setprecision(2) << current_alt 
                      << "m | Vận tốc rơi: " << vz << "m/s   " << std::flush;

            // Kiểm tra chạm đất
            if (current_alt <= 0.10) {
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
            target_alt = std::min(2.0, target_alt + 0.025); // Leo lên mượt mà dạng dốc
            float thrust = alt_controller.compute_thrust(target_alt, current_alt, vz, 0.033);

            float r_target = 0.0f;
            float p_target = 0.0f;
            if (current_alt > 0.25) {
                auto pos_out = pos_controller.update(
                    drone.state().pos_x.load(), drone.state().pos_y.load(),
                    drone.state().vx.load(), drone.state().vy.load(),
                    drone.state().yaw.load(),
                    0.0f, 0.0f, false, false, 8.0f, 0.033f
                );
                r_target = pos_out.roll_deg;
                p_target = pos_out.pitch_deg;
            }

            drone.send_attitude_target(DroneMath::deg2rad(r_target), DroneMath::deg2rad(p_target), current_target_yaw_rad, thrust);

            std::cout << "\r[🚀 ĐANG CẤT CÁNH LÊN 2.0M] Độ cao: " << std::fixed << std::setprecision(2) << current_alt 
                      << "m / 2.00m | Vz: " << vz << "m/s   " << std::flush;

            if (target_alt >= 2.0 && current_alt >= 1.90) {
                pos_controller.set_anchor(drone.state().pos_x.load(), drone.state().pos_y.load());
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
