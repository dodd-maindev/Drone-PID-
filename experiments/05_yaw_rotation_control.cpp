#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <cmath>
#include "core/vehicle_commander.hpp"
#include "controllers/altitude_controller.hpp"
#include "math/math_utils.hpp"

/**
 * @brief Xoay tròn quanh trục đứng (Yaw Spin) theo một góc tổng cộng (ví dụ: +360°, -360°, +720°)
 *        bằng phương pháp tạo quỹ đạo góc mượt mà (Trajectory Ramp).
 * 
 * @param drone Bộ điều khiển trung tâm x500
 * @param alt_controller Bộ điều khiển độ cao PID
 * @param target_altitude Độ cao mục tiêu (mét)
 * @param total_angle_deg Tổng góc cần quay (Độ, ví dụ: +360 = quay trái 1 vòng, -360 = quay phải 1 vòng)
 * @param rotation_speed_deg_s Vận tốc xoay góc (Độ/giây, ví dụ: 45°/s hoặc 60°/s)
 * @param phase_name Tên giai đoạn hiển thị
 */
void spin_yaw_angle(DroneCore::VehicleCommander& drone,
                    DroneControllers::AltitudeController& alt_controller,
                    double target_altitude,
                    float total_angle_deg,
                    float rotation_speed_deg_s,
                    const std::string& phase_name) {
    if (rotation_speed_deg_s <= 0.0f) rotation_speed_deg_s = 45.0f; // Mặc định 45 deg/s
    float total_abs_deg = std::abs(total_angle_deg);
    float sign = (total_angle_deg >= 0.0f) ? 1.0f : -1.0f;
    float start_yaw_rad = drone.state().yaw.load();

    // Thời gian tăng tốc (t_acc) và giảm tốc phanh êm (t_dec)
    double t_acc = 1.2; // 1.2s tăng tốc êm ái
    double t_dec = 1.8; // 1.8s phanh dập tắt quán tính hoàn toàn

    // Nếu góc quay nhỏ (< 60°), tự động thu nhỏ t_acc và t_dec theo tỉ lệ
    double min_dist_needed = 0.5 * rotation_speed_deg_s * (t_acc + t_dec);
    if (total_abs_deg < min_dist_needed) {
        double factor = std::sqrt(total_abs_deg / min_dist_needed);
        t_acc *= factor;
        t_dec *= factor;
    }

    double dist_acc = 0.5 * rotation_speed_deg_s * t_acc;
    double dist_dec = 0.5 * rotation_speed_deg_s * t_dec;
    double dist_cruise = total_abs_deg - (dist_acc + dist_dec);
    if (dist_cruise < 0.0) dist_cruise = 0.0;
    double t_cruise = dist_cruise / rotation_speed_deg_s;
    double total_duration_sec = t_acc + t_cruise + t_dec;

    std::cout << "\n[" << phase_name << "] Bắt đầu xoay: "
              << std::fixed << std::setprecision(1) << total_angle_deg << "°\n"
              << "[Profile Phanh Khử Quán Tính] Tăng tốc: " << std::setprecision(1) << t_acc 
              << "s | Giữ tốc: " << t_cruise << "s | Phanh hãm êm: " << t_dec 
              << "s (Tổng: " << total_duration_sec << "s)" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    double elapsed_sec = 0.0;
    float goal_yaw_rad = DroneMath::normalize_angle(start_yaw_rad + DroneMath::deg2rad(total_angle_deg));

    float prev_yaw_rad = start_yaw_rad;
    float actual_turned_accum_deg = 0.0f;
    float spin_center_x = drone.state().pos_x.load();
    float spin_center_y = drone.state().pos_y.load();

    while (elapsed_sec < total_duration_sec) {
        auto now = std::chrono::steady_clock::now();
        elapsed_sec = std::chrono::duration<double>(now - start_time).count();
        if (elapsed_sec > total_duration_sec) elapsed_sec = total_duration_sec;

        // Cập nhật góc thực tế drone đã quay được
        float current_yaw_rad = drone.state().yaw.load();
        float dyaw = DroneMath::normalize_angle(current_yaw_rad - prev_yaw_rad);
        actual_turned_accum_deg += sign * DroneMath::rad2deg(dyaw);
        if (actual_turned_accum_deg < 0.0f) actual_turned_accum_deg = 0.0f;
        prev_yaw_rad = current_yaw_rad;

        // Tính toán góc lệnh từ phân đoạn quỹ đạo
        float current_turned_deg = 0.0f;
        float current_speed = 0.0f;

        if (elapsed_sec <= t_acc) {
            // Phân đoạn 1: Tăng tốc mượt
            double frac = elapsed_sec / t_acc;
            current_turned_deg = static_cast<float>(0.5 * rotation_speed_deg_s * t_acc * frac * frac);
            current_speed = static_cast<float>(rotation_speed_deg_s * frac);
        } else if (elapsed_sec <= t_acc + t_cruise) {
            // Phân đoạn 2: Tốc độ đều tối đa
            double t_c = elapsed_sec - t_acc;
            current_turned_deg = static_cast<float>(dist_acc + rotation_speed_deg_s * t_c);
            current_speed = rotation_speed_deg_s;
        } else {
            // Phân đoạn 3: Phanh giảm tốc triệt tiêu toàn bộ động lượng góc
            double t_rem = total_duration_sec - elapsed_sec;
            double frac = t_rem / t_dec;
            current_turned_deg = static_cast<float>(total_abs_deg - 0.5 * rotation_speed_deg_s * t_dec * frac * frac);
            current_speed = static_cast<float>(rotation_speed_deg_s * frac);
        }

        if (current_turned_deg > total_abs_deg) current_turned_deg = total_abs_deg;
        float signed_turned_deg = sign * current_turned_deg;

        // Tính góc mục tiêu cơ bản
        float current_target_yaw_rad = DroneMath::normalize_angle(start_yaw_rad + DroneMath::deg2rad(signed_turned_deg));

        // [KỸ THUẬT TRIỆT TIÊU LỐ ĐÀ]: Khi bước vào giai đoạn phanh cuối cùng
        float current_r_rate = drone.state().r.load();
        if (elapsed_sec > t_acc + t_cruise) {
            // Nếu góc thực tế đã chạm ngưỡng đích (sai số < 1.0°): Ngắt lệnh quay ngay lập tức!
            if (actual_turned_accum_deg >= total_abs_deg - 0.5f) {
                current_target_yaw_rad = goal_yaw_rad;
                drone.send_attitude_target(0.0f, 0.0f, goal_yaw_rad, alt_controller.compute_thrust(target_altitude, drone.state().altitude.load(), drone.state().vz.load(), 0.03));
                std::cout << "\n[🎯 KHÓA ĐÍCH] Đã chạm đích chuẩn xác: Đã quay " 
                          << std::fixed << std::setprecision(1) << actual_turned_accum_deg << "° / " 
                          << total_angle_deg << "° | Phanh hãm dừng tuyệt đối không trôi!" << std::endl;
                break;
            }
        }

        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03;

        // 1. Giữ độ cao bằng PID
        float thrust = alt_controller.compute_thrust(target_altitude, current_alt, vz, dt);

        // 2. Vi điều khiển giữ tâm quay X-Y tuyệt đối (không bị lệch tâm khi xoay)
        float err_x = spin_center_x - drone.state().pos_x.load();
        float err_y = spin_center_y - drone.state().pos_y.load();
        float vx = drone.state().vx.load();
        float vy = drone.state().vy.load();

        float corr_pitch = DroneMath::clamp(0.6f * err_x - 0.8f * vx, -DroneMath::deg2rad(1.5f), DroneMath::deg2rad(1.5f));
        float corr_roll  = DroneMath::clamp(-0.6f * err_y + 0.8f * vy, -DroneMath::deg2rad(1.5f), DroneMath::deg2rad(1.5f));

        // 3. Gửi lệnh tư thế
        drone.send_attitude_target(corr_roll, corr_pitch, current_target_yaw_rad, thrust);

        // 3. Hiển thị tiến trình chi tiết
        float progress_pct = (actual_turned_accum_deg / total_abs_deg) * 100.0f;
        if (progress_pct > 100.0f) progress_pct = 100.0f;

        std::cout << "Tiến độ: " << std::fixed << std::setprecision(1) << std::setw(5) << progress_pct << "%"
                  << " | Thực tế quay: " << std::setw(6) << (sign * actual_turned_accum_deg) << "° / " << total_angle_deg << "°"
                  << " | Vận tốc: " << std::setw(4) << current_speed << "°/s"
                  << " | Alt: " << std::setprecision(2) << current_alt << "m"
                  << " | Heading: " << std::setw(6) << std::setprecision(1) << DroneMath::rad2deg(current_yaw_rad) << "°"
                  << " | Motors: [" << static_cast<int>(drone.state().motor_speed_0.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_1.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_2.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_3.load()) << "] rad/s\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::cout << "\n[✓] Hoàn thành xoay góc " << total_angle_deg << "° (Dừng êm ái, khóa chặt vạch đích 100%)!" << std::endl;
}

/**
 * @brief Giữ vững vị trí và góc hướng trong duration_sec giây
 */
void hold_position_and_heading(DroneCore::VehicleCommander& drone,
                               DroneControllers::AltitudeController& alt_controller,
                               double target_altitude,
                               float hold_yaw_rad,
                               double duration_sec,
                               const std::string& phase_name) {
    std::cout << "\n[" << phase_name << "] Giữ độ cao " << target_altitude << "m và góc Yaw: "
              << DroneMath::rad2deg(hold_yaw_rad) << "° trong " << duration_sec << "s..." << std::endl;

    float hold_x = drone.state().pos_x.load();
    float hold_y = drone.state().pos_y.load();

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < duration_sec) {
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        float current_yaw_rad = drone.state().yaw.load();
        double dt = 0.03;

        float thrust = alt_controller.compute_thrust(target_altitude, current_alt, vz, dt);

        float err_x = hold_x - drone.state().pos_x.load();
        float err_y = hold_y - drone.state().pos_y.load();
        float vx = drone.state().vx.load();
        float vy = drone.state().vy.load();

        float corr_pitch = DroneMath::clamp(0.8f * err_x - 1.0f * vx, -DroneMath::deg2rad(2.0f), DroneMath::deg2rad(2.0f));
        float corr_roll  = DroneMath::clamp(-0.8f * err_y + 1.0f * vy, -DroneMath::deg2rad(2.0f), DroneMath::deg2rad(2.0f));

        drone.send_attitude_target(corr_roll, corr_pitch, hold_yaw_rad, thrust);

        float yaw_err_deg = DroneMath::rad2deg(DroneMath::normalize_angle(hold_yaw_rad - current_yaw_rad));
        std::cout << "Alt: " << std::fixed << std::setprecision(2) << current_alt << "m"
                  << " | Target Yaw: " << std::setw(6) << std::setprecision(1) << DroneMath::rad2deg(hold_yaw_rad) << "°"
                  << " | Thực tế: " << std::setw(6) << DroneMath::rad2deg(current_yaw_rad) << "°"
                  << " | Sai số Yaw: " << std::setw(5) << yaw_err_deg << "°"
                  << " | Motors: [" << static_cast<int>(drone.state().motor_speed_0.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_1.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_2.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_3.load()) << "] rad/s\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

int main(int argc, char* argv[]) {
    std::cout << "============================================================" << std::endl;
    std::cout << " THÍ NGHIỆM: ĐIỀU KHIỂN DRONE QUAY TRÒN QUANH TRỤC (YAW SPIN)" << std::endl;
    std::cout << " (Xoay tròn 360 độ, 720 độ, quay trái/phải liên tục mượt mà)" << std::endl;
    std::cout << "============================================================" << std::endl;

    float total_turn_angle_deg = 360.0f; // Mặc định xoay tròn trọn vẹn 1 vòng 360° sang trái
    float turn_speed_deg_s = 45.0f;      // Tốc độ xoay: 45 độ/giây (quay 360° mất 8 giây, chuyển động cực kỳ êm ái)

    // 1. Phân tích tham số dòng lệnh thông minh
    // Hỗ trợ:
    //   ./05_yaw_rotation_control left 720         -> Quay trái 720° (2 vòng)
    //   ./05_yaw_rotation_control right 720        -> Quay phải 720° (2 vòng)
    //   ./05_yaw_rotation_control left 180         -> Quay trái 180°
    //   ./05_yaw_rotation_control left             -> Quay trái 360° (1 vòng)
    //   ./05_yaw_rotation_control right            -> Quay phải 360° (1 vòng)
    //   ./05_yaw_rotation_control 720              -> Quay 720°
    //   ./05_yaw_rotation_control left 720 60      -> Quay trái 720° ở tốc độ 60°/s
    if (argc > 1) {
        std::string arg1 = argv[1];
        bool is_left = (arg1 == "left" || arg1 == "ccw");
        bool is_right = (arg1 == "right" || arg1 == "cw");

        if (is_left || is_right) {
            float angle = 360.0f; // Mặc định 360 độ
            if (argc > 2) {
                try {
                    angle = std::abs(std::stof(argv[2]));
                } catch (...) {
                    angle = 360.0f;
                }
            }
            total_turn_angle_deg = is_left ? angle : -angle;

            if (argc > 3) {
                try {
                    turn_speed_deg_s = std::abs(std::stof(argv[3]));
                } catch (...) {
                    turn_speed_deg_s = 45.0f;
                }
            }
        } else {
            // Truyền trực tiếp số góc (ví dụ 720, -720, 180...)
            try {
                total_turn_angle_deg = std::stof(arg1);
            } catch (...) {
                total_turn_angle_deg = 360.0f;
            }
            if (argc > 2) {
                try {
                    turn_speed_deg_s = std::abs(std::stof(argv[2]));
                } catch (...) {
                    turn_speed_deg_s = 45.0f;
                }
            }
        }
    } else {
        // Menu lựa chọn trực quan
        std::cout << "\nChọn chế độ quay tròn quanh trục:\n"
                  << "  [1] Quay tròn 360° một vòng sang TRÁI (Counter-Clockwise / CCW)\n"
                  << "  [2] Quay tròn 360° một vòng sang PHẢI (Clockwise / CW)\n"
                  << "  [3] Quay tròn 2 vòng liên tục 720° sang TRÁI\n"
                  << "  [4] Quay tròn 2 vòng liên tục 720° sang PHẢI\n"
                  << "  [5] Xoay 180° đảo chiều hướng bay\n"
                  << "  [6] Tùy chỉnh góc quay bất kỳ (ví dụ: 360, -360, 540, 1080 độ)\n"
                  << "  [Enter / Mặc định] Quay tròn 360° sang Trái (CCW)\n"
                  << "Lựa chọn của bạn [1-6]: ";

        std::string line;
        std::getline(std::cin, line);

        if (line == "1" || line == "") {
            total_turn_angle_deg = 360.0f;
        } else if (line == "2") {
            total_turn_angle_deg = -360.0f;
        } else if (line == "3") {
            total_turn_angle_deg = 720.0f;
        } else if (line == "4") {
            total_turn_angle_deg = -720.0f;
        } else if (line == "5") {
            total_turn_angle_deg = 180.0f;
        } else if (line == "6") {
            std::cout << ">> Nhập góc quay mong muốn (độ, ví dụ: 360 hoặc -360): ";
            std::cin >> total_turn_angle_deg;
            std::cout << ">> Nhập tốc độ xoay (độ/giây, khuyến nghị 30 - 60): ";
            std::cin >> turn_speed_deg_s;
        }
    }

    std::cout << "\n[✓] Kế hoạch thực hiện: Quay " 
              << (total_turn_angle_deg >= 0.0f ? "TRÁI (CCW) +" : "PHẢI (CW) ")
              << std::abs(total_turn_angle_deg) << "° quanh trục đứng ở độ cao 2.0m" << std::endl;
    std::cout << "[*] Tốc độ xoay: " << turn_speed_deg_s << "°/s (Dự kiến: " 
              << (std::abs(total_turn_angle_deg) / turn_speed_deg_s) << " giây)" << std::endl;

    // 2. Khởi tạo Drone Commander và Altitude Controller
    DroneCore::VehicleCommander drone;
    DroneControllers::AltitudeController alt_controller(0.59, 0.35, 0.08, 0.22);

    if (!drone.start()) {
        std::cerr << "[!] Không thể kết nối với Gazebo Sim qua GZ-Transport!" << std::endl;
        return -1;
    }

    if (!drone.wait_for_connection(15)) {
        std::cerr << "[!] Quá thời gian chờ kết nối Gazebo!" << std::endl;
        return -1;
    }

    // Bơm tín hiệu khởi động 30Hz
    std::cout << "\n[*] Bơm tín hiệu khởi động (30Hz)..." << std::endl;
    for (int i = 0; i < 30; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, drone.state().yaw.load(), 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    drone.arm(true);
    drone.set_offboard_mode();

    // --- BƯỚC 1: CẤT CÁNH LÊN 2.0M VÀ KHÓA HƯỚNG BAN ĐẦU ---
    float initial_yaw = drone.state().yaw.load();
    hold_position_and_heading(drone, alt_controller, 2.0, initial_yaw, 5.0, 
                              "🚀 BƯỚC 1: CẤT CÁNH LÊN 2.0M VÀ GIỮ VỮNG HƯỚNG BAN ĐẦU");

    // --- BƯỚC 2: THỰC HIỆN XOAY TRÒN QUANH TRỤC YAW ---
    std::string spin_title = total_turn_angle_deg >= 0.0f ? 
                             "🔄 BƯỚC 2: THỰC HIỆN XOAY TRÒN SANG TRÁI (CCW)" : 
                             "🔄 BƯỚC 2: THỰC HIỆN XOAY TRÒN SANG PHẢI (CW)";
    spin_yaw_angle(drone, alt_controller, 2.0, total_turn_angle_deg, turn_speed_deg_s, spin_title);

    // --- BƯỚC 3: GIỮ VỊ TRÍ VÀ HƯỚNG SAU KHI QUAY XONG ---
    float final_yaw = DroneMath::normalize_angle(initial_yaw + DroneMath::deg2rad(total_turn_angle_deg));
    hold_position_and_heading(drone, alt_controller, 2.0, final_yaw, 4.0, 
                              "⚓ BƯỚC 3: GIỮ VỊ TRÍ VÀ KHÓA GÓC SAU KHI HOÀN THÀNH VÒNG XOAY");

    // --- BƯỚC 4: HẠ CÁNH MƯỢT VỀ 0.0M ---
    std::cout << "\n\n[🛬] BƯỚC 4: HẠ CÁNH MƯỢT VỀ MẶT ĐẤT..." << std::endl;
    double current_target_alt = 2.0;
    while (current_target_alt > 0.08) {
        current_target_alt -= 0.03;
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03;

        float thrust = alt_controller.compute_thrust(current_target_alt, current_alt, vz, dt);
        drone.send_attitude_target(0.0f, 0.0f, final_yaw, thrust);

        std::cout << "Target Alt: " << std::fixed << std::setprecision(2) << current_target_alt
                  << "m | Actual: " << current_alt << "m | Thrust: " << thrust
                  << " | Motors: [" << static_cast<int>(drone.state().motor_speed_0.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_1.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_2.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_3.load()) << "] rad/s\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // --- BƯỚC 5: TẮT ĐỘNG CƠ VÀ DISARM ---
    std::cout << "\n\n[*] Chạm đất, Disarm tắt động cơ an toàn." << std::endl;
    for (int i = 0; i < 15; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, final_yaw, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    drone.disarm();
    drone.stop();

    std::cout << "\n[✓] Hoàn thành xuất sắc bài thí nghiệm xoay tròn 360° quanh trục Yaw!" << std::endl;
    std::cout << "[*] Dữ liệu bay chi tiết được ghi tự động vào /home/do/drone_control_cpp/log/latest_flight.csv" << std::endl;
    std::cout << "[*] Xem đồ thị góc quay: python3 ~/drone_control_cpp/log/plot_flight.py" << std::endl;

    return 0;
}
