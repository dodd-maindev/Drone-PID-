#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include "core/vehicle_commander.hpp"
#include "controllers/altitude_controller.hpp"
#include "math/math_utils.hpp"

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " THÍ NGHIỆM 2: GIỮ ĐỘ CAO PID CÔNG NGHIỆP (ULTRA-STABLE)" << std::endl;
    std::cout << " (Có khóa tọa độ ngang X-Y & Quỹ đạo leo dốc mượt không vọt đà)" << std::endl;
    std::cout << "============================================================" << std::endl;

    // 1. Khởi tạo Vehicle Commander và Altitude Controller
    DroneCore::VehicleCommander drone;
    // Điểm lơ lửng hover = 0.59 (chuẩn x500 trong Gazebo)
    DroneControllers::AltitudeController alt_controller(0.59, 0.35, 0.08, 0.22);

    if (!drone.start()) {
        std::cerr << "[!] Không thể kết nối với Gazebo Sim qua GZ-Transport!" << std::endl;
        return -1;
    }

    if (!drone.wait_for_connection(15)) {
        std::cerr << "[!] Quá thời gian chờ kết nối Gazebo!" << std::endl;
        return -1;
    }

    // 2. Bơm tín hiệu mồi 30Hz
    std::cout << "[*] Đang gửi dòng tín hiệu khởi động (30Hz)..." << std::endl;
    for (int i = 0; i < 30; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, drone.state().yaw.load(), 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // 3. Arm và bật Offboard Mode
    drone.arm(true);
    drone.set_offboard_mode();

    // Ghi nhớ góc hướng Yaw và tọa độ xuất phát (X0, Y0) để khóa cứng vị trí
    float initial_yaw = drone.state().yaw.load();
    float target_x = drone.state().pos_x.load();
    float target_y = drone.state().pos_y.load();

    // Hàm lambda tính góc nghiêng hiệu chỉnh vi sai để khóa tọa độ ngang X-Y thẳng tắp
    auto compute_position_lock_tilt = [&](float& corr_roll, float& corr_pitch) {
        float err_x = target_x - drone.state().pos_x.load();
        float err_y = target_y - drone.state().pos_y.load();
        float vx = drone.state().vx.load();
        float vy = drone.state().vy.load();

        // Chuyển sai số vị trí từ World Frame sang Body Frame theo góc Yaw hiện tại
        float current_yaw = drone.state().yaw.load();
        float cy = std::cos(current_yaw);
        float sy = std::sin(current_yaw);
        float err_xb =  cy * err_x + sy * err_y;
        float err_yb = -sy * err_x + cy * err_y;

        // Khi ở sát mặt đất (< 0.35m), giữ thăng bằng phẳng tuyệt đối (Roll=0, Pitch=0)
        if (drone.state().altitude.load() < 0.35) {
            corr_pitch = 0.0f;
            corr_roll  = 0.0f;
            return;
        }

        // Vùng chết (deadband) 2cm chống rung lắc vi mô
        if (std::abs(err_xb) < 0.02f) err_xb = 0.0f;
        if (std::abs(err_yb) < 0.02f) err_yb = 0.0f;

        // PD Position Control êm dịu, không giật (giới hạn max 0.8 độ)
        float p_gain = 0.15f;
        float d_gain = 0.20f;
        corr_pitch = DroneMath::clamp(p_gain * err_xb - d_gain * vx, -DroneMath::deg2rad(0.8f), DroneMath::deg2rad(0.8f));
        corr_roll  = DroneMath::clamp(-p_gain * err_yb + d_gain * vy, -DroneMath::deg2rad(0.8f), DroneMath::deg2rad(0.8f));
    };

    // --- BƯỚC 1: CẤT CÁNH LÊN 2.0M VÀ LƠ LỬNG TRONG 10 GIÂY ---
    double current_target_alt = 0.0;
    std::cout << "\n[🚀] BƯỚC 1: CẤT CÁNH LÊN ĐỘ CAO 2.00M (THẲNG TẮP NHƯ DÂY DỌI)..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < 10.0) {
        // Quỹ đạo leo mượt mà: vận tốc lên 0.75 m/s
        if (current_target_alt < 2.0) {
            current_target_alt = std::min(2.0, current_target_alt + 0.025);
        }

        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03;

        float thrust = alt_controller.compute_thrust(current_target_alt, current_alt, vz, dt);

        float corr_roll = 0.0f, corr_pitch = 0.0f;
        compute_position_lock_tilt(corr_roll, corr_pitch);

        drone.send_attitude_target(corr_roll, corr_pitch, initial_yaw, thrust);

        std::cout << "Mục tiêu: " << std::fixed << std::setprecision(2) << current_target_alt
                  << "m | Thực tế: " << current_alt << "m | Sai số Z: " << std::setw(6) << std::setprecision(3) << (current_target_alt - current_alt)
                  << "m | Lệch X: " << std::setprecision(2) << (drone.state().pos_x.load() - target_x)
                  << "m | Lệch Y: " << (drone.state().pos_y.load() - target_y)
                  << "m | Motors: [" << static_cast<int>(drone.state().motor_speed_0.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_1.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_2.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_3.load()) << "] rad/s\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // --- BƯỚC 2: QUỸ ĐẠO LEO LÊN 3.5M MƯỢT MÀ KHÔNG VỌT ĐÀ (ZERO OVERSHOOT) ---
    std::cout << "\n\n[📈] BƯỚC 2: QUỸ ĐẠO NÂNG ĐỘ CAO LÊN 3.50M (ÊM ÁI NHƯ THANG MÁY)..." << std::endl;
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < 10.0) {
        if (current_target_alt < 3.5) {
            current_target_alt = std::min(3.5, current_target_alt + 0.022);
        }

        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03;

        float thrust = alt_controller.compute_thrust(current_target_alt, current_alt, vz, dt);

        float corr_roll = 0.0f, corr_pitch = 0.0f;
        compute_position_lock_tilt(corr_roll, corr_pitch);

        drone.send_attitude_target(corr_roll, corr_pitch, initial_yaw, thrust);

        std::cout << "Mục tiêu: " << std::fixed << std::setprecision(2) << current_target_alt
                  << "m | Thực tế: " << current_alt << "m | Sai số Z: " << std::setw(6) << std::setprecision(3) << (current_target_alt - current_alt)
                  << "m | Lệch X: " << std::setprecision(2) << (drone.state().pos_x.load() - target_x)
                  << "m | Lệch Y: " << (drone.state().pos_y.load() - target_y)
                  << "m | Motors: [" << static_cast<int>(drone.state().motor_speed_0.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_1.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_2.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_3.load()) << "] rad/s\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // --- BƯỚC 3: HẠ CÁNH THẲNG ĐỨNG VỀ MẶT ĐẤT ---
    std::cout << "\n\n[🛬] BƯỚC 3: HẠ CÁNH THẲNG ĐỨNG VỀ MẶT ĐẤT..." << std::endl;
    while (drone.state().altitude.load() > 0.10) {
        if (current_target_alt > 0.0) {
            current_target_alt = std::max(0.0, current_target_alt - 0.015);
        }
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03;

        float thrust = alt_controller.compute_thrust(current_target_alt, current_alt, vz, dt);

        // Giữ phẳng hoàn toàn khi hạ cánh (Roll=0, Pitch=0) để 4 chân tiếp đất cùng lúc, không bị ngã
        drone.send_attitude_target(0.0f, 0.0f, initial_yaw, thrust);

        std::cout << "Hạ cánh: " << std::fixed << std::setprecision(2) << current_target_alt
                  << "m | Thực tế: " << current_alt << "m | Thrust: " << thrust
                  << " | Motors: [" << static_cast<int>(drone.state().motor_speed_0.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_1.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_2.load()) << ", "
                  << static_cast<int>(drone.state().motor_speed_3.load()) << "] rad/s\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // --- BƯỚC 4: NGẮT GA VÀ DISARM TẮT ĐỘNG CƠ ---
    std::cout << "\n\n[*] Chạm đất, Disarm tắt động cơ an toàn." << std::endl;
    for (int i = 0; i < 15; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, initial_yaw, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    drone.disarm();
    drone.stop();

    std::cout << "\n[✓] Hoàn thành xuất sắc bài thí nghiệm giữ độ cao siêu ổn định!" << std::endl;
    std::cout << "[*] Dữ liệu bay được tự động lưu vào /home/do/drone_control_cpp/log/latest_flight.csv" << std::endl;
    return 0;
}
