#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include "core/vehicle_commander.hpp"
#include "controllers/altitude_controller.hpp"

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " THÍ NGHIỆM 2: GIỮ ĐỘ CAO BẰNG MODULE PID (MODULAR C++)" << std::endl;
    std::cout << "============================================================" << std::endl;

    // 1. Khởi tạo Module Commander và Module Altitude Controller
    DroneCore::VehicleCommander drone;
    // Điểm lơ lửng hover = 0.59 (chuẩn x500 Gazebo), Kp = 0.35, Ki = 0.08, Kd = 0.22
    DroneControllers::AltitudeController alt_controller(0.59, 0.35, 0.08, 0.22);

    if (!drone.start(14540)) {
        return -1;
    }

    if (!drone.wait_for_connection(15)) {
        return -1;
    }

    // 2. Bơm tín hiệu mồi 30Hz
    std::cout << "[*] Đang gửi dòng tín hiệu khởi động (30Hz)..." << std::endl;
    for (int i = 0; i < 30; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // 3. Arm và bật Offboard
    drone.arm(true);
    for (int i = 0; i < 15; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    drone.set_offboard_mode();
    for (int i = 0; i < 15; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // --- BƯỚC 1: GIỮ ĐỘ CAO TARGET = 2.0M TRONG 12 GIÂY ---
    double target_altitude = 2.0;
    std::cout << "\n[🚀] BẮT ĐẦU VÒNG LẶP PID C++: Target = " << target_altitude << " m" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < 12.0) {
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03; // Chu kỳ ~30Hz (33ms)

        // Tính lực đẩy qua Module điều khiển độ cao
        float thrust = alt_controller.compute_thrust(target_altitude, current_alt, vz, dt);

        // Bơm lệnh tới Drone (Roll=0, Pitch=0 để giữ thăng bằng)
        drone.send_attitude_target(0.0f, 0.0f, 0.0f, thrust);

        std::cout << "Target: " << std::fixed << std::setprecision(2) << target_altitude
                  << "m | Actual: " << current_alt << "m | Err: " << (target_altitude - current_alt)
                  << "m | Thrust: " << thrust << "\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // --- BƯỚC 2: TĂNG ĐỘ CAO LÊN 3.5M TRONG 8 GIÂY ---
    target_altitude = 3.5;
    std::cout << "\n\n[📈] ĐỔI MỤC TIÊU LÊN: " << target_altitude << " m" << std::endl;
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < 8.0) {
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03;

        float thrust = alt_controller.compute_thrust(target_altitude, current_alt, vz, dt);
        drone.send_attitude_target(0.0f, 0.0f, 0.0f, thrust);

        std::cout << "Target: " << std::fixed << std::setprecision(2) << target_altitude
                  << "m | Actual: " << current_alt << "m | Err: " << (target_altitude - current_alt)
                  << "m | Thrust: " << thrust << "\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // --- BƯỚC 3: HẠ CÁNH MƯỢT VỀ 0.0M ---
    std::cout << "\n\n[🛬] HẠ CÁNH MƯỢT VỀ 0.0M..." << std::endl;
    double current_target = 3.5;
    while (current_target > 0.1) {
        current_target -= 0.03;
        double current_alt = drone.state().altitude.load();
        double vz = drone.state().vz.load();
        double dt = 0.03;

        float thrust = alt_controller.compute_thrust(current_target, current_alt, vz, dt);
        drone.send_attitude_target(0.0f, 0.0f, 0.0f, thrust);

        std::cout << "Target: " << std::fixed << std::setprecision(2) << current_target
                  << "m | Actual: " << current_alt << "m | Thrust: " << thrust << "\r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // --- BƯỚC 4: NGẮT GA VÀ DISARM ---
    std::cout << "\n\n[*] Chạm đất, Disarm động cơ." << std::endl;
    for (int i = 0; i < 20; ++i) {
        drone.send_attitude_target(0.0f, 0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    drone.disarm();
    drone.stop();
    std::cout << "\n[✓] Hoàn thành bài thí nghiệm 2!" << std::endl;
    return 0;
}
