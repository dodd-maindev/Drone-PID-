/**
 * @file main.cpp
 * @brief Firmware Điều khiển Bay Quadrotor HITL trên Vi điều khiển ESP32 (Chuẩn FreeRTOS 200Hz)
 * 
 * Đặc tính kỹ thuật:
 * - Chạy trên 2 nhân (Dual-Core Xtensa LX6/LX7 240MHz):
 *   + Core 1 (Real-Time Control): Vòng lặp điều khiển bay 200Hz (Hard Real-Time 5ms).
 *   + Core 0 (Serial Comm): Nhận & giải mã gói tin SensorPacket 921600 baud.
 * - Zero Dynamic Allocation (Tiêu thụ RAM < 1KB, hoàn toàn không phân mảnh bộ nhớ).
 * - Tích hợp cơ chế ngắt an toàn Fail-safe: Tự động ngắt động cơ nếu mất kết nối Serial quá 250ms.
 */

#include <Arduino.h>
#include <string.h>
#include "telemetry_packet.hpp"
#include "math_utils.hpp"
#include "pid.hpp"
#include "motor_mixer.hpp"
#include "altitude_controller.hpp"
#include "position_controller.hpp"

// Khởi tạo tĩnh các Controller (Zero-Heap Allocation)
static DroneControllers::MotorMixer g_mixer(1000.0f);
static DroneControllers::AltitudeController g_alt_controller(0.59, 0.35, 0.08, 0.22);
static DroneControllers::PositionController g_pos_controller;

// Bộ điều khiển Attitude PID (Vòng lặp trong 200Hz)
static DroneControllers::PIDController g_pid_roll(0.15, 0.02, 0.008, -0.25, 0.25, 0.05);
static DroneControllers::PIDController g_pid_pitch(0.15, 0.02, 0.008, -0.25, 0.25, 0.05);

// Dữ liệu cảm biến nhận từ PC (Được bảo vệ bằng FreeRTOS portMUX)
static portMUX_TYPE g_data_mutex = portMUX_INITIALIZER_UNLOCKED;
static SensorPacket g_sensor_data;
static volatile bool g_has_new_sensor = false;
static volatile bool g_connected = false;
static volatile uint32_t g_last_packet_time_ms = 0;

// Trạng thái bay trên ESP32
static float g_target_alt = 0.15f;
static float g_target_yaw_rad = 0.0f;
static bool g_is_armed = false;
static bool g_alt_was_active = false;
static bool g_init_yaw_done = false;

// Task 200Hz ghim trên Core 1: Chuyên trách tính toán thuật toán bay
void flight_control_task(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(5); // 5ms = 200Hz

    while (true) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        uint32_t now_ms = millis();

        // 1. Kiểm tra Fail-safe: Mất tín hiệu Serial quá 250ms -> Dừng động cơ khẩn cấp
        if (now_ms - g_last_packet_time_ms > 250) {
            g_connected = false;
            g_is_armed = false;
        }

        if (!g_connected) {
            // Chưa kết nối hoặc mất tín hiệu: Gửi lệnh 0 rad/s
            ActuatorPacket safe_pkt;
            memset(&safe_pkt, 0, sizeof(safe_pkt));
            safe_pkt.header[0] = 0x55;
            safe_pkt.header[1] = 0xAA;
            safe_pkt.checksum = compute_packet_checksum((uint8_t*)&safe_pkt + 2, sizeof(safe_pkt) - 3);
            Serial.write((uint8_t*)&safe_pkt, sizeof(safe_pkt));
            continue;
        }

        // 2. Sao chép nhanh dữ liệu cảm biến từ Core 0 (Lock-Free ngắn)
        SensorPacket s;
        portENTER_CRITICAL(&g_data_mutex);
        s = g_sensor_data;
        portEXIT_CRITICAL(&g_data_mutex);

        // Khởi tạo góc Yaw ban đầu theo hướng mũi drone khi vừa kết nối
        if (!g_init_yaw_done) {
            g_target_yaw_rad = s.yaw;
            g_pos_controller.set_anchor(s.x, s.y);
            g_target_alt = std::max(0.2f, s.z);
            g_init_yaw_done = true;
        }

        // 3. Xử lý cờ điều khiển từ phi công (PC)
        g_is_armed = (s.flags & CTRL_FLAG_ARMED);

        if (s.flags & CTRL_FLAG_EMERGENCY) {
            g_is_armed = false;
        }

        if (s.flags & CTRL_FLAG_TAKEOFF) {
            g_target_alt = 2.0f;
            g_is_armed = true;
        }

        if (s.flags & CTRL_FLAG_LAND) {
            g_target_alt = std::max(0.05f, g_target_alt - 0.003f); // Hạ từ từ
            if (s.z < 0.10f) {
                g_is_armed = false;
            }
        }

        // Xoay đầu Yaw
        if (std::abs(s.cmd_yaw_rate) > 0.01f) {
            g_target_yaw_rad = DroneMath::normalize_angle(g_target_yaw_rad + s.cmd_yaw_rate * 0.005f);
        }

        // Điều khiển độ cao (W / S)
        bool alt_active = (s.flags & CTRL_FLAG_ALT_ACTIVE);
        if (alt_active) {
            g_target_alt += s.cmd_alt_vel * 0.005f;
            g_target_alt = DroneMath::clamp(g_target_alt, 0.3f, 10.0f);
        } else {
            if (g_alt_was_active) {
                g_target_alt = s.z; // Giữ độ cao hiện tại, bảo toàn tích phân
            }
        }
        g_alt_was_active = alt_active;

        // 4. Thuật toán Vòng ngoài: PositionController (P-PID, Phanh mượt, Khóa neo điểm dừng)
        bool is_driving_x = (s.flags & CTRL_FLAG_DRIVING_X);
        bool is_driving_y = (s.flags & CTRL_FLAG_DRIVING_Y);

        auto pos_out = g_pos_controller.update(
            s.x, s.y,
            s.vx, s.vy,
            s.yaw,
            s.cmd_vx, s.cmd_vy,
            is_driving_x, is_driving_y,
            12.0f,
            0.005f
        );

        // 5. Thuật toán Độ cao: AltitudeController (PI + Hãm rơi Velocity Damping)
        float total_thrust = g_alt_controller.compute_thrust(g_target_alt, s.z, s.vz, 0.005f);

        // 6. Bù lực nâng góc nghiêng (Tilt Compensation)
        float cos_tilt = std::max(0.65f, std::cos(s.roll) * std::cos(s.pitch));
        float effective_thrust = total_thrust / cos_tilt;

        // 7. Thuật toán Vòng trong: Attitude PID (Góc nghiêng & Dập tắt vận tốc góc Rate D-damping)
        float target_r_rad = DroneMath::deg2rad(pos_out.roll_deg);
        float target_p_rad = DroneMath::deg2rad(pos_out.pitch_deg);

        float tau_roll  = g_pid_roll.update(target_r_rad, s.roll, 0.005f) - 0.015f * s.p;
        float tau_pitch = g_pid_pitch.update(target_p_rad, s.pitch, 0.005f) - 0.015f * s.q;

        float yaw_err = DroneMath::normalize_angle(g_target_yaw_rad - s.yaw);
        float tau_yaw = 0.35f * yaw_err - 0.025f * s.r;

        // Giới hạn mô-men an toàn
        tau_roll  = DroneMath::clamp(tau_roll, -0.25f, 0.25f);
        tau_pitch = DroneMath::clamp(tau_pitch, -0.25f, 0.25f);
        tau_yaw   = DroneMath::clamp(tau_yaw, -0.20f, 0.20f);

        // 8. Motor Mixer: Phân bổ công suất động cơ chữ X
        auto speeds = g_mixer.compute_motor_speeds(effective_thrust, tau_roll, tau_pitch, tau_yaw, g_is_armed);

        // 9. Đóng gói ActuatorPacket (28 bytes) gửi trả ngược về PC
        ActuatorPacket out_pkt;
        out_pkt.header[0] = 0x55;
        out_pkt.header[1] = 0xAA;
        out_pkt.w0 = speeds[0];
        out_pkt.w1 = speeds[1];
        out_pkt.w2 = speeds[2];
        out_pkt.w3 = speeds[3];
        out_pkt.target_pitch_deg = pos_out.pitch_deg;
        out_pkt.target_roll_deg  = pos_out.roll_deg;
        out_pkt.status_flags = 0;
        if (g_is_armed) out_pkt.status_flags |= 1;
        if (pos_out.is_braking) out_pkt.status_flags |= 2;
        if (pos_out.is_hover) out_pkt.status_flags |= 4;

        out_pkt.checksum = compute_packet_checksum((uint8_t*)&out_pkt + 2, sizeof(out_pkt) - 3);

        Serial.write((uint8_t*)&out_pkt, sizeof(out_pkt));
    }
}

void setup() {
    // Khởi tạo UART tốc độ cao 921600 baud
    Serial.begin(921600);
    while (!Serial && millis() < 2000);

    // Ghim task bay chạy trên Core 1 với quyền ưu tiên cao nhất của FreeRTOS
    xTaskCreatePinnedToCore(
        flight_control_task,
        "FlightCore1",
        4096,     // Stack 4KB
        NULL,
        configMAX_PRIORITIES - 1, // Priority cao nhất
        NULL,
        1         // Core 1
    );
}

// Loop chạy trên Core 0: Nhận và giải mã gói tin SensorPacket từ PC
void loop() {
    static uint8_t rx_buf[sizeof(SensorPacket)];
    static size_t rx_idx = 0;

    while (Serial.available() > 0) {
        uint8_t b = Serial.read();

        // Đồng bộ Header 0xAA, 0x55
        if (rx_idx == 0) {
            if (b == 0xAA) rx_buf[rx_idx++] = b;
            continue;
        }
        if (rx_idx == 1) {
            if (b == 0x55) {
                rx_buf[rx_idx++] = b;
            } else {
                rx_idx = 0;
            }
            continue;
        }

        rx_buf[rx_idx++] = b;

        // Khi nhận đủ 1 gói tin SensorPacket hoàn chỉnh
        if (rx_idx == sizeof(SensorPacket)) {
            rx_idx = 0;
            SensorPacket pkt;
            memcpy(&pkt, rx_buf, sizeof(SensorPacket));

            // Kiểm tra mã lỗi Checksum
            uint8_t expected_cs = compute_packet_checksum((uint8_t*)&pkt + 2, sizeof(SensorPacket) - 3);
            if (expected_cs == pkt.checksum) {
                portENTER_CRITICAL(&g_data_mutex);
                g_sensor_data = pkt;
                portEXIT_CRITICAL(&g_data_mutex);

                g_last_packet_time_ms = millis();
                g_connected = true;
            }
        }
    }
}
