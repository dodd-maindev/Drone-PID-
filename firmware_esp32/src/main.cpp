/**
 * @file main.cpp
 * @brief Firmware Điều khiển Bay Quadrotor HITL trên ESP32 (Chuẩn FreeRTOS 200Hz - Cấu trúc Module)
 * 
 * Kiến trúc phần mềm:
 * - 100% Tham số, hệ số PID, ngưỡng an toàn đặt tại: include/flight_config.h
 * - Các thuật toán được module hóa trong: include/
 * - File main.cpp chịu trách nhiệm khởi chạy 2 FreeRTOS tasks:
 *   + Core 1: flight_control_task (200Hz Hard Real-time)
 *   + Core 0 / loop: UART Serial Packet Parser (921600 baud, sliding-window XOR checksum)
 */

#include <Arduino.h>
#include <string.h>

#include "flight_config.h"
#include "telemetry_packet.hpp"
#include "math_utils.hpp"
#include "pid.hpp"
#include "motor_mixer.hpp"
#include "altitude_controller.hpp"
#include "position_controller.hpp"

using namespace FlightConfig;
using namespace DroneControllers;

// Khởi tạo tĩnh các Controller (Zero-Heap Allocation)
static MotorMixer g_mixer(Mixer::MAX_MOTOR_ROT_VELOCITY);
static AltitudeController g_alt_controller;
static PositionController g_pos_controller;

// Dữ liệu cảm biến nhận từ PC (Được bảo vệ bằng FreeRTOS portMUX)
static portMUX_TYPE g_data_mutex = portMUX_INITIALIZER_UNLOCKED;
static SensorPacket g_sensor_data;
static volatile bool g_connected = false;
static volatile uint32_t g_last_packet_time_ms = 0;

// Trạng thái bay trên ESP32
static float g_target_alt = 0.15f;
static float g_target_yaw_rad = 0.0f;
static bool g_is_armed = false;
static bool g_alt_was_active = false;
static bool g_init_yaw_done = false;

static float g_int_roll = 0.0f;
static float g_int_pitch = 0.0f;

// Task 200Hz: Chạy vòng lặp điều khiển thời gian thực
void flight_control_task(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(Timing::LOOP_PERIOD_TICKS_MS);

    while (true) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        uint32_t now_ms = millis();
        // Fail-safe: Ngắt khẩn cấp nếu mất tín hiệu từ máy tính quá FAILSAFE_TIMEOUT_MS
        if (now_ms - g_last_packet_time_ms > Hardware::FAILSAFE_TIMEOUT_MS) {
            g_connected = false;
            g_is_armed = false;
        }

        if (!g_connected) {
            ActuatorPacket safe_pkt;
            memset(&safe_pkt, 0, sizeof(safe_pkt));
            safe_pkt.header[0] = 0x55;
            safe_pkt.header[1] = 0xAA;
            safe_pkt.checksum = compute_packet_checksum((uint8_t*)&safe_pkt + 2, sizeof(safe_pkt) - 3);
            Serial.write((uint8_t*)&safe_pkt, sizeof(safe_pkt));
            continue;
        }

        // Sao chép nhanh dữ liệu cảm biến
        SensorPacket s;
        portENTER_CRITICAL(&g_data_mutex);
        s = g_sensor_data;
        portEXIT_CRITICAL(&g_data_mutex);

        // Khởi tạo điểm neo ban đầu khi mới kết nối
        if (!g_init_yaw_done) {
            g_target_yaw_rad = s.yaw;
            g_pos_controller.set_anchor(s.x, s.y);
            g_target_alt = std::max(0.2f, s.z);
            g_init_yaw_done = true;
        }

        g_is_armed = (s.flags & CTRL_FLAG_ARMED);
        if (s.flags & CTRL_FLAG_EMERGENCY) {
            g_is_armed = false;
        }

        // Cất cánh tự động: tăng dần độ cao theo chu kỳ
        if (s.flags & CTRL_FLAG_TAKEOFF) {
            g_target_alt = std::min(FlightLogic::TAKEOFF_TARGET_ALT_M, 
                                    g_target_alt + FlightLogic::TAKEOFF_STEP_PER_TICK);
            g_is_armed = true;
        }

        // Hạ cánh tự động: giảm dần độ cao, chạm đất thì ngắt động cơ
        if (s.flags & CTRL_FLAG_LAND) {
            g_target_alt = std::max(0.05f, g_target_alt - FlightLogic::LAND_STEP_PER_TICK);
            if (s.z < FlightLogic::TOUCHDOWN_ALT_THRESHOLD) {
                g_is_armed = false;
            }
        }

        // Cập nhật góc Yaw đặt khi phi công xoay cần lái
        if (std::abs(s.cmd_yaw_rate) > 0.01f) {
            g_target_yaw_rad = DroneMath::normalize_angle(g_target_yaw_rad + s.cmd_yaw_rate * Timing::DT);
        }

        // Điều khiển độ cao Z thủ công
        bool alt_active = (s.flags & CTRL_FLAG_ALT_ACTIVE);
        if (alt_active) {
            g_target_alt += s.cmd_alt_vel * Timing::DT;
            g_target_alt = DroneMath::clamp(g_target_alt, FlightLogic::MIN_FLIGHT_ALT_M, FlightLogic::MAX_FLIGHT_ALT_M);
        } else {
            if (g_alt_was_active) {
                g_target_alt = s.z;
            }
        }
        g_alt_was_active = alt_active;

        bool is_driving_x = (s.flags & CTRL_FLAG_DRIVING_X);
        bool is_driving_y = (s.flags & CTRL_FLAG_DRIVING_Y);

        // 1. VÒNG NGOÀI: Điều khiển Vị trí X/Y -> Ra góc nghiêng mong muốn (Roll, Pitch)
        auto pos_out = g_pos_controller.update(
            s.x, s.y, s.vx, s.vy, s.yaw,
            s.cmd_vx, s.cmd_vy, is_driving_x, is_driving_y,
            Position::MAX_TILT_DRIVING_DEG, Timing::DT
        );

        // 2. Điều khiển Lực nâng thẳng đứng (Altitude Controller)
        float total_thrust = g_alt_controller.compute_thrust(g_target_alt, s.z, s.vz, Timing::DT);
        float cos_tilt = std::max(Altitude::MIN_COS_TILT, std::cos(s.roll) * std::cos(s.pitch));
        float effective_thrust = total_thrust / cos_tilt;

        float target_r_rad = 0.0f;
        float target_p_rad = 0.0f;
        // Chỉ kích hoạt nghiêng thân lái vị trí khi đã nhấc khỏi mặt đất
        if (s.z > FlightLogic::MIN_TAKEOFF_ALT_FOR_TILT) {
            target_r_rad = DroneMath::deg2rad(pos_out.roll_deg);
            target_p_rad = DroneMath::deg2rad(pos_out.pitch_deg);
        } else {
            g_pos_controller.set_anchor(s.x, s.y);
        }

        // 3. VÒNG TRONG: Điều khiển Góc thái độ (Attitude PID + Gyro Rate Damping)
        // Roll PID
        float err_roll = target_r_rad - s.roll;
        g_int_roll += err_roll * Timing::DT;
        g_int_roll = DroneMath::clamp(g_int_roll, -Attitude::INT_ATT_LIMIT, Attitude::INT_ATT_LIMIT);
        float tau_roll = Attitude::KP_ATT * err_roll + Attitude::KI_ATT * g_int_roll - Attitude::KD_ATT * s.p;

        // Pitch PID
        float err_pitch = target_p_rad - s.pitch;
        g_int_pitch += err_pitch * Timing::DT;
        g_int_pitch = DroneMath::clamp(g_int_pitch, -Attitude::INT_ATT_LIMIT, Attitude::INT_ATT_LIMIT);
        float tau_pitch = Attitude::KP_ATT * err_pitch + Attitude::KI_ATT * g_int_pitch - Attitude::KD_ATT * s.q;

        // Yaw PD
        float err_yaw = DroneMath::normalize_angle(g_target_yaw_rad - s.yaw);
        float tau_yaw = Attitude::KP_YAW * err_yaw - Attitude::KD_YAW * s.r;

        // Giới hạn mô-men xoắn ngõ ra
        tau_roll  = DroneMath::clamp(tau_roll, -Attitude::MAX_TAU_ROLL, Attitude::MAX_TAU_ROLL);
        tau_pitch = DroneMath::clamp(tau_pitch, -Attitude::MAX_TAU_PITCH, Attitude::MAX_TAU_PITCH);
        tau_yaw   = DroneMath::clamp(tau_yaw, -Attitude::MAX_TAU_YAW, Attitude::MAX_TAU_YAW);

        // 4. BỘ TRỘN ĐỘNG CƠ: Phân bổ lực cho 4 cánh quạt (Motor Mixer)
        auto speeds = g_mixer.compute_motor_speeds(effective_thrust, tau_roll, tau_pitch, tau_yaw, g_is_armed);

        // Đóng gói ActuatorPacket gửi ngược về máy tính qua Serial
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
    pinMode(Hardware::STATUS_LED_PIN, OUTPUT);
    digitalWrite(Hardware::STATUS_LED_PIN, LOW);

    Serial.setRxBufferSize(Hardware::UART_RX_BUFFER_SIZE);
    Serial.begin(Hardware::UART_BAUD_RATE);
    while (!Serial && millis() < 2000);

#if CONFIG_FREERTOS_UNICORE || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S2)
    xTaskCreate(
        flight_control_task,
        "FlightTask",
        4096,
        NULL,
        2,
        NULL
    );
#else
    xTaskCreatePinnedToCore(
        flight_control_task,
        "FlightTask",
        4096,
        NULL,
        2,
        NULL,
        1
    );
#endif
}

void loop() {
    static uint8_t rx_buf[256];
    static size_t rx_len = 0;

    while (Serial.available() > 0) {
        if (rx_len < sizeof(rx_buf)) {
            rx_buf[rx_len++] = Serial.read();
        } else {
            memmove(rx_buf, rx_buf + 1, rx_len - 1);
            rx_len--;
            rx_buf[rx_len++] = Serial.read();
        }

        // Sliding-window parser: Không bao giờ bị lệch byte hay mất đồng bộ
        while (rx_len >= sizeof(SensorPacket)) {
            if (rx_buf[0] == 0xAA && rx_buf[1] == 0x55) {
                SensorPacket pkt;
                memcpy(&pkt, rx_buf, sizeof(SensorPacket));

                uint8_t expected_cs = compute_packet_checksum((uint8_t*)&pkt + 2, sizeof(SensorPacket) - 3);
                if (expected_cs == pkt.checksum) {
                    portENTER_CRITICAL(&g_data_mutex);
                    g_sensor_data = pkt;
                    portEXIT_CRITICAL(&g_data_mutex);

                    g_last_packet_time_ms = millis();
                    g_connected = true;

                    digitalWrite(Hardware::STATUS_LED_PIN, (millis() / 200) % 2);

                    memmove(rx_buf, rx_buf + sizeof(SensorPacket), rx_len - sizeof(SensorPacket));
                    rx_len -= sizeof(SensorPacket);
                } else {
                    memmove(rx_buf, rx_buf + 1, rx_len - 1);
                    rx_len--;
                }
            } else {
                memmove(rx_buf, rx_buf + 1, rx_len - 1);
                rx_len--;
            }
        }
    }
}
