/**
 * @file firmware_esp32.ino
 * @brief Firmware Điều khiển Bay Quadrotor HITL cho ESP32 / ESP32-C3 / ESP32-S3
 * 
 * ============================================================================
 * KIẾN TRÚC PHẦN MỀM:
 * 1. Toàn bộ tham số, hệ số PID, giới hạn góc, tốc độ phanh ĐƯỢC ĐẶT TẠI: flight_config.h
 * 2. File này chỉ chứa luồng xử lý FreeRTOS và thuật toán lõi (Core Logic).
 * 3. Khi cần tinh chỉnh phản ứng của Drone, chỉ cần mở file flight_config.h.
 * ============================================================================
 */

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <cmath>
#include <algorithm>
#include <array>

#include "flight_config.h"
#include "telemetry_packet.hpp"
#include "math_utils.hpp"
#include "mavlink_telemetry.hpp"
#include "itsmc_controller.hpp"
#include "wifi_credential_storage.h"
#include "wifi_scanner_service.h"
#include "wifi_serial_configurator.h" 

using namespace FlightConfig;
using namespace DroneComm;

static MavlinkTelemetry g_mavlink;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// 3. BỘ ĐIỀU KHIỂN PID CƠ BẢN (PID CONTROLLER)
// ============================================================================
class PIDController {
public:
    PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0, 
                  double output_min = 0.0, double output_max = 1.0, 
                  double integral_max = 0.5)
        : kp_(kp), ki_(ki), kd_(kd), 
          output_min_(output_min), output_max_(output_max), 
          integral_max_(integral_max), integral_(0.0), prev_error_(0.0), first_run_(true) {}

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_run_ = true;
    }

    double update(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        if (first_run_) {
            prev_error_ = error;
            first_run_ = false;
        }
        if (dt <= 0.0) dt = Timing::DT;

        double p_term = kp_ * error;
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -integral_max_, integral_max_);
        double i_term = ki_ * integral_;

        double derivative = (error - prev_error_) / dt;
        double d_term = kd_ * derivative;

        double output = p_term + i_term + d_term;
        prev_error_ = error;
        return std::clamp(output, output_min_, output_max_);
    }

private:
    double kp_{0.0}, ki_{0.0}, kd_{0.0};
    double output_min_{0.0}, output_max_{1.0};
    double integral_max_{0.5};
    double integral_{0.0};
    double prev_error_{0.0};
    bool first_run_{true};
};

// ============================================================================
// 4. BỘ ĐIỀU KHIỂN ĐỘ CAO (ALTITUDE CONTROLLER)
//    Tham số đọc từ namespace FlightConfig::Altitude
// ============================================================================
class AltitudeController {
public:
    AltitudeController()
        : hover_thrust_(Altitude::HOVER_THRUST),
          pid_(Altitude::KP_ALT, Altitude::KI_ALT, 0.0, -0.30, 0.25, Altitude::INT_ALT_LIMIT) {}

    void reset() { pid_.reset(); }

    float compute_thrust(double target_alt, double current_alt, double vz, double dt) {
        double delta_thrust = pid_.update(target_alt, current_alt, dt);
        double total_thrust = hover_thrust_ + delta_thrust - Altitude::KV_DAMPING * vz;
        return static_cast<float>(std::clamp(total_thrust, 
                                             static_cast<double>(Altitude::MIN_TOTAL_THRUST), 
                                             static_cast<double>(Altitude::MAX_TOTAL_THRUST)));
    }

private:
    double hover_thrust_{Altitude::HOVER_THRUST};
    PIDController pid_;
};

// ============================================================================
// 5. BỘ ĐIỀU KHIỂN VỊ TRÍ 2D & PHANH DỪNG (POSITION CONTROLLER)
//    Tham số đọc từ namespace FlightConfig::Position
// ============================================================================
struct PositionControlOutput {
    float roll_deg{0.0f};
    float pitch_deg{0.0f};
    bool is_hover{true};
    bool is_braking{false};
    float anchor_x{0.0f};
    float anchor_y{0.0f};
};

class PositionController {
public:
    enum class AxisMode { DRIVING, BRAKING, HOLD };

    PositionController() = default;

    void reset() {
        i_wind_xw_ = 0.0f; i_wind_yw_ = 0.0f;
        smoothed_cmd_vx_ = 0.0f; smoothed_cmd_vy_ = 0.0f;
        filtered_pitch_deg_ = 0.0f; filtered_roll_deg_ = 0.0f;
        anchor_x_ = 0.0f; anchor_y_ = 0.0f;
        has_anchor_ = false;
        mode_x_ = AxisMode::HOLD; mode_y_ = AxisMode::HOLD;
        was_driving_x_ = false; was_driving_y_ = false;
        brake_timer_x_ = 0.0f; brake_timer_y_ = 0.0f;
    }

    void set_anchor(float x, float y) {
        anchor_x_ = x; anchor_y_ = y;
        has_anchor_ = true;
        mode_x_ = AxisMode::HOLD; mode_y_ = AxisMode::HOLD;
        smoothed_cmd_vx_ = 0.0f; smoothed_cmd_vy_ = 0.0f;
        brake_timer_x_ = 0.0f; brake_timer_y_ = 0.0f;
    }

    PositionControlOutput update(
        float current_x, float current_y,
        float vx_body, float vy_body,
        float current_yaw_rad,
        float cmd_vx_body, float cmd_vy_body,
        bool is_driving_x, bool is_driving_y,
        float max_tilt_deg = Position::MAX_TILT_DRIVING_DEG,
        float dt = Timing::DT
    ) {
        if (!has_anchor_) set_anchor(current_x, current_y);
        if (dt <= 0.001f || dt > 0.1f) dt = Timing::DT;

        float cy = std::cos(current_yaw_rad);
        float sy = std::sin(current_yaw_rad);
        float vx_b = vx_body;
        float vy_b = vy_body;

        const float max_dv = Position::MAX_CMD_ACCEL * dt;

        if (is_driving_x) {
            if (!was_driving_x_) smoothed_cmd_vx_ = vx_b;
            if (cmd_vx_body > smoothed_cmd_vx_) smoothed_cmd_vx_ = std::min(cmd_vx_body, smoothed_cmd_vx_ + max_dv);
            else smoothed_cmd_vx_ = std::max(cmd_vx_body, smoothed_cmd_vx_ - max_dv);
        } else {
            smoothed_cmd_vx_ = 0.0f;
        }

        if (is_driving_y) {
            if (!was_driving_y_) smoothed_cmd_vy_ = vy_b;
            if (cmd_vy_body > smoothed_cmd_vy_) smoothed_cmd_vy_ = std::min(cmd_vy_body, smoothed_cmd_vy_ + max_dv);
            else smoothed_cmd_vy_ = std::max(cmd_vy_body, smoothed_cmd_vy_ - max_dv);
        } else {
            smoothed_cmd_vy_ = 0.0f;
        }

        was_driving_x_ = is_driving_x;
        was_driving_y_ = is_driving_y;

        float target_vx_b = 0.0f;
        float target_vy_b = 0.0f;

        // ====================================================================
        // TRỤC X (TIẾN / LÙI)
        // ====================================================================
        if (is_driving_x) {
            mode_x_ = AxisMode::DRIVING;
            target_vx_b = smoothed_cmd_vx_;
            anchor_x_ = current_x;
            brake_timer_x_ = 0.0f;
            // GIỮ NGUYÊN i_wind_xw_: Không xóa về 0 để bảo toàn góc nghiêng kháng gió!
        } else {
            // Khi buông cần lái:
            if (was_driving_x_) {
                mode_x_ = AxisMode::BRAKING;
                brake_timer_x_ = 0.0f;
            }

            if (mode_x_ == AxisMode::BRAKING) {
                target_vx_b = 0.0f;
                brake_timer_x_ += dt;
                // Chuyển sang HOLD khi vận tốc nhỏ hoặc đã phanh đủ 0.35s (tránh bị gió giữ kẹt ở BRAKING)
                if (std::abs(vx_b) <= Position::STOP_VEL_THRESHOLD || brake_timer_x_ >= 0.35f) {
                    mode_x_ = AxisMode::HOLD;
                    anchor_x_ = current_x;
                }
            }

            if (mode_x_ == AxisMode::HOLD) {
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_xb  = cy * err_x_w + sy * err_y_w;

                // VÙNG TRƠ ĐIỂM NEO (Deadband): Khử hoàn toàn hiện tượng lắc lư qua lại quanh tâm!
                if (std::abs(err_xb) < Position::POS_DEADBAND_M) {
                    target_vx_b = 0.0f; // Ở trong tâm sai số < 3.5cm: Khóa chết target=0
                } else {
                    float eff_err_x = (err_xb > 0.0f) ? (err_xb - Position::POS_DEADBAND_M) : (err_xb + Position::POS_DEADBAND_M);
                    target_vx_b = DroneMath::clamp(Position::KP_POS * eff_err_x, -Position::MAX_APPROACH_VEL, Position::MAX_APPROACH_VEL);
                }
            }
        }

        // ====================================================================
        // TRỤC Y (TRÁI / PHẢI)
        // ====================================================================
        if (is_driving_y) {
            mode_y_ = AxisMode::DRIVING;
            target_vy_b = smoothed_cmd_vy_;
            anchor_y_ = current_y;
            brake_timer_y_ = 0.0f;
            // GIỮ NGUYÊN i_wind_yw_: Bảo toàn góc nghiêng bù gió ngang!
        } else {
            if (was_driving_y_) {
                mode_y_ = AxisMode::BRAKING;
                brake_timer_y_ = 0.0f;
            }

            if (mode_y_ == AxisMode::BRAKING) {
                target_vy_b = 0.0f;
                brake_timer_y_ += dt;
                if (std::abs(vy_b) <= Position::STOP_VEL_THRESHOLD || brake_timer_y_ >= 0.35f) {
                    mode_y_ = AxisMode::HOLD;
                    anchor_y_ = current_y;
                }
            }

            if (mode_y_ == AxisMode::HOLD) {
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_yb  = -sy * err_x_w + cy * err_y_w;

                if (std::abs(err_yb) < Position::POS_DEADBAND_M) {
                    target_vy_b = 0.0f;
                } else {
                    float eff_err_y = (err_yb > 0.0f) ? (err_yb - Position::POS_DEADBAND_M) : (err_yb + Position::POS_DEADBAND_M);
                    target_vy_b = DroneMath::clamp(Position::KP_POS * eff_err_y, -Position::MAX_APPROACH_VEL, Position::MAX_APPROACH_VEL);
                }
            }
        }

        float evx = target_vx_b - vx_b;
        float evy = target_vy_b - vy_b;

        const float int_limit = Position::MAX_WIND_ACCEL / Position::KI_VEL;

        // Tích phân ước lượng và kháng gió chủ động khi ở chế độ HOLD:
        // Tích lũy sai lệch vận tốc trong hệ tọa độ thế giới (World Frame)
        if (mode_x_ == AxisMode::HOLD) {
            float evx_w = cy * evx - sy * evy;
            i_wind_xw_ += evx_w * dt;
            i_wind_xw_ = DroneMath::clamp(i_wind_xw_, -int_limit, int_limit);
        }
        if (mode_y_ == AxisMode::HOLD) {
            float evy_w = sy * evx + cy * evy;
            i_wind_yw_ += evy_w * dt;
            i_wind_yw_ = DroneMath::clamp(i_wind_yw_, -int_limit, int_limit);
        }

        float wind_accel_xb =  cy * (Position::KI_VEL * i_wind_xw_) + sy * (Position::KI_VEL * i_wind_yw_);
        float wind_accel_yb = -sy * (Position::KI_VEL * i_wind_xw_) + cy * (Position::KI_VEL * i_wind_yw_);

        const float g = 9.80665f;
        float raw_pitch_deg = 0.0f;
        float raw_roll_deg  = 0.0f;

        // Trục X (Pitch)
        if (mode_x_ == AxisMode::DRIVING) {
            float a_sp_x = Position::KP_VEL * evx + wind_accel_xb;
            float ff_tilt_x = (smoothed_cmd_vx_ / 2.5f) * (max_tilt_deg * 0.35f);
            raw_pitch_deg = DroneMath::rad2deg(std::atan2(a_sp_x, g)) + ff_tilt_x;
        } else if (mode_x_ == AxisMode::BRAKING) {
            // Hãm phanh mượt mà: Lực hãm tỷ lệ thuận với vận tốc trôi, triệt tiêu dần về 0
            float a_brake_x = -Position::KP_BRAKE * vx_b;
            raw_pitch_deg = DroneMath::rad2deg(std::atan2(a_brake_x, g));
        } else {
            // HOLD: Dập tắt vận tốc và giữ vững điểm neo
            float a_sp_x = Position::KP_VEL * evx + wind_accel_xb;
            raw_pitch_deg = DroneMath::rad2deg(std::atan2(a_sp_x, g));
        }

        // Trục Y (Roll)
        if (mode_y_ == AxisMode::DRIVING) {
            float a_sp_y = Position::KP_VEL * evy + wind_accel_yb;
            float ff_tilt_y = (smoothed_cmd_vy_ / 2.5f) * (max_tilt_deg * 0.35f);
            raw_roll_deg = DroneMath::rad2deg(std::atan2(-a_sp_y, g)) - ff_tilt_y;
        } else if (mode_y_ == AxisMode::BRAKING) {
            float a_brake_y = -Position::KP_BRAKE * vy_b;
            raw_roll_deg = DroneMath::rad2deg(std::atan2(-a_brake_y, g));
        } else {
            float a_sp_y = Position::KP_VEL * evy + wind_accel_yb;
            raw_roll_deg = DroneMath::rad2deg(std::atan2(-a_sp_y, g));
        }

        float max_p = (mode_x_ == AxisMode::DRIVING) ? max_tilt_deg : ((mode_x_ == AxisMode::BRAKING) ? Position::MAX_TILT_BRAKING_DEG : Position::MAX_TILT_HOLD_DEG);
        float max_r = (mode_y_ == AxisMode::DRIVING) ? max_tilt_deg : ((mode_y_ == AxisMode::BRAKING) ? Position::MAX_TILT_BRAKING_DEG : Position::MAX_TILT_HOLD_DEG);

        raw_pitch_deg = DroneMath::clamp(raw_pitch_deg, -max_p, max_p);
        raw_roll_deg  = DroneMath::clamp(raw_roll_deg,  -max_r, max_r);

        float max_angle_step = Position::MAX_TILT_RATE_DEG_S * dt;

        filtered_pitch_deg_ = DroneMath::clamp(raw_pitch_deg, filtered_pitch_deg_ - max_angle_step, filtered_pitch_deg_ + max_angle_step);
        filtered_roll_deg_  = DroneMath::clamp(raw_roll_deg,  filtered_roll_deg_ - max_angle_step,  filtered_roll_deg_ + max_angle_step);

        PositionControlOutput out;
        out.pitch_deg = filtered_pitch_deg_;
        out.roll_deg  = filtered_roll_deg_;
        out.is_hover  = (!is_driving_x && !is_driving_y && mode_x_ == AxisMode::HOLD && mode_y_ == AxisMode::HOLD);
        out.is_braking = (mode_x_ == AxisMode::BRAKING || mode_y_ == AxisMode::BRAKING);
        out.anchor_x = anchor_x_;
        out.anchor_y = anchor_y_;
        return out;
    }

private:
    float anchor_x_{0.0f}; float anchor_y_{0.0f};
    bool has_anchor_{false};
    AxisMode mode_x_{AxisMode::HOLD}; AxisMode mode_y_{AxisMode::HOLD};
    float i_wind_xw_{0.0f}; float i_wind_yw_{0.0f};
    float smoothed_cmd_vx_{0.0f}; float smoothed_cmd_vy_{0.0f};
    float filtered_pitch_deg_{0.0f}; float filtered_roll_deg_{0.0f};
    bool was_driving_x_{false}; bool was_driving_y_{false};
    float brake_timer_x_{0.0f}; float brake_timer_y_{0.0f};
};

// ============================================================================
// 6. BỘ TRỘN ĐỘNG CƠ (MOTOR MIXER)
//    Tham số đọc từ namespace FlightConfig::Mixer
// ============================================================================
class MotorMixer {
public:
    explicit MotorMixer(float max_rot_velocity = Mixer::MAX_MOTOR_ROT_VELOCITY) 
        : max_rot_velocity_(max_rot_velocity) {
        current_speeds_.fill(0.0f);
    }

    void reset() {
        current_speeds_.fill(0.0f);
    }

    std::array<float, 4> compute_motor_speeds(float thrust, float roll, float pitch, float yaw, bool is_armed = true) const {
        std::array<float, 4> speeds{0.0f, 0.0f, 0.0f, 0.0f};
        if (!is_armed || thrust <= Mixer::MIN_THRUST_THRESHOLD) return speeds;

        float u0 = thrust - roll - pitch - yaw; // Front-Right (CCW)
        float u1 = thrust + roll + pitch - yaw; // Rear-Left   (CCW)
        float u2 = thrust + roll - pitch + yaw; // Front-Left  (CW)
        float u3 = thrust - roll + pitch + yaw; // Rear-Right  (CW)

        std::array<float, 4> u = {u0, u1, u2, u3};
        float max_u = *std::max_element(u.begin(), u.end());
        if (max_u > 1.0f) {
            float excess = max_u - 1.0f;
            for (size_t i = 0; i < 4; ++i) u[i] -= excess;
        }
        float min_u = *std::min_element(u.begin(), u.end());
        if (min_u < 0.0f) {
            float deficit = -min_u;
            for (size_t i = 0; i < 4; ++i) u[i] += deficit;
        }

        for (size_t i = 0; i < 4; ++i) {
            float clamped_u = DroneMath::clamp(u[i], 0.0f, 1.0f);
            speeds[i] = max_rot_velocity_ * std::sqrt(clamped_u);
        }
        return speeds;
    }

    // Bộ lọc gia tốc động cơ Slew Rate Limiter (tăng giảm mượt mà, chống giật vọt)
    std::array<float, 4> apply_slew_rate(const std::array<float, 4>& target_speeds, float dt = Timing::DT) {
        const float max_delta = Mixer::MAX_MOTOR_SLEW_RATE * dt;
        for (size_t i = 0; i < 4; ++i) {
            float diff = target_speeds[i] - current_speeds_[i];
            diff = DroneMath::clamp(diff, -max_delta, max_delta);
            current_speeds_[i] += diff;
        }
        return current_speeds_;
    }

private:
    float max_rot_velocity_{Mixer::MAX_MOTOR_ROT_VELOCITY};
    std::array<float, 4> current_speeds_{0.0f, 0.0f, 0.0f, 0.0f};
};

// ============================================================================
// 7. KHỞI TẠO ĐỐI TƯỢNG TOÀN CỤC & TÁC VỤ BAY
// ============================================================================
static MotorMixer g_mixer(Mixer::MAX_MOTOR_ROT_VELOCITY);
static AltitudeController g_alt_controller;
static PositionController g_pos_controller;

// Khởi tạo các bộ điều khiển ITSMC (Integral Terminal Sliding Mode Control)
static DroneControl::ITSMCController g_itsmc_roll(
    ITSMC::ATT_K11, ITSMC::ATT_K12, ITSMC::ATT_K2, ITSMC::ATT_K3,
    ITSMC::ATT_B_INV, ITSMC::ATT_PHI_B, ITSMC::ATT_EPS_LIN,
    ITSMC::ALPHA, ITSMC::ATT_MAX_INTEGRAL, Attitude::MAX_TAU_ROLL
);

static DroneControl::ITSMCController g_itsmc_pitch(
    ITSMC::ATT_K11, ITSMC::ATT_K12, ITSMC::ATT_K2, ITSMC::ATT_K3,
    ITSMC::ATT_B_INV, ITSMC::ATT_PHI_B, ITSMC::ATT_EPS_LIN,
    ITSMC::ALPHA, ITSMC::ATT_MAX_INTEGRAL, Attitude::MAX_TAU_PITCH
);

static DroneControl::ITSMCController g_itsmc_yaw(
    ITSMC::YAW_K11, ITSMC::YAW_K12, ITSMC::YAW_K2, ITSMC::YAW_K3,
    ITSMC::YAW_B_INV, ITSMC::YAW_PHI_B, ITSMC::YAW_EPS_LIN,
    ITSMC::ALPHA, ITSMC::YAW_MAX_INTEGRAL, Attitude::MAX_TAU_YAW
);

static DroneControl::ITSMCAltitudeController g_itsmc_alt(
    Altitude::HOVER_THRUST,
    ITSMC::ALT_K11, ITSMC::ALT_K12, ITSMC::ALT_K2, ITSMC::ALT_K3,
    ITSMC::ALT_B_INV,
    Altitude::MIN_TOTAL_THRUST, Altitude::MAX_TOTAL_THRUST,
    ITSMC::ALT_PHI_B, ITSMC::ALT_EPS_LIN
);

static portMUX_TYPE g_data_mutex = portMUX_INITIALIZER_UNLOCKED;
static SensorPacket g_sensor_data;
static volatile bool g_connected = false;
static volatile uint32_t g_last_packet_time_ms = 0;

enum class DroneFlightState {
    DISARMED,      // Động cơ tắt (0 rad/s)
    GROUND_IDLE,   // Đã ARM nhưng đang ở dưới đất: Cánh quạt quay nhẹ cầm chừng (200 rad/s), drone nằm im
    FLYING         // Đang bay trên không: Đẩy cần ga Z để bay lên, giữ thăng bằng và định vị
};

static DroneFlightState g_flight_state = DroneFlightState::DISARMED;
static uint32_t g_land_stick_held_start_ms = 0;

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
        // Fail-safe: Ngắt khẩn cấp nếu mất tín hiệu từ máy tính
        if (now_ms - g_last_packet_time_ms > Hardware::FAILSAFE_TIMEOUT_MS) {
            g_connected = false;
            g_is_armed = false;
            g_flight_state = DroneFlightState::DISARMED;
            digitalWrite(Hardware::STATUS_LED_PIN, LOW); // Tắt đèn LED báo hiệu khi ngắt kết nối
        }

        if (!g_connected) {
            digitalWrite(Hardware::STATUS_LED_PIN, LOW); // Đảm bảo LED luôn tắt khi chưa kết nối
            ActuatorPacket safe_pkt;
            memset(&safe_pkt, 0, sizeof(safe_pkt));
            safe_pkt.header[0] = 0x55; safe_pkt.header[1] = 0xAA;
            safe_pkt.checksum = compute_packet_checksum((uint8_t*)&safe_pkt + 2, sizeof(safe_pkt) - 3);
            Serial.write((uint8_t*)&safe_pkt, sizeof(safe_pkt));
            continue;
        }

        SensorPacket s;
        portENTER_CRITICAL(&g_data_mutex);
        s = g_sensor_data;
        portEXIT_CRITICAL(&g_data_mutex);

        // Khởi tạo điểm neo ban đầu khi mới kết nối
        if (!g_init_yaw_done) {
            g_target_yaw_rad = s.yaw;
            g_pos_controller.set_anchor(s.x, s.y);
            g_target_alt = s.z;
            g_init_yaw_done = true;
        }

        // 1. Quản lý cờ ARM / DISARM
        bool cmd_armed = (s.flags & CTRL_FLAG_ARMED);
        if (s.flags & CTRL_FLAG_EMERGENCY) {
            cmd_armed = false;
        }

        if (!cmd_armed) {
            if (g_flight_state != DroneFlightState::DISARMED) {
                g_flight_state = DroneFlightState::DISARMED;
            }
            g_is_armed = false;
        } else {
            g_is_armed = true;
            if (g_flight_state == DroneFlightState::DISARMED) {
                // Vừa được ARM: Chuyển sang trạng thái quay không tải dưới đất (GROUND_IDLE)
                g_flight_state = DroneFlightState::GROUND_IDLE;
                g_target_alt = s.z;
                g_pos_controller.set_anchor(s.x, s.y);
            }
        }

        // Khi ở DISARMED: Reset bộ điều khiển và khâu tích phân
        if (g_flight_state == DroneFlightState::DISARMED) {
            g_int_roll = 0.0f;
            g_int_pitch = 0.0f;
            g_itsmc_roll.reset();
            g_itsmc_pitch.reset();
            g_itsmc_yaw.reset();
            g_itsmc_alt.reset();
            g_land_stick_held_start_ms = 0;
            g_target_alt = s.z;
        }

        // 2. Chuyển trạng thái giữa GROUND_IDLE và FLYING
        if (g_flight_state == DroneFlightState::GROUND_IDLE) {
            // Khi đang ở dưới đất:
            // Người lái đẩy cần ga Z lên (cmd_alt_vel > 0.15f) HOẶC có cờ tự động cất cánh TAKEOFF
            if (s.cmd_alt_vel > 0.15f || (s.flags & CTRL_FLAG_TAKEOFF)) {
                g_flight_state = DroneFlightState::FLYING;
                g_target_alt = std::max(s.z + 0.10f, 0.25f);
                g_pos_controller.set_anchor(s.x, s.y);
                g_itsmc_alt.reset();
            }

            // Nếu người lái kéo cần ga Z xuống kịch sàn (cmd_alt_vel < -0.4f) liên tục 1.5s dưới đất -> Tự động Disarm tắt máy!
            if (s.cmd_alt_vel < -0.4f) {
                if (g_land_stick_held_start_ms == 0) g_land_stick_held_start_ms = now_ms;
                else if (now_ms - g_land_stick_held_start_ms > 1500) {
                    g_flight_state = DroneFlightState::DISARMED;
                    g_is_armed = false;
                    g_land_stick_held_start_ms = 0;
                }
            } else {
                g_land_stick_held_start_ms = 0;
            }
        } else if (g_flight_state == DroneFlightState::FLYING) {
            g_land_stick_held_start_ms = 0;

            // Xử lý điều khiển độ cao bằng cần ga
            bool alt_active = (s.flags & CTRL_FLAG_ALT_ACTIVE);
            if (s.flags & CTRL_FLAG_TAKEOFF) {
                // Cất cánh tự động
                g_target_alt = std::min(FlightLogic::TAKEOFF_TARGET_ALT_M, 
                                        g_target_alt + FlightLogic::TAKEOFF_STEP_PER_TICK);
            } else if (s.flags & CTRL_FLAG_LAND) {
                // Hạ cánh tự động
                g_target_alt = std::max(0.05f, g_target_alt - FlightLogic::LAND_STEP_PER_TICK);
            } else if (alt_active) {
                // Lái cần ga Z thủ công
                g_target_alt += s.cmd_alt_vel * Timing::DT;
                g_target_alt = DroneMath::clamp(g_target_alt, 0.05f, FlightLogic::MAX_FLIGHT_ALT_M);
            }

            // Nhận diện tiếp đất (Touchdown Detection):
            // Khi drone đã xuống sát đất (z < 0.12m) VÀ đang kéo ga hạ (cmd_alt_vel < -0.1f hoặc cờ LAND)
            if (s.z < FlightLogic::TOUCHDOWN_ALT_THRESHOLD && (s.cmd_alt_vel < -0.1f || (s.flags & CTRL_FLAG_LAND))) {
                // Đã chạm đất an toàn -> Chuyển về GROUND_IDLE (cánh quạt quay nhẹ 200 rad/s)
                g_flight_state = DroneFlightState::GROUND_IDLE;
                g_target_alt = s.z;
                g_itsmc_alt.reset();
            }
        }

        // 3. Tính toán các bộ điều khiển & Motor Speeds
        PositionControlOutput pos_out;
        std::array<float, 4> raw_speeds{0.0f, 0.0f, 0.0f, 0.0f};

        if (g_flight_state == DroneFlightState::DISARMED) {
            raw_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        } else if (g_flight_state == DroneFlightState::GROUND_IDLE) {
            // Khi ở dưới đất: Cả 4 cánh quạt quay nhẹ đều ở tốc độ cầm chừng 200 rad/s
            // Thân nằm yên vững chắc trên mặt đất, không bị bay vọt hay lật nghiêng
            raw_speeds = {
                Mixer::IDLE_MOTOR_ROT_VELOCITY,
                Mixer::IDLE_MOTOR_ROT_VELOCITY,
                Mixer::IDLE_MOTOR_ROT_VELOCITY,
                Mixer::IDLE_MOTOR_ROT_VELOCITY
            };
        } else {
            // Đang bay (FLYING): Chạy đầy đủ các vòng lặp điều khiển ITSMC / PID
            if (std::abs(s.cmd_yaw_rate) > 0.01f) {
                g_target_yaw_rad = DroneMath::normalize_angle(g_target_yaw_rad + s.cmd_yaw_rate * Timing::DT);
            }

            bool is_driving_x = (s.flags & CTRL_FLAG_DRIVING_X);
            bool is_driving_y = (s.flags & CTRL_FLAG_DRIVING_Y);

            // 3.1. VÒNG NGOÀI: Điều khiển Vị trí X/Y -> Ra góc nghiêng mong muốn (Roll, Pitch)
            pos_out = g_pos_controller.update(
                s.x, s.y, s.vx, s.vy, s.yaw,
                s.cmd_vx, s.cmd_vy, is_driving_x, is_driving_y,
                Position::MAX_TILT_DRIVING_DEG, Timing::DT
            );

            // 3.2. Điều khiển Lực nâng thẳng đứng (Altitude Controller: ITSMC hoặc PID)
            float total_thrust = 0.0f;
            if (ITSMC::ENABLE_ITSMC) {
                total_thrust = g_itsmc_alt.compute_thrust(g_target_alt, s.z, s.vz, Timing::DT);
            } else {
                total_thrust = g_alt_controller.compute_thrust(g_target_alt, s.z, s.vz, Timing::DT);
            }
            float cos_tilt = std::max(Altitude::MIN_COS_TILT, std::cos(s.roll) * std::cos(s.pitch));
            float effective_thrust = total_thrust / cos_tilt;

            float target_r_rad = 0.0f;
            float target_p_rad = 0.0f;
            if (s.z > FlightLogic::MIN_TAKEOFF_ALT_FOR_TILT) {
                target_r_rad = DroneMath::deg2rad(pos_out.roll_deg);
                target_p_rad = DroneMath::deg2rad(pos_out.pitch_deg);
            } else {
                g_pos_controller.set_anchor(s.x, s.y);
            }

            // 3.3. VÒNG TRONG: Điều khiển Góc thái độ (ITSMC hoặc PID)
            float tau_roll  = 0.0f;
            float tau_pitch = 0.0f;
            float tau_yaw   = 0.0f;

            if (ITSMC::ENABLE_ITSMC) {
                tau_roll  = g_itsmc_roll.update(target_r_rad, s.roll, s.p, Timing::DT);
                tau_pitch = g_itsmc_pitch.update(target_p_rad, s.pitch, s.q, Timing::DT);
                float err_yaw = DroneMath::normalize_angle(g_target_yaw_rad - s.yaw);
                tau_yaw   = g_itsmc_yaw.update_error(err_yaw, s.r, Timing::DT);
            } else {
                float err_roll = target_r_rad - s.roll;
                g_int_roll += err_roll * Timing::DT;
                g_int_roll = DroneMath::clamp(g_int_roll, -Attitude::INT_ATT_LIMIT, Attitude::INT_ATT_LIMIT);
                tau_roll = Attitude::KP_ATT * err_roll + Attitude::KI_ATT * g_int_roll - Attitude::KD_ATT * s.p;

                float err_pitch = target_p_rad - s.pitch;
                g_int_pitch += err_pitch * Timing::DT;
                g_int_pitch = DroneMath::clamp(g_int_pitch, -Attitude::INT_ATT_LIMIT, Attitude::INT_ATT_LIMIT);
                tau_pitch = Attitude::KP_ATT * err_pitch + Attitude::KI_ATT * g_int_pitch - Attitude::KD_ATT * s.q;

                float err_yaw = DroneMath::normalize_angle(g_target_yaw_rad - s.yaw);
                tau_yaw = Attitude::KP_YAW * err_yaw - Attitude::KD_YAW * s.r;
            }

            // Giới hạn mô-men xoắn ngõ ra
            tau_roll  = DroneMath::clamp(tau_roll, -Attitude::MAX_TAU_ROLL, Attitude::MAX_TAU_ROLL);
            tau_pitch = DroneMath::clamp(tau_pitch, -Attitude::MAX_TAU_PITCH, Attitude::MAX_TAU_PITCH);
            tau_yaw   = DroneMath::clamp(tau_yaw, -Attitude::MAX_TAU_YAW, Attitude::MAX_TAU_YAW);

            // 3.4. BỘ TRỘN ĐỘNG CƠ: Phân bổ lực cho 4 cánh quạt (Motor Mixer)
            raw_speeds = g_mixer.compute_motor_speeds(effective_thrust, tau_roll, tau_pitch, tau_yaw, true);

            // Đảm bảo khi đang bay, tốc độ không bị rớt dưới IDLE_MOTOR_ROT_VELOCITY
            for (size_t i = 0; i < 4; ++i) {
                raw_speeds[i] = std::max(Mixer::IDLE_MOTOR_ROT_VELOCITY, raw_speeds[i]);
            }
        }

        // 4. BỘ LỌC TỐC ĐỘ GIA TỐC ĐỘNG CƠ (MOTOR SLEW RATE LIMITER)
        // Giúp tốc độ động cơ tăng giảm êm ái mượt mà, không bao giờ bị tăng vọt 1 phát!
        auto speeds = g_mixer.apply_slew_rate(raw_speeds, Timing::DT);

        // Đóng gói dữ liệu gửi ngược về máy tính qua Serial
        ActuatorPacket out_pkt;
        out_pkt.header[0] = 0x55; out_pkt.header[1] = 0xAA;
        out_pkt.w0 = speeds[0]; out_pkt.w1 = speeds[1];
        out_pkt.w2 = speeds[2]; out_pkt.w3 = speeds[3];
        out_pkt.target_pitch_deg = (g_flight_state == DroneFlightState::FLYING) ? pos_out.pitch_deg : 0.0f;
        out_pkt.target_roll_deg  = (g_flight_state == DroneFlightState::FLYING) ? pos_out.roll_deg : 0.0f;
        out_pkt.status_flags = 0;
        if (g_flight_state != DroneFlightState::DISARMED) out_pkt.status_flags |= 1;
        if (pos_out.is_braking) out_pkt.status_flags |= 2;
        if (pos_out.is_hover) out_pkt.status_flags |= 4;
        out_pkt.checksum = compute_packet_checksum((uint8_t*)&out_pkt + 2, sizeof(out_pkt) - 3);

        Serial.write((uint8_t*)&out_pkt, sizeof(out_pkt));
    }
}


// ============================================================================
// TASK GCS MAVLINK TELEMETRY (FreeRTOS Core 0 / Low Priority Task)
// Chuyên xử lý truyền nhận MAVLink qua WiFi UDP mà không làm chậm vòng lặp Serial
// ============================================================================
void gcs_wifi_task(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        if (WiFi.status() == WL_CONNECTED && g_mavlink.is_initialized()) {
            SensorPacket s_copy;
            portENTER_CRITICAL(&g_data_mutex);
            s_copy = g_sensor_data;
            portEXIT_CRITICAL(&g_data_mutex);

            bool qgc_arm = false;
            bool qgc_disarm = false;
            bool qgc_takeoff = false;
            bool qgc_land = false;

            g_mavlink.update(s_copy, g_is_armed, g_target_alt,
                             qgc_arm, qgc_disarm, qgc_takeoff, qgc_land);

            if (qgc_arm) {
                g_is_armed = true;
            }
            if (qgc_disarm) {
                g_is_armed = false;
            }
            if (qgc_takeoff) {
                g_is_armed = true;
                g_target_alt = FlightLogic::TAKEOFF_TARGET_ALT_M;
            }
            if (qgc_land) {
                g_target_alt = 0.05f;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Chu kỳ 50Hz, tự động nhường CPU cho Serial
    }
}

void setup() {
    pinMode(Hardware::STATUS_LED_PIN, OUTPUT);
    digitalWrite(Hardware::STATUS_LED_PIN, LOW);

    // Tăng bộ đệm phần cứng RX lên 2048 bytes để chống tràn khi truyền 100Hz-200Hz
    Serial.setRxBufferSize(Hardware::UART_RX_BUFFER_SIZE);
    Serial.begin(Hardware::UART_BAUD_RATE);
    while (!Serial && millis() < 1500);

    Serial.println(F("\n========================================================"));
    Serial.println(F("       DRONE HITL FLIGHT CONTROLLER - ESP32"));
    Serial.println(F("========================================================"));

    // Khởi tạo Wi-Fi Station (Client) kết nối vào router mạng nội bộ (như VietQR app)
    if (FlightConfig::GCS::ENABLE_WIFI_MAVLINK) {
        WiFi.mode(WIFI_STA);
        WifiCredentialStorage credStorage;
        WifiScannerService scanner;
        bool wifi_connected = false;

        if (credStorage.hasStoredCredentials()) {
            String saved_ssid = credStorage.getStoredSsid();
            String saved_pass = credStorage.getStoredPassword();
            Serial.printf("[WIFI STA] Tim thay mang da luu: '%s'. Dang ket noi...\n", saved_ssid.c_str());
            Serial.println(F(">> Meo: Go 'c' roi an Enter trong 3s neu muon doi mang khac."));

            uint32_t wait_start = millis();
            bool user_override = false;
            while (millis() - wait_start < 3000) {
                if (Serial.available() > 0) {
                    char ch = (char)Serial.read();
                    if (ch == 'c' || ch == 'C') {
                        user_override = true;
                        break;
                    }
                }
                delay(20);
            }

            if (!user_override) {
                WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
                uint32_t conn_start = millis();
                while (WiFi.status() != WL_CONNECTED && (millis() - conn_start < 7000)) {
                    delay(300);
                    Serial.print('.');
                }
                Serial.println();
                if (WiFi.status() == WL_CONNECTED) {
                    wifi_connected = true;
                    Serial.printf("[WIFI STA] Ket noi thanh cong! IP Drone: %s\n", WiFi.localIP().toString().c_str());
                } else {
                    Serial.println(F("[WIFI STA] Ket noi mang cu that bai hoac het thoi gian."));
                }
            } else {
                Serial.println(F("\n[WIFI] Nguoi dung yeu cau cau hinh mang moi!"));
            }
        }

        // Nếu chưa lưu mạng hoặc kết nối thất bại: Quét mạng và cho chọn qua Serial
        if (!wifi_connected) {
            Serial.println(F("[WIFI] Bat dau quet mang 2.4GHz de cau hinh ket noi..."));
            String chosenSsid = "";
            String enteredPass = "";
            if (WifiSerialConfigurator::promptUserForNetwork(Serial, scanner, chosenSsid, enteredPass, 12000)) {
                Serial.printf("[WIFI STA] Dang ket noi toi '%s'...\n", chosenSsid.c_str());
                WiFi.begin(chosenSsid.c_str(), enteredPass.c_str());
                uint32_t conn_start = millis();
                while (WiFi.status() != WL_CONNECTED && (millis() - conn_start < 8000)) {
                    delay(300);
                    Serial.print('.');
                }
                Serial.println();
                if (WiFi.status() == WL_CONNECTED) {
                    wifi_connected = true;
                    credStorage.saveCredentials(chosenSsid, enteredPass);
                    Serial.printf("[WIFI STA] Ket noi thanh cong & da luu vao Flash! IP: %s\n", WiFi.localIP().toString().c_str());
                } else {
                    Serial.println(F("[WIFI STA] Khong the ket noi toi mang da chon."));
                }
            } else {
                Serial.println(F("[WIFI] Bo qua cau hinh WiFi. Chay che do Offline."));
            }
        }

        if (wifi_connected) {
            g_mavlink.init(FlightConfig::GCS::UDP_PORT);
#if CONFIG_FREERTOS_UNICORE || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S2)
            xTaskCreate(gcs_wifi_task, "GCS_WiFi", 4096, NULL, 1, NULL);
#else
            xTaskCreatePinnedToCore(gcs_wifi_task, "GCS_WiFi", 4096, NULL, 1, NULL, 0);
#endif
        }
    }

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

                    // Nhấp nháy LED pin 2 để quan sát trực quan
                    digitalWrite(Hardware::STATUS_LED_PIN, (millis() / 200) % 2);

                    // Xóa gói hợp lệ khỏi buffer
                    memmove(rx_buf, rx_buf + sizeof(SensorPacket), rx_len - sizeof(SensorPacket));
                    rx_len -= sizeof(SensorPacket);
                } else {
                    // Checksum sai: dịch 1 byte để tìm lại đồng bộ
                    memmove(rx_buf, rx_buf + 1, rx_len - 1);
                    rx_len--;
                }
            } else {
                // Header không khớp: dịch 1 byte
                memmove(rx_buf, rx_buf + 1, rx_len - 1);
                rx_len--;
            }
        }
    }

    // Nếu mất tín hiệu từ máy tính (> 500ms), tự động tắt đèn LED trạng thái
    if (!g_connected || (millis() - g_last_packet_time_ms > Hardware::FAILSAFE_TIMEOUT_MS)) {
        digitalWrite(Hardware::STATUS_LED_PIN, LOW);
    }
}
