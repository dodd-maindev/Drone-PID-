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

using namespace FlightConfig;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// 1. ĐỊNH NGHĨA GÓI TIN NHỊ PHÂN (TELEMETRY PACKET)
// ============================================================================
#pragma pack(push, 1)

enum ControlFlagBits : uint8_t {
    CTRL_FLAG_ARMED       = 1 << 0,
    CTRL_FLAG_DRIVING_X   = 1 << 1,
    CTRL_FLAG_DRIVING_Y   = 1 << 2,
    CTRL_FLAG_ALT_ACTIVE  = 1 << 3,
    CTRL_FLAG_EMERGENCY   = 1 << 4,
    CTRL_FLAG_TAKEOFF     = 1 << 5,
    CTRL_FLAG_LAND        = 1 << 6
};

struct SensorPacket {
    uint8_t header[2];      // 0xAA, 0x55
    float x, y, z;          // Vị trí (m)
    float vx, vy, vz;       // Vận tốc (m/s)
    float roll, pitch, yaw; // Góc Euler (rad)
    float p, q, r;          // Vận tốc góc (rad/s)
    float cmd_vx;           // Lệnh vận tốc X (m/s)
    float cmd_vy;           // Lệnh vận tốc Y (m/s)
    float cmd_alt_vel;      // Lệnh leo/hạ Z (m/s)
    float cmd_yaw_rate;     // Lệnh xoay Yaw (rad/s)
    uint8_t flags;          // Cờ điều khiển
    uint8_t checksum;       // Mã kiểm lỗi XOR
};

struct ActuatorPacket {
    uint8_t header[2];      // 0x55, 0xAA
    float w0, w1, w2, w3;   // Tốc độ 4 cánh quạt (rad/s)
    float target_pitch_deg; // Góc Pitch mục tiêu
    float target_roll_deg;  // Góc Roll mục tiêu
    uint8_t status_flags;   // Cờ trạng thái (Bit 0: Armed, Bit 1: Braking, Bit 2: Hover)
    uint8_t checksum;       // Mã kiểm lỗi XOR
};

#pragma pack(pop)

inline uint8_t compute_packet_checksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; ++i) cs ^= data[i];
    return cs;
}

// ============================================================================
// 2. TOÁN HỌC HÀNG KHÔNG (MATH UTILS)
// ============================================================================
namespace DroneMath {
    inline float normalize_angle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
    inline float deg2rad(float deg) { return deg * static_cast<float>(M_PI / 180.0); }
    inline float rad2deg(float rad) { return rad * static_cast<float>(180.0 / M_PI); }
    template<typename T>
    inline T clamp(T val, T min_val, T max_val) {
        return std::max(min_val, std::min(val, max_val));
    }
}

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
    }

    void set_anchor(float x, float y) {
        anchor_x_ = x; anchor_y_ = y;
        has_anchor_ = true;
        mode_x_ = AxisMode::HOLD; mode_y_ = AxisMode::HOLD;
        smoothed_cmd_vx_ = 0.0f; smoothed_cmd_vy_ = 0.0f;
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

        // Trục X (Tiến / Lùi)
        if (is_driving_x) {
            mode_x_ = AxisMode::DRIVING;
            anchor_x_ = current_x;
            target_vx_b = smoothed_cmd_vx_;
        } else {
            if (mode_x_ == AxisMode::DRIVING) mode_x_ = AxisMode::BRAKING;
            if (mode_x_ == AxisMode::BRAKING) {
                anchor_x_ = current_x;
                target_vx_b = 0.0f;
                if (std::abs(vx_b) < Position::STOP_VEL_THRESHOLD) {
                    mode_x_ = AxisMode::HOLD;
                    anchor_x_ = current_x;
                }
            } else {
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_xb  = cy * err_x_w + sy * err_y_w;
                target_vx_b = DroneMath::clamp(Position::KP_POS * err_xb, -Position::MAX_APPROACH_VEL, Position::MAX_APPROACH_VEL);
            }
        }

        // Trục Y (Trái / Phải)
        if (is_driving_y) {
            mode_y_ = AxisMode::DRIVING;
            anchor_y_ = current_y;
            target_vy_b = smoothed_cmd_vy_;
        } else {
            if (mode_y_ == AxisMode::DRIVING) mode_y_ = AxisMode::BRAKING;
            if (mode_y_ == AxisMode::BRAKING) {
                anchor_y_ = current_y;
                target_vy_b = 0.0f;
                if (std::abs(vy_b) < Position::STOP_VEL_THRESHOLD) {
                    mode_y_ = AxisMode::HOLD;
                    anchor_y_ = current_y;
                }
            } else {
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_yb  = -sy * err_x_w + cy * err_y_w;
                target_vy_b = DroneMath::clamp(Position::KP_POS * err_yb, -Position::MAX_APPROACH_VEL, Position::MAX_APPROACH_VEL);
            }
        }

        float evx = target_vx_b - vx_b;
        float evy = target_vy_b - vy_b;

        const float int_limit = Position::MAX_WIND_ACCEL / Position::KI_VEL;

        if (mode_x_ == AxisMode::HOLD && mode_y_ == AxisMode::HOLD &&
            std::abs(vx_b) < 0.15f && std::abs(vy_b) < 0.15f) {
            float evx_w = cy * evx - sy * evy;
            float evy_w = sy * evx + cy * evy;
            i_wind_xw_ += evx_w * dt;
            i_wind_xw_ = DroneMath::clamp(i_wind_xw_, -int_limit, int_limit);
            i_wind_yw_ += evy_w * dt;
            i_wind_yw_ = DroneMath::clamp(i_wind_yw_, -int_limit, int_limit);
        }

        float wind_accel_xb =  cy * (Position::KI_VEL * i_wind_xw_) + sy * (Position::KI_VEL * i_wind_yw_);
        float wind_accel_yb = -sy * (Position::KI_VEL * i_wind_xw_) + cy * (Position::KI_VEL * i_wind_yw_);

        float a_sp_x = Position::KP_VEL * evx + wind_accel_xb;
        float a_sp_y = Position::KP_VEL * evy + wind_accel_yb;

        const float g = 9.80665f;
        float raw_pitch_deg = DroneMath::rad2deg(std::atan2(a_sp_x, g));
        float raw_roll_deg  = DroneMath::rad2deg(std::atan2(-a_sp_y, g));

        float max_p = (mode_x_ == AxisMode::DRIVING) ? max_tilt_deg : Position::MAX_TILT_BRAKING_DEG;
        float max_r = (mode_y_ == AxisMode::DRIVING) ? max_tilt_deg : Position::MAX_TILT_BRAKING_DEG;

        raw_pitch_deg = DroneMath::clamp(raw_pitch_deg, -max_p, max_p);
        raw_roll_deg  = DroneMath::clamp(raw_roll_deg,  -max_r, max_r);

        float max_angle_step = Position::MAX_TILT_RATE_DEG_S * dt;

        filtered_pitch_deg_ = DroneMath::clamp(raw_pitch_deg, filtered_pitch_deg_ - max_angle_step, filtered_pitch_deg_ + max_angle_step);
        filtered_roll_deg_  = DroneMath::clamp(raw_roll_deg,  filtered_roll_deg_ - max_angle_step,  filtered_roll_deg_ + max_angle_step);

        PositionControlOutput out;
        out.pitch_deg = filtered_pitch_deg_;
        out.roll_deg  = filtered_roll_deg_;
        out.is_hover  = (mode_x_ == AxisMode::HOLD && mode_y_ == AxisMode::HOLD);
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
};

// ============================================================================
// 6. BỘ TRỘN ĐỘNG CƠ (MOTOR MIXER)
//    Tham số đọc từ namespace FlightConfig::Mixer
// ============================================================================
class MotorMixer {
public:
    explicit MotorMixer(float max_rot_velocity = Mixer::MAX_MOTOR_ROT_VELOCITY) 
        : max_rot_velocity_(max_rot_velocity) {}

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

private:
    float max_rot_velocity_{Mixer::MAX_MOTOR_ROT_VELOCITY};
};

// ============================================================================
// 7. KHỞI TẠO ĐỐI TƯỢNG TOÀN CỤC & TÁC VỤ BAY
// ============================================================================
static MotorMixer g_mixer(Mixer::MAX_MOTOR_ROT_VELOCITY);
static AltitudeController g_alt_controller;
static PositionController g_pos_controller;

static portMUX_TYPE g_data_mutex = portMUX_INITIALIZER_UNLOCKED;
static SensorPacket g_sensor_data;
static volatile bool g_connected = false;
static volatile uint32_t g_last_packet_time_ms = 0;

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
        }

        if (!g_connected) {
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
            g_target_alt = std::max(0.2f, s.z);
            g_init_yaw_done = true;
        }

        g_is_armed = (s.flags & CTRL_FLAG_ARMED);
        if (s.flags & CTRL_FLAG_EMERGENCY) g_is_armed = false;

        // Cất cánh tự động: tăng dần độ cao theo chu kỳ
        if (s.flags & CTRL_FLAG_TAKEOFF) {
            g_target_alt = std::min(FlightLogic::TAKEOFF_TARGET_ALT_M, 
                                    g_target_alt + FlightLogic::TAKEOFF_STEP_PER_TICK);
            g_is_armed = true;
        }

        // Hạ cánh tự động: giảm dần độ cao, chạm đất thì ngắt động cơ
        if (s.flags & CTRL_FLAG_LAND) {
            g_target_alt = std::max(0.05f, g_target_alt - FlightLogic::LAND_STEP_PER_TICK);
            if (s.z < FlightLogic::TOUCHDOWN_ALT_THRESHOLD) g_is_armed = false;
        }

        // Cập nhật góc Yaw đặt khi người dùng xoay cần lái
        if (std::abs(s.cmd_yaw_rate) > 0.01f) {
            g_target_yaw_rad = DroneMath::normalize_angle(g_target_yaw_rad + s.cmd_yaw_rate * Timing::DT);
        }

        // Điều khiển độ cao Z thủ công
        bool alt_active = (s.flags & CTRL_FLAG_ALT_ACTIVE);
        if (alt_active) {
            g_target_alt += s.cmd_alt_vel * Timing::DT;
            g_target_alt = DroneMath::clamp(g_target_alt, FlightLogic::MIN_FLIGHT_ALT_M, FlightLogic::MAX_FLIGHT_ALT_M);
        } else {
            if (g_alt_was_active) g_target_alt = s.z;
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
        // Khi sát đất (< MIN_TAKEOFF_ALT_FOR_TILT), khóa phẳng Roll=0, Pitch=0 để 4 chân rời đất an toàn
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

        // Đóng gói dữ liệu gửi ngược về máy tính qua Serial
        ActuatorPacket out_pkt;
        out_pkt.header[0] = 0x55; out_pkt.header[1] = 0xAA;
        out_pkt.w0 = speeds[0]; out_pkt.w1 = speeds[1];
        out_pkt.w2 = speeds[2]; out_pkt.w3 = speeds[3];
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

    // Tăng bộ đệm phần cứng RX lên 2048 bytes để chống tràn khi truyền 100Hz-200Hz
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
}
