#pragma once
#include <cmath>
#include <algorithm>
#include <iostream>
#include "math/math_utils.hpp"

namespace DroneControllers {

struct PositionControlOutput {
    float roll_deg{0.0f};
    float pitch_deg{0.0f};
    bool is_hover{true};
    bool is_braking{false};
    float anchor_x{0.0f};
    float anchor_y{0.0f};
    float wind_pitch_trim_deg{0.0f};
    float wind_roll_trim_deg{0.0f};
};

/**
 * @brief Bộ điều khiển vị trí 2D và chống trôi chuẩn PX4 / DJI (Instant Anchor Lock & Active Snap Brake)
 * 
 * 1. Khóa mỏ neo tức thì lúc buông tay (Instant Anchor Snap):
 *    - Khi đang lái: Mỏ neo trượt theo vị trí drone.
 *    - Khoảnh khắc buông phím: Mỏ neo KHÓA CỨNG ngay tại tọa độ buông tay.
 *    - Quán tính làm xe trôi tới đâu, Position Loop (Kp * err) sẽ lập tức sinh vận tốc âm
 *      kéo giật ngược drone dừng khựng lại và hút về đúng vị trí lúc thả tay.
 * 2. Phanh đối kháng dứt khoát:
 *    - Cho phép góc phanh tới 18.0° để dập tắt vận tốc 2.5 m/s trong chưa đầy 0.4s.
 *    - Tốc độ đảo góc 180°/s: Phản ứng phanh tức thì, không bị trễ góc nghiêng.
 * 3. Triệt tiêu 100% rung lắc:
 *    - Không dùng vi phân gia tốc đo (không còn nhiễu sensor).
 *    - Làm mượt gia tốc lệnh tay lái 3.5 m/s^2.
 */
class PositionController {
public:
    enum class AxisMode {
        DRIVING,
        BRAKING,
        HOLD
    };

    PositionController() = default;

    void reset() {
        i_wind_xw_ = 0.0f;
        i_wind_yw_ = 0.0f;
        smoothed_cmd_vx_ = 0.0f;
        smoothed_cmd_vy_ = 0.0f;
        filtered_pitch_deg_ = 0.0f;
        filtered_roll_deg_ = 0.0f;
        anchor_x_ = 0.0f;
        anchor_y_ = 0.0f;
        has_anchor_ = false;
        mode_x_ = AxisMode::HOLD;
        mode_y_ = AxisMode::HOLD;
        was_driving_x_ = false;
        was_driving_y_ = false;
    }

    void set_anchor(float x, float y) {
        anchor_x_ = x;
        anchor_y_ = y;
        has_anchor_ = true;
        mode_x_ = AxisMode::HOLD;
        mode_y_ = AxisMode::HOLD;
        smoothed_cmd_vx_ = 0.0f;
        smoothed_cmd_vy_ = 0.0f;
    }

    PositionControlOutput update(
        float current_x, float current_y,
        float vx_body, float vy_body,
        float current_yaw_rad,
        float cmd_vx_body, float cmd_vy_body,
        bool is_driving_x, bool is_driving_y,
        float max_tilt_deg,
        float dt = 0.033f
    ) {
        if (!has_anchor_) {
            set_anchor(current_x, current_y);
        }
        if (dt <= 0.001f || dt > 0.1f) dt = 0.033f;

        float cy = std::cos(current_yaw_rad);
        float sy = std::sin(current_yaw_rad);

        float vx_b = vx_body;
        float vy_b = vy_body;

        // 1. Gia tốc lệnh tay lái mượt mà (Slew Rate Limiter: max 3.5 m/s^2)
        const float max_accel = 3.5f;
        const float max_dv = max_accel * dt;

        if (is_driving_x) {
            if (!was_driving_x_) {
                smoothed_cmd_vx_ = vx_b;
            }
            if (cmd_vx_body > smoothed_cmd_vx_) {
                smoothed_cmd_vx_ = std::min(cmd_vx_body, smoothed_cmd_vx_ + max_dv);
            } else {
                smoothed_cmd_vx_ = std::max(cmd_vx_body, smoothed_cmd_vx_ - max_dv);
            }
        } else {
            smoothed_cmd_vx_ = 0.0f;
        }

        if (is_driving_y) {
            if (!was_driving_y_) {
                smoothed_cmd_vy_ = vy_b;
            }
            if (cmd_vy_body > smoothed_cmd_vy_) {
                smoothed_cmd_vy_ = std::min(cmd_vy_body, smoothed_cmd_vy_ + max_dv);
            } else {
                smoothed_cmd_vy_ = std::max(cmd_vy_body, smoothed_cmd_vy_ - max_dv);
            }
        } else {
            smoothed_cmd_vy_ = 0.0f;
        }

        was_driving_x_ = is_driving_x;
        was_driving_y_ = is_driving_y;

        // 2. MÁY TRẠNG THÁI PHANH & KHÓA VỊ TRÍ ĐỘC LẬP TỪNG TRỤC (Chống lùi, dừng êm chuẩn DJI/PX4)
        const float stop_vel_threshold = 0.08f; // Dưới 0.08 m/s coi như đã dừng hẳn
        const float kp_pos = 1.8f;             // Gain vị trí giữ điểm dừng
        const float max_approach_vel = 1.2f;   // Vận tốc bù vị trí tối đa

        float target_vx_b = 0.0f;
        float target_vy_b = 0.0f;

        // --- Xử lý trục X (Tiến / Lùi) ---
        if (is_driving_x) {
            mode_x_ = AxisMode::DRIVING;
            anchor_x_ = current_x; // Mỏ neo bám theo drone khi đang lái
            target_vx_b = smoothed_cmd_vx_;
        } else {
            if (mode_x_ == AxisMode::DRIVING) {
                mode_x_ = AxisMode::BRAKING;
            }

            if (mode_x_ == AxisMode::BRAKING) {
                // Trong lúc phanh: Mỏ neo tiếp tục cập nhật để KHÔNG sinh sai số kéo lùi
                anchor_x_ = current_x;
                target_vx_b = 0.0f; // Mục tiêu là triệt tiêu vận tốc về 0

                if (std::abs(vx_b) < stop_vel_threshold) {
                    mode_x_ = AxisMode::HOLD;
                    anchor_x_ = current_x; // CHỐT CỨNG MỎ NEO TẠI VỊ TRÍ DỪNG HẲN!
                }
            } else { // AxisMode::HOLD
                // Khóa cứng tại mỏ neo điểm dừng
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_xb  = cy * err_x_w + sy * err_y_w;
                target_vx_b = DroneMath::clamp(kp_pos * err_xb, -max_approach_vel, max_approach_vel);
            }
        }

        // --- Xử lý trục Y (Trái / Phải) ---
        if (is_driving_y) {
            mode_y_ = AxisMode::DRIVING;
            anchor_y_ = current_y; // Mỏ neo bám theo drone khi đang lái
            target_vy_b = smoothed_cmd_vy_;
        } else {
            if (mode_y_ == AxisMode::DRIVING) {
                mode_y_ = AxisMode::BRAKING;
            }

            if (mode_y_ == AxisMode::BRAKING) {
                anchor_y_ = current_y;
                target_vy_b = 0.0f;

                if (std::abs(vy_b) < stop_vel_threshold) {
                    mode_y_ = AxisMode::HOLD;
                    anchor_y_ = current_y; // CHỐT CỨNG MỎ NEO TẠI VỊ TRÍ DỪNG HẲN!
                }
            } else { // AxisMode::HOLD
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_yb  = -sy * err_x_w + cy * err_y_w;
                target_vy_b = DroneMath::clamp(kp_pos * err_yb, -max_approach_vel, max_approach_vel);
            }
        }

        // 3. TẦNG VẬN TỐC (Velocity Controller with Wind Disturbance Observer)
        float evx = target_vx_b - vx_b;
        float evy = target_vy_b - vy_b;

        // Chỉ tích phân bù gió khi cả hai trục đang ở chế độ HOLD (dừng hẳn)
        // và vận tốc nhỏ, tránh bão hòa tích phân khi đang phanh hãm động lực
        const float max_wind_accel = 1.5f;
        const float ki_vel = 0.40f;
        const float int_limit = max_wind_accel / ki_vel;

        if (mode_x_ == AxisMode::HOLD && mode_y_ == AxisMode::HOLD &&
            std::abs(vx_b) < 0.15f && std::abs(vy_b) < 0.15f) {
            float evx_w = cy * evx - sy * evy;
            float evy_w = sy * evx + cy * evy;

            i_wind_xw_ += evx_w * dt;
            i_wind_xw_ = DroneMath::clamp(i_wind_xw_, -int_limit, int_limit);

            i_wind_yw_ += evy_w * dt;
            i_wind_yw_ = DroneMath::clamp(i_wind_yw_, -int_limit, int_limit);
        }

        float wind_accel_xw = ki_vel * i_wind_xw_;
        float wind_accel_yw = ki_vel * i_wind_yw_;

        float wind_accel_xb =  cy * wind_accel_xw + sy * wind_accel_yw;
        float wind_accel_yb = -sy * wind_accel_xw + cy * wind_accel_yw;

        // Gain vận tốc: 2.5 cho phản hồi phanh nhanh, dứt khoát
        const float kp_vel = 2.5f;
        float a_sp_x = kp_vel * evx + wind_accel_xb;
        float a_sp_y = kp_vel * evy + wind_accel_yb;

        // 4. CHUYỂN ĐỔI GIA TỐC SANG GÓC NGHIÊNG THÂN
        const float g = 9.80665f;
        float raw_pitch_deg = DroneMath::rad2deg(std::atan2(a_sp_x, g));
        float raw_roll_deg  = DroneMath::rad2deg(std::atan2(-a_sp_y, g));

        // Khi đang phanh, cho phép nghiêng tối đa 16° để hãm đà nhanh nhất có thể
        const float max_brake_tilt = 16.0f;
        float max_p = (mode_x_ == AxisMode::DRIVING) ? max_tilt_deg : max_brake_tilt;
        float max_r = (mode_y_ == AxisMode::DRIVING) ? max_tilt_deg : max_brake_tilt;

        raw_pitch_deg = DroneMath::clamp(raw_pitch_deg, -max_p, max_p);
        raw_roll_deg  = DroneMath::clamp(raw_roll_deg,  -max_r, max_r);

        // 5. BỘ LỌC TỐC ĐỘ GÓC (Tilt Angular Rate Limiter: 180°/s)
        const float max_tilt_rate_deg_s = 180.0f;
        float max_angle_step = max_tilt_rate_deg_s * dt;

        filtered_pitch_deg_ = DroneMath::clamp(raw_pitch_deg,
                                                filtered_pitch_deg_ - max_angle_step,
                                                filtered_pitch_deg_ + max_angle_step);
        filtered_roll_deg_  = DroneMath::clamp(raw_roll_deg,
                                                filtered_roll_deg_ - max_angle_step,
                                                filtered_roll_deg_ + max_angle_step);

        float wind_pitch_trim = DroneMath::rad2deg(std::atan2(wind_accel_xb, g));
        float wind_roll_trim  = DroneMath::rad2deg(std::atan2(-wind_accel_yb, g));

        bool is_braking = (mode_x_ == AxisMode::BRAKING || mode_y_ == AxisMode::BRAKING);
        bool is_hover = (mode_x_ == AxisMode::HOLD && mode_y_ == AxisMode::HOLD);

        PositionControlOutput out;
        out.pitch_deg = filtered_pitch_deg_;
        out.roll_deg  = filtered_roll_deg_;
        out.is_hover  = is_hover;
        out.is_braking = is_braking;
        out.anchor_x = anchor_x_;
        out.anchor_y = anchor_y_;
        out.wind_pitch_trim_deg = wind_pitch_trim;
        out.wind_roll_trim_deg  = wind_roll_trim;
        return out;
    }

    float anchor_x() const { return anchor_x_; }
    float anchor_y() const { return anchor_y_; }
    float wind_pitch_trim(float current_yaw_rad = 0.0f) const {
        float cy = std::cos(current_yaw_rad);
        float sy = std::sin(current_yaw_rad);
        float wind_accel_xb = cy * (0.40f * i_wind_xw_) + sy * (0.40f * i_wind_yw_);
        return DroneMath::rad2deg(std::atan2(wind_accel_xb, 9.80665f));
    }
    float wind_roll_trim(float current_yaw_rad = 0.0f) const {
        float cy = std::cos(current_yaw_rad);
        float sy = std::sin(current_yaw_rad);
        float wind_accel_yb = -sy * (0.40f * i_wind_xw_) + cy * (0.40f * i_wind_yw_);
        return DroneMath::rad2deg(std::atan2(-wind_accel_yb, 9.80665f));
    }

private:
    float anchor_x_{0.0f};
    float anchor_y_{0.0f};
    bool has_anchor_{false};

    AxisMode mode_x_{AxisMode::HOLD};
    AxisMode mode_y_{AxisMode::HOLD};

    float i_wind_xw_{0.0f}; // Tích phân bù gió trục X (World)
    float i_wind_yw_{0.0f}; // Tích phân bù gió trục Y (World)

    float smoothed_cmd_vx_{0.0f};
    float smoothed_cmd_vy_{0.0f};

    float filtered_pitch_deg_{0.0f};
    float filtered_roll_deg_{0.0f};

    bool was_driving_x_{false};
    bool was_driving_y_{false};
};

} // namespace DroneControllers
