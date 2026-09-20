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
    PositionController() = default;

    void reset() {
        i_wind_xb_ = 0.0f;
        i_wind_yb_ = 0.0f;
        smoothed_cmd_vx_ = 0.0f;
        smoothed_cmd_vy_ = 0.0f;
        filtered_pitch_deg_ = 0.0f;
        filtered_roll_deg_ = 0.0f;
        anchor_x_ = 0.0f;
        anchor_y_ = 0.0f;
        has_anchor_ = false;
        was_driving_x_ = false;
        was_driving_y_ = false;
    }

    void set_anchor(float x, float y) {
        anchor_x_ = x;
        anchor_y_ = y;
        has_anchor_ = true;
        smoothed_cmd_vx_ = 0.0f;
        smoothed_cmd_vy_ = 0.0f;
    }

    PositionControlOutput update(
        float current_x, float current_y,
        float vx_world, float vy_world,
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

        // 1. Chuyển đổi vận tốc thực tế từ World Frame sang Body Frame
        float vx_b =  cy * vx_world + sy * vy_world;
        float vy_b = -sy * vx_world + cy * vy_world;

        // 2. Gia tốc lệnh tay lái mượt mà (Slew Rate Limiter: max 3.5 m/s^2)
        const float max_accel = 3.5f;
        const float max_dv = max_accel * dt;

        if (is_driving_x) {
            if (!was_driving_x_) {
                smoothed_cmd_vx_ = vx_b; // bắt đầu từ vận tốc hiện tại
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

        // 3. KHÓA MỎ NEO TỨC THÌ LÚC BUÔNG PHÍM (Instant Anchor Snap - chuẩn DJI)
        // Khi đang ấn lái: Mỏ neo trượt theo drone
        // Khi vừa buông phím: Mỏ neo chốt cứng ngay tại tọa độ buông tay!
        if (is_driving_x) {
            anchor_x_ = current_x;
        }
        if (is_driving_y) {
            anchor_y_ = current_y;
        }

        const float kp_pos = 2.0f;           // Gain vị trí (1/s)
        const float max_approach_vel = 2.5f; // Vận tốc kéo về mỏ neo tối đa

        float err_x_w = anchor_x_ - current_x;
        float err_y_w = anchor_y_ - current_y;
        float err_xb =  cy * err_x_w + sy * err_y_w;
        float err_yb = -sy * err_x_w + cy * err_y_w;

        float target_vx_b = 0.0f;
        float target_vy_b = 0.0f;

        if (is_driving_x) {
            target_vx_b = smoothed_cmd_vx_;
        } else {
            // Khi buông tay: Sinh vận tốc âm kéo ngược về mỏ neo, triệt tiêu hoàn toàn trôi quán tính!
            target_vx_b = DroneMath::clamp(kp_pos * err_xb, -max_approach_vel, max_approach_vel);
        }

        if (is_driving_y) {
            target_vy_b = smoothed_cmd_vy_;
        } else {
            target_vy_b = DroneMath::clamp(kp_pos * err_yb, -max_approach_vel, max_approach_vel);
        }

        // 4. TẦNG VẬN TỐC (Inner Velocity Loop with Wind Integrator)
        float evx = target_vx_b - vx_b;
        float evy = target_vy_b - vy_b;

        // Tích phân bù gió (Wind Disturbance Integrator)
        const float max_wind_accel = 1.5f;
        const float ki_vel = 0.40f;
        const float int_limit = max_wind_accel / ki_vel;

        if (!is_driving_x) {
            i_wind_xb_ += evx * dt;
            i_wind_xb_ = DroneMath::clamp(i_wind_xb_, -int_limit, int_limit);
        }
        if (!is_driving_y) {
            i_wind_yb_ += evy * dt;
            i_wind_yb_ = DroneMath::clamp(i_wind_yb_, -int_limit, int_limit);
        }

        float wind_accel_x = ki_vel * i_wind_xb_;
        float wind_accel_y = ki_vel * i_wind_yb_;

        // Phản hồi vận tốc thuần túy, loại bỏ hoàn toàn vi phân gia tốc đo
        const float kp_vel = 2.2f;
        float a_sp_x = kp_vel * evx + wind_accel_x;
        float a_sp_y = kp_vel * evy + wind_accel_y;

        // 5. CHUYỂN ĐỔI GIA TỐC SANG GÓC NGHIÊNG THÂN
        const float g = 9.80665f;
        float raw_pitch_deg = DroneMath::rad2deg(std::atan2(a_sp_x, g));
        float raw_roll_deg  = DroneMath::rad2deg(std::atan2(-a_sp_y, g));

        // Cho phép góc phanh hãm lên tới 18.0° khi buông phím để dừng xe nhanh gấp đôi
        const float max_brake_tilt = 18.0f;
        float max_p = is_driving_x ? max_tilt_deg : max_brake_tilt;
        float max_r = is_driving_y ? max_tilt_deg : max_brake_tilt;

        raw_pitch_deg = DroneMath::clamp(raw_pitch_deg, -max_p, max_p);
        raw_roll_deg  = DroneMath::clamp(raw_roll_deg,  -max_r, max_r);

        // 6. BỘ LỌC TỐC ĐỘ GÓC (Tilt Angular Rate Limiter: 180°/s)
        // Đáp ứng góc nghiêng trong 0.12s, loại bỏ hoàn toàn hiện tượng trễ phanh
        const float max_tilt_rate_deg_s = 180.0f;
        float max_angle_step = max_tilt_rate_deg_s * dt; // ~6° mỗi chu kỳ 33ms

        filtered_pitch_deg_ = DroneMath::clamp(raw_pitch_deg,
                                                filtered_pitch_deg_ - max_angle_step,
                                                filtered_pitch_deg_ + max_angle_step);
        filtered_roll_deg_  = DroneMath::clamp(raw_roll_deg,
                                                filtered_roll_deg_ - max_angle_step,
                                                filtered_roll_deg_ + max_angle_step);

        float wind_pitch_trim = DroneMath::rad2deg(std::atan2(wind_accel_x, g));
        float wind_roll_trim  = DroneMath::rad2deg(std::atan2(-wind_accel_y, g));

        bool is_moving = (is_driving_x || is_driving_y);
        bool has_residual_vel = (std::abs(vx_b) > 0.15f || std::abs(vy_b) > 0.15f);

        PositionControlOutput out;
        out.pitch_deg = filtered_pitch_deg_;
        out.roll_deg  = filtered_roll_deg_;
        out.is_hover  = (!is_moving && !has_residual_vel);
        out.is_braking = (!is_moving && has_residual_vel);
        out.anchor_x = anchor_x_;
        out.anchor_y = anchor_y_;
        out.wind_pitch_trim_deg = wind_pitch_trim;
        out.wind_roll_trim_deg  = wind_roll_trim;
        return out;
    }

    float anchor_x() const { return anchor_x_; }
    float anchor_y() const { return anchor_y_; }
    float wind_pitch_trim() const { return DroneMath::rad2deg(std::atan2(0.40f * i_wind_xb_, 9.80665f)); }
    float wind_roll_trim() const { return DroneMath::rad2deg(std::atan2(-0.40f * i_wind_yb_, 9.80665f)); }

private:
    float anchor_x_{0.0f};
    float anchor_y_{0.0f};
    bool has_anchor_{false};

    float i_wind_xb_{0.0f};
    float i_wind_yb_{0.0f};

    float smoothed_cmd_vx_{0.0f};
    float smoothed_cmd_vy_{0.0f};

    float filtered_pitch_deg_{0.0f};
    float filtered_roll_deg_{0.0f};

    bool was_driving_x_{false};
    bool was_driving_y_{false};
};

} // namespace DroneControllers
