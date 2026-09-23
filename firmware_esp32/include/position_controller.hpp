#pragma once
#include <cmath>
#include <algorithm>
#include "flight_config.h"
#include "math_utils.hpp"

namespace DroneControllers {

struct PositionControlOutput {
    float roll_deg{0.0f};
    float pitch_deg{0.0f};
    bool is_hover{true};
    bool is_braking{false};
    float anchor_x{0.0f};
    float anchor_y{0.0f};
};

/**
 * @brief Bộ điều khiển vị trí 2D và phanh dừng chủ động cho ESP32
 * Tham số đọc từ FlightConfig::Position và FlightConfig::Timing
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
        float max_tilt_deg = FlightConfig::Position::MAX_TILT_DRIVING_DEG,
        float dt = FlightConfig::Timing::DT
    ) {
        if (!has_anchor_) {
            set_anchor(current_x, current_y);
        }
        if (dt <= 0.001f || dt > 0.1f) dt = FlightConfig::Timing::DT;

        float cy = std::cos(current_yaw_rad);
        float sy = std::sin(current_yaw_rad);
        float vx_b = vx_body;
        float vy_b = vy_body;

        const float max_dv = FlightConfig::Position::MAX_CMD_ACCEL * dt;

        if (is_driving_x) {
            if (!was_driving_x_) smoothed_cmd_vx_ = vx_b;
            if (cmd_vx_body > smoothed_cmd_vx_) {
                smoothed_cmd_vx_ = std::min(cmd_vx_body, smoothed_cmd_vx_ + max_dv);
            } else {
                smoothed_cmd_vx_ = std::max(cmd_vx_body, smoothed_cmd_vx_ - max_dv);
            }
        } else {
            smoothed_cmd_vx_ = 0.0f;
        }

        if (is_driving_y) {
            if (!was_driving_y_) smoothed_cmd_vy_ = vy_b;
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

        float target_vx_b = 0.0f;
        float target_vy_b = 0.0f;

        // Trục X (Tiến / Lùi)
        if (is_driving_x) {
            mode_x_ = AxisMode::DRIVING;
            anchor_x_ = current_x;
            target_vx_b = smoothed_cmd_vx_;
        } else {
            if (mode_x_ == AxisMode::DRIVING) {
                mode_x_ = AxisMode::BRAKING;
            }
            if (mode_x_ == AxisMode::BRAKING) {
                anchor_x_ = current_x;
                target_vx_b = 0.0f;
                if (std::abs(vx_b) < FlightConfig::Position::STOP_VEL_THRESHOLD) {
                    mode_x_ = AxisMode::HOLD;
                    anchor_x_ = current_x;
                }
            } else {
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_xb  = cy * err_x_w + sy * err_y_w;
                target_vx_b = DroneMath::clamp(FlightConfig::Position::KP_POS * err_xb, 
                                               -FlightConfig::Position::MAX_APPROACH_VEL, 
                                               FlightConfig::Position::MAX_APPROACH_VEL);
            }
        }

        // Trục Y (Trái / Phải)
        if (is_driving_y) {
            mode_y_ = AxisMode::DRIVING;
            anchor_y_ = current_y;
            target_vy_b = smoothed_cmd_vy_;
        } else {
            if (mode_y_ == AxisMode::DRIVING) {
                mode_y_ = AxisMode::BRAKING;
            }
            if (mode_y_ == AxisMode::BRAKING) {
                anchor_y_ = current_y;
                target_vy_b = 0.0f;
                if (std::abs(vy_b) < FlightConfig::Position::STOP_VEL_THRESHOLD) {
                    mode_y_ = AxisMode::HOLD;
                    anchor_y_ = current_y;
                }
            } else {
                float err_x_w = anchor_x_ - current_x;
                float err_y_w = anchor_y_ - current_y;
                float err_yb  = -sy * err_x_w + cy * err_y_w;
                target_vy_b = DroneMath::clamp(FlightConfig::Position::KP_POS * err_yb, 
                                               -FlightConfig::Position::MAX_APPROACH_VEL, 
                                               FlightConfig::Position::MAX_APPROACH_VEL);
            }
        }

        float evx = target_vx_b - vx_b;
        float evy = target_vy_b - vy_b;

        const float int_limit = FlightConfig::Position::MAX_WIND_ACCEL / FlightConfig::Position::KI_VEL;

        if (mode_x_ == AxisMode::HOLD && mode_y_ == AxisMode::HOLD &&
            std::abs(vx_b) < 0.15f && std::abs(vy_b) < 0.15f) {
            float evx_w = cy * evx - sy * evy;
            float evy_w = sy * evx + cy * evy;
            i_wind_xw_ += evx_w * dt;
            i_wind_xw_ = DroneMath::clamp(i_wind_xw_, -int_limit, int_limit);
            i_wind_yw_ += evy_w * dt;
            i_wind_yw_ = DroneMath::clamp(i_wind_yw_, -int_limit, int_limit);
        }

        float wind_accel_xb =  cy * (FlightConfig::Position::KI_VEL * i_wind_xw_) + sy * (FlightConfig::Position::KI_VEL * i_wind_yw_);
        float wind_accel_yb = -sy * (FlightConfig::Position::KI_VEL * i_wind_xw_) + cy * (FlightConfig::Position::KI_VEL * i_wind_yw_);

        float a_sp_x = FlightConfig::Position::KP_VEL * evx + wind_accel_xb;
        float a_sp_y = FlightConfig::Position::KP_VEL * evy + wind_accel_yb;

        const float g = 9.80665f;
        float raw_pitch_deg = DroneMath::rad2deg(std::atan2(a_sp_x, g));
        float raw_roll_deg  = DroneMath::rad2deg(std::atan2(-a_sp_y, g));

        float max_p = (mode_x_ == AxisMode::DRIVING) ? max_tilt_deg : FlightConfig::Position::MAX_TILT_BRAKING_DEG;
        float max_r = (mode_y_ == AxisMode::DRIVING) ? max_tilt_deg : FlightConfig::Position::MAX_TILT_BRAKING_DEG;

        raw_pitch_deg = DroneMath::clamp(raw_pitch_deg, -max_p, max_p);
        raw_roll_deg  = DroneMath::clamp(raw_roll_deg,  -max_r, max_r);

        float max_angle_step = FlightConfig::Position::MAX_TILT_RATE_DEG_S * dt;

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
    float anchor_x_{0.0f};
    float anchor_y_{0.0f};
    bool has_anchor_{false};
    AxisMode mode_x_{AxisMode::HOLD};
    AxisMode mode_y_{AxisMode::HOLD};
    float i_wind_xw_{0.0f};
    float i_wind_yw_{0.0f};
    float smoothed_cmd_vx_{0.0f};
    float smoothed_cmd_vy_{0.0f};
    float filtered_pitch_deg_{0.0f};
    float filtered_roll_deg_{0.0f};
    bool was_driving_x_{false};
    bool was_driving_y_{false};
};

} // namespace DroneControllers
