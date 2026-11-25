/**
 * @file rl_critic_controller.hpp
 * @brief Bộ điều khiển Học tăng cường Critic-Only Online kết hợp Bộ quan sát trạng thái mở rộng ESO
 *
 * Chuyển thể từ mô hình nghiên cứu MATLAB (on_critic_only.m) sang C++ thời gian thực 200Hz cho ESP32:
 *   1. Mạng Critic xấp xỉ hàm giá trị HJB: V(z) = W^T * sigma(z) với 9 hàm cơ sở bậc 2.
 *   2. Chính sách điều khiển tối ưu suy ra trực tiếp từ Critic (Critic-Only):
 *        u2 = -0.5 * R^-1 * G^T * grad_af(z)^T * W
 *   3. Cập nhật trọng số W trực tuyến (Per-Axis Decoupled Bellman Error + PE stabilization):
 *        dW = -k1 * gamma * eB / (1 + 0.25*||gamma||^2)^2 + k2 * l * varphi * W
 *   4. Bộ quan sát trạng thái mở rộng (ESO bậc 3 có bù trễ cơ cấu chấp hành):
 *        Ước lượng nhiễu tổng hợp (gió giật, sai lệch mô hình) dis_p_hat, dis_eta_hat và bù trừ thẳng vào ngõ ra.
 */

#pragma once

#include <cmath>
#include <algorithm>
#include <array>

#include "flight_config.h"
#include "math_utils.hpp"

namespace DroneControl {

/**
 * @brief Tính ma trận Jacobian của hàm cơ sở Critic grad_af(z) kích thước 9x6 (chuẩn on_critic_only.m)
 *        Vector trạng thái z = [e0, e1, e2, de0, de1, de2]
 */
inline void compute_critic_grad_af(const float z[6], float out[9][6]) {
    for (int r = 0; r < 9; ++r) {
        for (int c = 0; c < 6; ++c) {
            out[r][c] = 0.0f;
        }
    }
    for (int k = 0; k < 3; ++k) {
        out[k][k]         = 2.0f * z[k];     // d(e_k^2)/d(e_k)
        out[k + 3][k + 3] = 2.0f * z[k + 3]; // d(de_k^2)/d(de_k)
        out[k + 6][k]     = z[k + 3];        // d(e_k * de_k)/d(e_k)
        out[k + 6][k + 3] = z[k];            // d(e_k * de_k)/d(de_k)
    }
}

struct RLPositionOutput {
    float total_thrust{FlightConfig::Altitude::HOVER_THRUST};
    float roll_deg{0.0f};
    float pitch_deg{0.0f};
    bool is_hover{true};
    bool is_braking{false};
    float anchor_x{0.0f};
    float anchor_y{0.0f};
};

/**
 * @brief Bộ điều khiển Vị trí & Độ cao 3D bằng Online Critic-Only RL + ESO
 */
class RLCriticPositionController {
public:
    enum class AxisMode { DRIVING, BRAKING, HOLD };

    RLCriticPositionController() {
        reset();
    }

    void reset() {
        // Khởi tạo ~92% nghiệm Riccati để đảm bảo bay mượt ngay từ đầu và trọng số Wp hội tụ trực tuyến
        static constexpr float kWpInit[9] = {
            0.830f, 0.830f, 1.920f,
            0.625f, 0.625f, 0.560f,
            0.832f, 0.832f, 1.075f
        };
        for (int i = 0; i < 9; ++i) {
            wp_init_[i] = kWpInit[i];
            wp_[i]      = kWpInit[i];
        }

        for (int i = 0; i < 3; ++i) {
            p_hat_[i]     = 0.0f;
            dp_hat_[i]    = 0.0f;
            dis_p_hat_[i] = 0.0f;
        }
        up_eff_[0] = 0.0f;
        up_eff_[1] = 0.0f;
        up_eff_[2] = FlightConfig::RLCritic::MASS_KG * FlightConfig::RLCritic::GRAVITY;
        eso_init_ = false;

        anchor_x_ = 0.0f;
        anchor_y_ = 0.0f;
        ref_x_ = 0.0f;
        ref_y_ = 0.0f;
        has_anchor_ = false;
        mode_x_ = AxisMode::HOLD;
        mode_y_ = AxisMode::HOLD;
        was_driving_x_ = false;
        was_driving_y_ = false;
        brake_timer_x_ = 0.0f;
        brake_timer_y_ = 0.0f;
        smoothed_cmd_vx_ = 0.0f;
        smoothed_cmd_vy_ = 0.0f;
        filtered_pitch_deg_ = 0.0f;
        filtered_roll_deg_ = 0.0f;
        last_bellman_err_ = 0.0f;
    }

    void set_anchor(float x, float y, float z = 0.15f) {
        anchor_x_ = x;
        anchor_y_ = y;
        ref_x_ = x;
        ref_y_ = y;
        has_anchor_ = true;
        mode_x_ = AxisMode::HOLD;
        mode_y_ = AxisMode::HOLD;
        was_driving_x_ = false;
        was_driving_y_ = false;
        smoothed_cmd_vx_ = 0.0f;
        smoothed_cmd_vy_ = 0.0f;
        brake_timer_x_ = 0.0f;
        brake_timer_y_ = 0.0f;
        if (!eso_init_) {
            p_hat_[0] = x;
            p_hat_[1] = y;
            p_hat_[2] = z;
            dp_hat_[0] = 0.0f;
            dp_hat_[1] = 0.0f;
            dp_hat_[2] = 0.0f;
            dis_p_hat_[0] = 0.0f;
            dis_p_hat_[1] = 0.0f;
            dis_p_hat_[2] = 0.0f;
            up_eff_[0] = 0.0f;
            up_eff_[1] = 0.0f;
            up_eff_[2] = FlightConfig::RLCritic::MASS_KG * FlightConfig::RLCritic::GRAVITY;
            eso_init_ = true;
        }
    }

    RLPositionOutput update(
        float current_x, float current_y, float current_z,
        float vx_body, float vy_body, float vz_world,
        float current_yaw_rad,
        float cmd_vx_body, float cmd_vy_body,
        bool is_driving_x, bool is_driving_y,
        float target_alt, float target_vz_cmd,
        float max_tilt_deg = FlightConfig::Position::MAX_TILT_DRIVING_DEG,
        float dt = FlightConfig::Timing::DT
    ) {
        using namespace FlightConfig;
        if (!has_anchor_) {
            set_anchor(current_x, current_y, current_z);
        }
        if (dt <= 0.001f || dt > 0.1f) dt = Timing::DT;

        const float m = RLCritic::MASS_KG;
        const float g = RLCritic::GRAVITY;

        if (!eso_init_) {
            p_hat_[0] = current_x;
            p_hat_[1] = current_y;
            p_hat_[2] = current_z;
            dp_hat_[0] = 0.0f;
            dp_hat_[1] = 0.0f;
            dp_hat_[2] = vz_world;
            dis_p_hat_[0] = 0.0f;
            dis_p_hat_[1] = 0.0f;
            dis_p_hat_[2] = 0.0f;
            up_eff_[0] = 0.0f;
            up_eff_[1] = 0.0f;
            up_eff_[2] = m * g;
            eso_init_ = true;
        }

        const float cy = std::cos(current_yaw_rad);
        const float sy = std::sin(current_yaw_rad);
        const float vx_w = cy * vx_body - sy * vy_body;
        const float vy_w = sy * vx_body + cy * vy_body;

        // 1. Làm mượt lệnh vận tốc người lái (Rate Limiter)
        const float max_dv = Position::MAX_CMD_ACCEL * dt;
        if (is_driving_x) {
            if (!was_driving_x_) smoothed_cmd_vx_ = vx_body;
            if (cmd_vx_body > smoothed_cmd_vx_) {
                smoothed_cmd_vx_ = std::min(cmd_vx_body, smoothed_cmd_vx_ + max_dv);
            } else {
                smoothed_cmd_vx_ = std::max(cmd_vx_body, smoothed_cmd_vx_ - max_dv);
            }
        } else {
            smoothed_cmd_vx_ = 0.0f;
        }

        if (is_driving_y) {
            if (!was_driving_y_) smoothed_cmd_vy_ = vy_body;
            if (cmd_vy_body > smoothed_cmd_vy_) {
                smoothed_cmd_vy_ = std::min(cmd_vy_body, smoothed_cmd_vy_ + max_dv);
            } else {
                smoothed_cmd_vy_ = std::max(cmd_vy_body, smoothed_cmd_vy_ - max_dv);
            }
        } else {
            smoothed_cmd_vy_ = 0.0f;
        }

        // 2. Máy trạng thái DRIVING / BRAKING / HOLD đồng nhất với PositionController
        if (is_driving_x) {
            if (!was_driving_x_) ref_x_ = current_x;
            mode_x_ = AxisMode::DRIVING;
            brake_timer_x_ = 0.0f;
        } else {
            if (was_driving_x_) {
                mode_x_ = AxisMode::BRAKING;
                brake_timer_x_ = 0.0f;
            }
            if (mode_x_ == AxisMode::BRAKING) {
                brake_timer_x_ += dt;
                if (std::abs(vx_body) <= Position::STOP_VEL_THRESHOLD || brake_timer_x_ >= 1.00f) {
                    mode_x_ = AxisMode::HOLD;
                    anchor_x_ = current_x;
                    ref_x_ = current_x;
                }
            }
        }

        if (is_driving_y) {
            if (!was_driving_y_) ref_y_ = current_y;
            mode_y_ = AxisMode::DRIVING;
            brake_timer_y_ = 0.0f;
        } else {
            if (was_driving_y_) {
                mode_y_ = AxisMode::BRAKING;
                brake_timer_y_ = 0.0f;
            }
            if (mode_y_ == AxisMode::BRAKING) {
                brake_timer_y_ += dt;
                if (std::abs(vy_body) <= Position::STOP_VEL_THRESHOLD || brake_timer_y_ >= 1.00f) {
                    mode_y_ = AxisMode::HOLD;
                    anchor_y_ = current_y;
                    ref_y_ = current_y;
                }
            }
        }

        was_driving_x_ = is_driving_x;
        was_driving_y_ = is_driving_y;

        if (mode_x_ == AxisMode::DRIVING) {
            anchor_x_ = current_x;
        }
        if (mode_y_ == AxisMode::DRIVING) {
            anchor_y_ = current_y;
        }

        // 3. Tạo quỹ đạo tham chiếu (pd, dpd) độc lập từng trục trong World ENU
        const float cmd_vx_w = cy * smoothed_cmd_vx_ - sy * smoothed_cmd_vy_;
        const float cmd_vy_w = sy * smoothed_cmd_vx_ + cy * smoothed_cmd_vy_;

        float pd_x = anchor_x_, pd_y = anchor_y_, pd_z = target_alt;
        float dpd_x = 0.0f, dpd_y = 0.0f, dpd_z = target_vz_cmd;

        if (is_driving_x || is_driving_y) {
            if (is_driving_x) {
                ref_x_ += cmd_vx_w * dt;
                ref_x_ = DroneMath::clamp(ref_x_, current_x - 0.45f, current_x + 0.45f);
                pd_x = ref_x_;
                dpd_x = cmd_vx_w;
            } else if (mode_x_ == AxisMode::BRAKING) {
                ref_x_ = current_x;
                pd_x = current_x;
                dpd_x = 0.0f;
            } else {
                ref_x_ = anchor_x_;
                pd_x = anchor_x_;
                dpd_x = 0.0f;
            }

            if (is_driving_y) {
                ref_y_ += cmd_vy_w * dt;
                ref_y_ = DroneMath::clamp(ref_y_, current_y - 0.45f, current_y + 0.45f);
                pd_y = ref_y_;
                dpd_y = cmd_vy_w;
            } else if (mode_y_ == AxisMode::BRAKING) {
                ref_y_ = current_y;
                pd_y = current_y;
                dpd_y = 0.0f;
            } else {
                ref_y_ = anchor_y_;
                pd_y = anchor_y_;
                dpd_y = 0.0f;
            }
        } else if (mode_x_ == AxisMode::BRAKING || mode_y_ == AxisMode::BRAKING) {
            ref_x_ = current_x;
            ref_y_ = current_y;
            pd_x = current_x;
            pd_y = current_y;
            dpd_x = 0.0f;
            dpd_y = 0.0f;
        } else {
            // HOLD tại điểm neo (áp dụng deadband nhỏ để khử rung lắc quanh tâm)
            float ex_hold = current_x - anchor_x_;
            float ey_hold = current_y - anchor_y_;
            if (std::abs(ex_hold) < Position::POS_DEADBAND_M) {
                pd_x = current_x;
            } else {
                pd_x = anchor_x_ + ((ex_hold > 0.0f) ? Position::POS_DEADBAND_M : -Position::POS_DEADBAND_M);
            }
            if (std::abs(ey_hold) < Position::POS_DEADBAND_M) {
                pd_y = current_y;
            } else {
                pd_y = anchor_y_ + ((ey_hold > 0.0f) ? Position::POS_DEADBAND_M : -Position::POS_DEADBAND_M);
            }
            dpd_x = 0.0f;
            dpd_y = 0.0f;
        }

        // 4. Tính sai số trạng thái vị trí zp = [ep; dep] (dạng chuẩn on_critic_only.m)
        float ep[3] = {
            DroneMath::clamp(current_x - pd_x, -3.0f, 3.0f),
            DroneMath::clamp(current_y - pd_y, -3.0f, 3.0f),
            DroneMath::clamp(current_z - pd_z, -3.0f, 3.0f)
        };
        float dep[3] = {
            DroneMath::clamp(vx_w - dpd_x, -5.0f, 5.0f),
            DroneMath::clamp(vy_w - dpd_y, -5.0f, 5.0f),
            DroneMath::clamp(vz_world - dpd_z, -4.0f, 4.0f)
        };

        const float rp[3] = {RLCritic::RP_XY, RLCritic::RP_XY, RLCritic::RP_Z};
        const float qp[6] = {
            RLCritic::QP_POS_XY, RLCritic::QP_POS_XY, RLCritic::QP_POS_Z,
            RLCritic::QP_VEL_XY, RLCritic::QP_VEL_XY, RLCritic::QP_VEL_Z
        };

        float zp[6] = {ep[0], ep[1], ep[2], dep[0], dep[1], dep[2]};
        float grad_p[9][6];
        compute_critic_grad_af(zp, grad_p);

        // 5. Chính sách điều khiển tối ưu từ Critic: up2 = -0.5 * Rp^-1 * Gp^T * grad_af(zp)^T * Wp
        float up2[3] = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < 3; ++i) {
            float dV_dvei = 2.0f * wp_[i + 3] * dep[i] + wp_[i + 6] * ep[i];
            up2[i] = -0.5f / (rp[i] * m) * dV_dvei;
        }

        // Tổng hợp lực đẩy: up = up1 + up2 - m * dis_p_hat (bù trọng lực + bù nhiễu ESO)
        float up[3] = {
            up2[0] - m * dis_p_hat_[0],
            up2[1] - m * dis_p_hat_[1],
            m * g + up2[2] - m * dis_p_hat_[2]
        };

        // Kẹp lực nâng đứng & lực đẩy ngang trong giới hạn vật lý tối đa
        up[2] = DroneMath::clamp(up[2], m * g * 0.35f, RLCritic::MAX_TOTAL_THRUST_N * Altitude::MAX_TOTAL_THRUST);

        float max_horiz_force = up[2] * std::tan(DroneMath::deg2rad(max_tilt_deg));
        float horiz_norm = std::sqrt(up[0] * up[0] + up[1] * up[1]);
        if (horiz_norm > max_horiz_force && horiz_norm > 1e-4f) {
            float scale = max_horiz_force / horiz_norm;
            up[0] *= scale;
            up[1] *= scale;
        }

        // 6. Học tăng cường trực tuyến tách trục (Per-Axis Decoupled Bellman Update)
        // Tránh sai số Bellman trục Z khi cất cánh làm sai lệch trọng số trục ngang X/Y
        float f_plus_gu[6] = {
            dep[0], dep[1], dep[2],
            up2[0] / m, up2[1] / m, up2[2] / m
        };
        float total_eB = 0.0f;
        for (int i = 0; i < 3; ++i) {
            const int idxs[3] = {i, i + 3, i + 6};
            float gamma_i[3];
            float vdot_i = 0.0f;
            float gamma_norm2_i = 0.0f;
            for (int a = 0; a < 3; ++a) {
                int j = idxs[a];
                float g_val = grad_p[j][i] * f_plus_gu[i] + grad_p[j][i + 3] * f_plus_gu[i + 3];
                gamma_i[a] = g_val;
                vdot_i += wp_[j] * g_val;
                gamma_norm2_i += g_val * g_val;
            }

            float r_cost_i = qp[i] * zp[i] * zp[i] + qp[i + 3] * zp[i + 3] * zp[i + 3] + rp[i] * up2[i] * up2[i];
            float eB_i = vdot_i + r_cost_i;
            total_eB += eB_i;
            float l_i = (vdot_i < 0.0f) ? 0.0f : 1.0f;

            float d_i = 1.0f / (m * m * rp[i]);
            float gtw = 2.0f * wp_[i + 3] * dep[i] + wp_[i + 6] * ep[i];
            float scaled = 0.5f * d_i * gtw;
            float varphi_i[3] = {
                0.0f,
                grad_p[i + 3][i + 3] * scaled,
                grad_p[i + 6][i + 3] * scaled
            };

            float denom_i = 1.0f + 0.25f * gamma_norm2_i;
            denom_i = denom_i * denom_i;
            for (int a = 0; a < 3; ++a) {
                int j = idxs[a];
                float dWp = -RLCritic::KP1_LEARN * gamma_i[a] * eB_i / denom_i +
                             RLCritic::KP2_STAB * l_i * varphi_i[a];
                wp_[j] = DroneMath::clamp(wp_[j] + dWp * dt, 0.65f * wp_init_[j], 1.60f * wp_init_[j]);
            }
        }
        last_bellman_err_ = total_eB;

        // 7. Chuyển đổi vector lực đẩy World ENU sang góc nghiêng (Roll, Pitch) hệ thân & lực nâng
        float uT = std::sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
        float u_xb =  cy * up[0] + sy * up[1];
        float u_yb = -sy * up[0] + cy * up[1];

        float raw_roll_deg  = DroneMath::rad2deg(std::asin(DroneMath::clamp(-u_yb / std::max(uT, 1.0f), -0.85f, 0.85f)));
        float raw_pitch_deg = DroneMath::rad2deg(std::atan2(u_xb, std::max(up[2], 1.0f)));

        float max_p = (mode_x_ == AxisMode::DRIVING) ? max_tilt_deg : ((mode_x_ == AxisMode::BRAKING) ? Position::MAX_TILT_BRAKING_DEG : Position::MAX_TILT_HOLD_DEG);
        float max_r = (mode_y_ == AxisMode::DRIVING) ? max_tilt_deg : ((mode_y_ == AxisMode::BRAKING) ? Position::MAX_TILT_BRAKING_DEG : Position::MAX_TILT_HOLD_DEG);

        raw_pitch_deg = DroneMath::clamp(raw_pitch_deg, -max_p, max_p);
        raw_roll_deg  = DroneMath::clamp(raw_roll_deg,  -max_r, max_r);

        float max_angle_step = Position::MAX_TILT_RATE_DEG_S * dt;
        filtered_pitch_deg_ = DroneMath::clamp(raw_pitch_deg, filtered_pitch_deg_ - max_angle_step, filtered_pitch_deg_ + max_angle_step);
        filtered_roll_deg_  = DroneMath::clamp(raw_roll_deg,  filtered_roll_deg_  - max_angle_step, filtered_roll_deg_  + max_angle_step);

        float clamped_thrust_norm = DroneMath::clamp(uT / RLCritic::MAX_TOTAL_THRUST_N, Altitude::MIN_TOTAL_THRUST, Altitude::MAX_TOTAL_THRUST);

        // 8. Cập nhật Bộ quan sát ESO bậc 3 với lực thực tế sau bão hòa & lọc trễ đáp ứng góc nghiêng
        float r_rad = DroneMath::deg2rad(filtered_roll_deg_);
        float p_rad = DroneMath::deg2rad(filtered_pitch_deg_);
        float sr_a = std::sin(r_rad), cr_a = std::cos(r_rad);
        float sp_a = std::sin(p_rad), cp_a = std::cos(p_rad);
        float uT_applied = clamped_thrust_norm * RLCritic::MAX_TOTAL_THRUST_N;
        const float up_applied[3] = {
            (cy * sp_a * cr_a + sy * sr_a) * uT_applied,
            (sy * sp_a * cr_a - cy * sr_a) * uT_applied,
            (cp_a * cr_a) * uT_applied
        };
        const float alpha_pos_lag = std::min(1.0f, dt / RLCritic::ESO_ACT_LAG_POS_S);
        for (int i = 0; i < 3; ++i) {
            up_eff_[i] += (up_applied[i] - up_eff_[i]) * alpha_pos_lag;
        }

        const float eps = RLCritic::ESO_EPS_POS;
        const float c1_e = RLCritic::ESO_C1 / eps;
        const float c2_e2 = RLCritic::ESO_C2 / (eps * eps);
        const float c3_e3 = RLCritic::ESO_C3 / (eps * eps * eps);
        const float cur_p[3] = {current_x, current_y, current_z};

        for (int i = 0; i < 3; ++i) {
            float err_hat = p_hat_[i] - cur_p[i];
            float p_hat_1dot = dp_hat_[i] - c1_e * err_hat;
            float p_hat_2dot = ((i == 2) ? -g : 0.0f) + (up_eff_[i] / m) - c2_e2 * err_hat + dis_p_hat_[i];
            float d_dis_p    = -c3_e3 * err_hat;

            p_hat_[i]     += p_hat_1dot * dt;
            dp_hat_[i]    += p_hat_2dot * dt;
            dis_p_hat_[i] += d_dis_p * dt;
        }
        dis_p_hat_[0] = DroneMath::clamp(dis_p_hat_[0], -RLCritic::ESO_MAX_DIS_P_XY, RLCritic::ESO_MAX_DIS_P_XY);
        dis_p_hat_[1] = DroneMath::clamp(dis_p_hat_[1], -RLCritic::ESO_MAX_DIS_P_XY, RLCritic::ESO_MAX_DIS_P_XY);
        dis_p_hat_[2] = DroneMath::clamp(dis_p_hat_[2], -RLCritic::ESO_MAX_DIS_P_Z,  RLCritic::ESO_MAX_DIS_P_Z);

        RLPositionOutput out;
        out.total_thrust = clamped_thrust_norm;
        out.pitch_deg = filtered_pitch_deg_;
        out.roll_deg  = filtered_roll_deg_;
        out.is_hover  = (!is_driving_x && !is_driving_y && mode_x_ == AxisMode::HOLD && mode_y_ == AxisMode::HOLD);
        out.is_braking = (mode_x_ == AxisMode::BRAKING || mode_y_ == AxisMode::BRAKING);
        out.anchor_x = anchor_x_;
        out.anchor_y = anchor_y_;
        return out;
    }

    float get_bellman_error() const { return last_bellman_err_; }
    float get_wp(int idx) const { return (idx >= 0 && idx < 9) ? wp_[idx] : 0.0f; }
    float get_dis_hat(int axis) const { return (axis >= 0 && axis < 3) ? dis_p_hat_[axis] : 0.0f; }

private:
    float wp_init_[9]{};
    float wp_[9]{};
    float p_hat_[3]{0.0f, 0.0f, 0.0f};
    float dp_hat_[3]{0.0f, 0.0f, 0.0f};
    float dis_p_hat_[3]{0.0f, 0.0f, 0.0f};
    float up_eff_[3]{0.0f, 0.0f, FlightConfig::RLCritic::MASS_KG * FlightConfig::RLCritic::GRAVITY};
    bool eso_init_{false};

    float anchor_x_{0.0f}, anchor_y_{0.0f};
    float ref_x_{0.0f}, ref_y_{0.0f};
    bool has_anchor_{false};
    AxisMode mode_x_{AxisMode::HOLD}, mode_y_{AxisMode::HOLD};
    bool was_driving_x_{false}, was_driving_y_{false};
    float brake_timer_x_{0.0f}, brake_timer_y_{0.0f};
    float smoothed_cmd_vx_{0.0f}, smoothed_cmd_vy_{0.0f};
    float filtered_pitch_deg_{0.0f}, filtered_roll_deg_{0.0f};
    float last_bellman_err_{0.0f};
};

/**
 * @brief Bộ điều khiển Góc thái độ (Roll, Pitch, Yaw) bằng Online Critic-Only RL + ESO
 */
class RLCriticAttitudeController {
public:
    RLCriticAttitudeController() {
        reset();
    }

    void reset() {
        // Khởi tạo ~92% nghiệm Riccati cho QE=(2.40, 0.055, RE=1.0) & Yaw=(0.25, 0.08, RE=10.0)
        static constexpr float kWeInit[9] = {
            0.498f, 0.498f, 0.210f,
            0.00700f, 0.00700f, 0.0530f,
            0.0618f, 0.0618f, 0.1165f
        };
        for (int i = 0; i < 9; ++i) {
            we_init_[i] = kWeInit[i];
            we_[i]      = kWeInit[i];
        }
        for (int i = 0; i < 3; ++i) {
            eta_hat_[i]     = 0.0f;
            deta_hat_[i]    = 0.0f;
            dis_eta_hat_[i] = 0.0f;
            ue_eff_[i]      = 0.0f;
        }
        eso_init_ = false;
        last_bellman_err_ = 0.0f;
    }

    void update(
        float target_roll, float target_pitch, float target_yaw,
        float cur_roll, float cur_pitch, float cur_yaw,
        float cur_p, float cur_q, float cur_r,
        float dt,
        float& out_tau_roll, float& out_tau_pitch, float& out_tau_yaw
    ) {
        using namespace FlightConfig;
        if (dt <= 0.0005f || dt > 0.1f) dt = Timing::DT;

        if (!eso_init_) {
            eta_hat_[0] = cur_roll;
            eta_hat_[1] = cur_pitch;
            eta_hat_[2] = cur_yaw;
            deta_hat_[0] = cur_p;
            deta_hat_[1] = cur_q;
            deta_hat_[2] = cur_r;
            dis_eta_hat_[0] = 0.0f;
            dis_eta_hat_[1] = 0.0f;
            dis_eta_hat_[2] = 0.0f;
            ue_eff_[0] = 0.0f;
            ue_eff_[1] = 0.0f;
            ue_eff_[2] = 0.0f;
            eso_init_ = true;
        }

        // 1. Tính sai số thái độ z_eta = [e_eta; de_eta] (không đạo hàm góc đặt để tránh derivative kick khi HITL có trễ)
        float e_eta[3] = {
            DroneMath::clamp(cur_roll - target_roll, -1.0f, 1.0f),
            DroneMath::clamp(cur_pitch - target_pitch, -1.0f, 1.0f),
            DroneMath::clamp(DroneMath::normalize_angle(cur_yaw - target_yaw), -1.5f, 1.5f)
        };
        float de_eta[3] = {
            DroneMath::clamp(cur_p, -6.0f, 6.0f),
            DroneMath::clamp(cur_q, -6.0f, 6.0f),
            DroneMath::clamp(cur_r, -4.0f, 4.0f)
        };

        const float J[3]  = {RLCritic::JX, RLCritic::JY, RLCritic::JZ};
        const float re[3] = {RLCritic::RE_RP, RLCritic::RE_RP, RLCritic::RE_YAW};
        const float qe[6] = {
            RLCritic::QE_ANG_RP,  RLCritic::QE_ANG_RP,  RLCritic::QE_ANG_YAW,
            RLCritic::QE_RATE_RP, RLCritic::QE_RATE_RP, RLCritic::QE_RATE_YAW
        };

        // Thành phần Coriolis con quay hồi chuyển: omega x (J * omega)
        const float cor[3] = {
            (J[2] - J[1]) * cur_q * cur_r,
            (J[0] - J[2]) * cur_p * cur_r,
            (J[1] - J[0]) * cur_p * cur_q
        };

        float z_eta[6] = {e_eta[0], e_eta[1], e_eta[2], de_eta[0], de_eta[1], de_eta[2]};
        float grad_e[9][6];
        compute_critic_grad_af(z_eta, grad_e);

        // 2. Tín hiệu điều khiển tối ưu Critic: ue2 = -0.5 * Re^-1 * Ge^T * grad_af(z_eta)^T * We
        float ue2[3] = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < 3; ++i) {
            float dV_ddei = 2.0f * we_[i + 3] * de_eta[i] + we_[i + 6] * e_eta[i];
            ue2[i] = -0.5f / (re[i] * J[i]) * dV_ddei;
        }

        // Tổng hợp mô-men vật lý (N.m): ue = Cor + ue2 - J * dis_eta_hat
        float ue[3] = {
            cor[0] + ue2[0] - J[0] * dis_eta_hat_[0],
            cor[1] + ue2[1] - J[1] * dis_eta_hat_[1],
            cor[2] + ue2[2] - J[2] * dis_eta_hat_[2]
        };

        // 3. Học tăng cường trực tuyến tách trục (Per-Axis Decoupled Bellman Update)
        float f_plus_gu[6] = {
            de_eta[0], de_eta[1], de_eta[2],
            (-cor[0] + ue2[0]) / J[0],
            (-cor[1] + ue2[1]) / J[1],
            (-cor[2] + ue2[2]) / J[2]
        };
        float total_eB = 0.0f;
        for (int i = 0; i < 3; ++i) {
            const int idxs[3] = {i, i + 3, i + 6};
            float gamma_i[3];
            float vdot_i = 0.0f;
            float gamma_norm2_i = 0.0f;
            for (int a = 0; a < 3; ++a) {
                int j = idxs[a];
                float g_val = grad_e[j][i] * f_plus_gu[i] + grad_e[j][i + 3] * f_plus_gu[i + 3];
                gamma_i[a] = g_val;
                vdot_i += we_[j] * g_val;
                gamma_norm2_i += g_val * g_val;
            }

            float r_cost_i = qe[i] * z_eta[i] * z_eta[i] + qe[i + 3] * z_eta[i + 3] * z_eta[i + 3] + re[i] * ue2[i] * ue2[i];
            float eB_i = vdot_i + r_cost_i;
            total_eB += eB_i;
            float l_i = (vdot_i < 0.0f) ? 0.0f : 1.0f;

            float d_i = 1.0f / (J[i] * J[i] * re[i]);
            float gtw = 2.0f * we_[i + 3] * de_eta[i] + we_[i + 6] * e_eta[i];
            float scaled = 0.5f * d_i * gtw;
            float varphi_i[3] = {
                0.0f,
                grad_e[i + 3][i + 3] * scaled,
                grad_e[i + 6][i + 3] * scaled
            };

            float denom_i = 1.0f + 0.25f * gamma_norm2_i;
            denom_i = denom_i * denom_i;
            for (int a = 0; a < 3; ++a) {
                int j = idxs[a];
                float dWe = -RLCritic::KE1_LEARN * gamma_i[a] * eB_i / denom_i +
                             RLCritic::KE2_STAB * l_i * varphi_i[a];
                we_[j] = DroneMath::clamp(we_[j] + dWe * dt, 0.70f * we_init_[j], 1.50f * we_init_[j]);
            }
        }
        last_bellman_err_ = total_eB;

        // 4. Chuẩn hóa & giới hạn mô-men vật lý sang thang MotorMixer TRƯỚC khi đưa vào ESO (Anti-windup)
        out_tau_roll  = DroneMath::clamp(ue[0] / RLCritic::MIXER_ROLL_SCALE_NM,  -Attitude::MAX_TAU_ROLL,  Attitude::MAX_TAU_ROLL);
        out_tau_pitch = DroneMath::clamp(ue[1] / RLCritic::MIXER_PITCH_SCALE_NM, -Attitude::MAX_TAU_PITCH, Attitude::MAX_TAU_PITCH);
        out_tau_yaw   = DroneMath::clamp(ue[2] / RLCritic::MIXER_YAW_SCALE_NM,   -Attitude::MAX_TAU_YAW,   Attitude::MAX_TAU_YAW);

        const float ue_applied[3] = {
            out_tau_roll  * RLCritic::MIXER_ROLL_SCALE_NM,
            out_tau_pitch * RLCritic::MIXER_PITCH_SCALE_NM,
            out_tau_yaw   * RLCritic::MIXER_YAW_SCALE_NM
        };
        const float alpha_att_lag = std::min(1.0f, dt / RLCritic::ESO_ACT_LAG_ATT_S);
        for (int i = 0; i < 3; ++i) {
            ue_eff_[i] += (ue_applied[i] - ue_eff_[i]) * alpha_att_lag;
        }

        // 5. Cập nhật Bộ quan sát ESO bậc 3 cho góc thái độ (dùng mô-men thực tế sau bão hòa & trễ động cơ)
        const float eps = RLCritic::ESO_EPS_ATT;
        const float c1_e = RLCritic::ESO_C1 / eps;
        const float c2_e2 = RLCritic::ESO_C2 / (eps * eps);
        const float c3_e3 = RLCritic::ESO_C3 / (eps * eps * eps);
        const float cur_eta[3] = {cur_roll, cur_pitch, cur_yaw};

        for (int i = 0; i < 3; ++i) {
            float err_hat = (i == 2) ? DroneMath::normalize_angle(eta_hat_[2] - cur_eta[2])
                                     : (eta_hat_[i] - cur_eta[i]);
            float eta_1dot = deta_hat_[i] - c1_e * err_hat;
            float eta_2dot = ((-cor[i] + ue_eff_[i]) / J[i]) - c2_e2 * err_hat + dis_eta_hat_[i];
            float d_dis    = -c3_e3 * err_hat;

            eta_hat_[i]     = (i == 2) ? DroneMath::normalize_angle(eta_hat_[i] + eta_1dot * dt)
                                       : (eta_hat_[i] + eta_1dot * dt);
            deta_hat_[i]    += eta_2dot * dt;
            dis_eta_hat_[i] += d_dis * dt;
        }
        dis_eta_hat_[0] = DroneMath::clamp(dis_eta_hat_[0], -RLCritic::ESO_MAX_DIS_ATT_RP,  RLCritic::ESO_MAX_DIS_ATT_RP);
        dis_eta_hat_[1] = DroneMath::clamp(dis_eta_hat_[1], -RLCritic::ESO_MAX_DIS_ATT_RP,  RLCritic::ESO_MAX_DIS_ATT_RP);
        dis_eta_hat_[2] = DroneMath::clamp(dis_eta_hat_[2], -RLCritic::ESO_MAX_DIS_ATT_YAW, RLCritic::ESO_MAX_DIS_ATT_YAW);
    }

    float get_bellman_error() const { return last_bellman_err_; }
    float get_we(int idx) const { return (idx >= 0 && idx < 9) ? we_[idx] : 0.0f; }

private:
    float we_init_[9]{};
    float we_[9]{};
    float eta_hat_[3]{0.0f, 0.0f, 0.0f};
    float deta_hat_[3]{0.0f, 0.0f, 0.0f};
    float dis_eta_hat_[3]{0.0f, 0.0f, 0.0f};
    float ue_eff_[3]{0.0f, 0.0f, 0.0f};
    bool eso_init_{false};
    float last_bellman_err_{0.0f};
};

} // namespace DroneControl
