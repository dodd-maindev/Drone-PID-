#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include "math/math_utils.hpp"

namespace DroneFormation {

struct Vec3 {
    float x{0.0f}, y{0.0f}, z{0.0f};
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
};

struct DroneState {
    float x{0.0f}, y{0.0f}, z{0.12f};
    float vx_w{0.0f}, vy_w{0.0f}, vz_w{0.0f};
    float roll{0.0f}, pitch{0.0f}, yaw{0.0f};
    float p{0.0f}, q{0.0f}, r{0.0f};
};

/**
 * @brief Bộ điều khiển SMC + High-Gain Observer (hgo_3_dynamics.m) cho 1 Quadrotor X500
 */
class SMCHGOController {
public:
    static constexpr float MASS_KG              = 2.064f;
    static constexpr float GRAVITY              = 9.80f;
    static constexpr float JX                   = 0.02167f;
    static constexpr float JY                   = 0.02167f;
    static constexpr float JZ                   = 0.04000f;
    static constexpr float MAX_TOTAL_THRUST_N   = 34.194f;
    static constexpr float MIXER_ROLL_SCALE_NM  = 5.950f;
    static constexpr float MIXER_PITCH_SCALE_NM = 5.950f;
    static constexpr float MIXER_YAW_SCALE_NM   = 0.5471f;

    void reset(const Vec3& init_pos) {
        eta_des_ = {};
        deta_des_ = {};
        ddeta_des_ = {};
        hgo_init_ = false;
        z_int_ = 0.0f;
        motor_speeds_.fill(0.0f);
        (void)init_pos;
    }

    std::array<float, 4> update(
        const DroneState& st,
        const Vec3& p_des,
        const Vec3& dp_des,
        const Vec3& ddp_des,
        float dt,
        bool armed = true
    ) {
        if (!armed) {
            motor_speeds_.fill(0.0f);
            return motor_speeds_;
        }
        if (dt <= 0.0005f || dt > 0.1f) dt = 0.01f;

        // 1. Position SMC (hệ số quá tắt dần zeta > 1.0, ổn định dưới nhiễu StateEstimator & trễ 100Hz)
        const float ep1_x = DroneMath::clamp(st.x - p_des.x, -2.5f, 2.5f);
        const float ep1_y = DroneMath::clamp(st.y - p_des.y, -2.5f, 2.5f);
        const float ep1_z = DroneMath::clamp(st.z - p_des.z, -2.5f, 2.5f);

        const float ep2_x = DroneMath::clamp(st.vx_w - dp_des.x, -3.5f, 3.5f);
        const float ep2_y = DroneMath::clamp(st.vy_w - dp_des.y, -3.5f, 3.5f);
        const float ep2_z = DroneMath::clamp(st.vz_w - dp_des.z, -3.5f, 3.5f);

        constexpr float alpha_pxy = 0.65f;
        constexpr float kpi1_xy   = 1.40f;
        constexpr float kpi2_xy   = 0.25f;

        constexpr float alpha_pz  = 1.00f;
        constexpr float kpi1_z    = 1.60f;
        constexpr float kpi2_z    = 0.35f;

        const float sp_x = ep2_x + alpha_pxy * ep1_x;
        const float sp_y = ep2_y + alpha_pxy * ep1_y;
        const float sp_z = ep2_z + alpha_pz  * ep1_z;

        z_int_ = DroneMath::clamp(z_int_ + ep1_z * dt, -1.5f, 1.5f);
        float ux = -MASS_KG * (-ddp_des.x + alpha_pxy * ep2_x + kpi1_xy * sp_x + kpi2_xy * std::tanh(sp_x / 1.0f));
        float uy = -MASS_KG * (-ddp_des.y + alpha_pxy * ep2_y + kpi1_xy * sp_y + kpi2_xy * std::tanh(sp_y / 1.0f));
        float uz =  MASS_KG * GRAVITY - MASS_KG * (-ddp_des.z + alpha_pz * ep2_z + kpi1_z * sp_z + kpi2_z * std::tanh(sp_z / 0.8f) + 0.6f * z_int_);

        // Tuyệt đối giữ thăng bằng thẳng đứng khi chân đáp chưa rời mặt đất (z < 0.45m)
        if (st.z < 0.45f) {
            ux = 0.0f;
            uy = 0.0f;
        }

        uz = DroneMath::clamp(uz, MASS_KG * GRAVITY * 0.45f, MAX_TOTAL_THRUST_N * 0.85f);
        const float cos_tilt = std::max(0.85f, std::cos(st.roll) * std::cos(st.pitch));
        uz = DroneMath::clamp(uz / cos_tilt, MASS_KG * GRAVITY * 0.45f, MAX_TOTAL_THRUST_N * 0.85f);
        const float max_horiz = uz * std::tan(15.0f * static_cast<float>(M_PI) / 180.0f);
        const float h_norm = std::sqrt(ux * ux + uy * uy);
        if (h_norm > max_horiz && h_norm > 1e-4f) {
            const float sc = max_horiz / h_norm;
            ux *= sc;
            uy *= sc;
        }

        const float uT = std::sqrt(ux * ux + uy * uy + uz * uz);
        const float cy = std::cos(st.yaw);
        const float sy = std::sin(st.yaw);
        const float u_xb =  cy * ux + sy * uy;
        const float u_yb = -sy * ux + cy * uy;

        const float phi_ref   = std::asin(DroneMath::clamp(-u_yb / std::max(uT, 1.0f), -0.26f, 0.26f));
        const float theta_ref = DroneMath::clamp(std::atan2(u_xb, std::max(uz, 1.0f)), -0.26f, 0.26f);
        const float psi_ref   = 0.0f;

        const Vec3 eta_ref{phi_ref, theta_ref, psi_ref};
        if (!hgo_init_) {
            eta_des_ = eta_ref;
            deta_des_ = {};
            ddeta_des_ = {};
            hgo_init_ = true;
        }

        // 2. High-Gain Observer (hgo_3_dynamics.m: c1=3, c2=3, c3=1) với eps=0.30s ổn định dưới trễ 100Hz Gazebo
        constexpr float c1 = 3.0f, c2 = 3.0f, c3 = 1.0f;
        constexpr float eps = 0.30f;
        const int substeps = std::max(1, static_cast<int>(std::ceil(dt / 0.002f)));
        const float dt_sub = dt / static_cast<float>(substeps);
        for (int s = 0; s < substeps; ++s) {
            Vec3 err_hgo{
                eta_ref.x - eta_des_.x,
                eta_ref.y - eta_des_.y,
                DroneMath::normalize_angle(eta_ref.z - eta_des_.z)
            };
            Vec3 d1{
                deta_des_.x + (c1 / eps) * err_hgo.x,
                deta_des_.y + (c1 / eps) * err_hgo.y,
                deta_des_.z + (c1 / eps) * err_hgo.z
            };
            Vec3 d2{
                ddeta_des_.x + (c2 / (eps * eps)) * err_hgo.x,
                ddeta_des_.y + (c2 / (eps * eps)) * err_hgo.y,
                ddeta_des_.z + (c2 / (eps * eps)) * err_hgo.z
            };
            Vec3 d3{
                (c3 / (eps * eps * eps)) * err_hgo.x,
                (c3 / (eps * eps * eps)) * err_hgo.y,
                (c3 / (eps * eps * eps)) * err_hgo.z
            };
            eta_des_.x   += d1.x * dt_sub;
            eta_des_.y   += d1.y * dt_sub;
            eta_des_.z    = DroneMath::normalize_angle(eta_des_.z + d1.z * dt_sub);
            deta_des_.x   = DroneMath::clamp(deta_des_.x + d2.x * dt_sub, -2.5f, 2.5f);
            deta_des_.y   = DroneMath::clamp(deta_des_.y + d2.y * dt_sub, -2.5f, 2.5f);
            deta_des_.z   = DroneMath::clamp(deta_des_.z + d2.z * dt_sub, -1.5f, 1.5f);
            ddeta_des_.x  = DroneMath::clamp(ddeta_des_.x + d3.x * dt_sub, -3.0f, 3.0f);
            ddeta_des_.y  = DroneMath::clamp(ddeta_des_.y + d3.y * dt_sub, -3.0f, 3.0f);
            ddeta_des_.z  = DroneMath::clamp(ddeta_des_.z + d3.z * dt_sub, -1.5f, 1.5f);
        }

        // 3. Attitude SMC (ổn định pha dưới trễ Gazebo 100Hz + Motor lag, giảm khuếch đại nhiễu HGO)
        const float e_eta1_r = DroneMath::clamp(st.roll  - eta_des_.x, -1.0f, 1.0f);
        const float e_eta1_p = DroneMath::clamp(st.pitch - eta_des_.y, -1.0f, 1.0f);
        const float e_eta1_y = DroneMath::clamp(DroneMath::normalize_angle(st.yaw - eta_des_.z), -1.5f, 1.5f);

        const float e_eta2_r = DroneMath::clamp(st.p - 0.4f * deta_des_.x, -6.0f, 6.0f);
        const float e_eta2_p = DroneMath::clamp(st.q - 0.4f * deta_des_.y, -6.0f, 6.0f);
        const float e_eta2_y = DroneMath::clamp(st.r - 0.4f * deta_des_.z, -4.0f, 4.0f);

        constexpr float alpha_e = 4.0f;
        constexpr float kei1    = 5.5f;
        constexpr float kei2    = 0.35f;

        const float s_r = e_eta2_r + alpha_e * e_eta1_r;
        const float s_p = e_eta2_p + alpha_e * e_eta1_p;
        const float s_y = e_eta2_y + alpha_e * e_eta1_y;

        const float f_phi   = -(JY - JZ) / JX * st.q * st.r;
        const float f_theta = -(JX - JZ) / JY * st.p * st.r;
        const float f_psi   = -(JY - JX) / JZ * st.p * st.q;

        const float tau_r = -JX * (f_phi   + alpha_e * e_eta2_r + kei1 * s_r + kei2 * std::tanh(s_r / 0.35f));
        const float tau_p = -JY * (f_theta + alpha_e * e_eta2_p + kei1 * s_p + kei2 * std::tanh(s_p / 0.35f));
        const float tau_y = -JZ * (f_psi   + alpha_e * e_eta2_y + kei1 * s_y + kei2 * std::tanh(s_y / 0.35f));

        // 4. MotorMixer chuẩn X500 + Slew Rate Limiter
        const float thrust_norm = DroneMath::clamp(uT / MAX_TOTAL_THRUST_N, 0.10f, 0.85f);
        const float roll_norm   = DroneMath::clamp(tau_r / MIXER_ROLL_SCALE_NM,  -0.25f, 0.25f);
        const float pitch_norm  = DroneMath::clamp(tau_p / MIXER_PITCH_SCALE_NM, -0.25f, 0.25f);
        const float yaw_norm    = DroneMath::clamp(tau_y / MIXER_YAW_SCALE_NM,   -0.25f, 0.25f);

        std::array<float, 4> u = {
            thrust_norm - roll_norm - pitch_norm - yaw_norm,
            thrust_norm + roll_norm + pitch_norm - yaw_norm,
            thrust_norm + roll_norm - pitch_norm + yaw_norm,
            thrust_norm - roll_norm + pitch_norm + yaw_norm
        };
        float max_u = *std::max_element(u.begin(), u.end());
        if (max_u > 1.0f) {
            for (auto& v : u) v -= (max_u - 1.0f);
        }
        float min_u = *std::min_element(u.begin(), u.end());
        if (min_u < 0.0f) {
            for (auto& v : u) v += (-min_u);
        }
        constexpr float max_step = 1500.0f;
        for (size_t i = 0; i < 4; ++i) {
            float target_w = 1000.0f * std::sqrt(DroneMath::clamp(u[i], 0.0f, 1.0f));
            float dw = DroneMath::clamp(target_w - motor_speeds_[i], -max_step * dt, max_step * dt);
            motor_speeds_[i] = DroneMath::clamp(motor_speeds_[i] + dw, 0.0f, 1000.0f);
        }
        return motor_speeds_;
    }

private:
    Vec3 eta_des_{};
    Vec3 deta_des_{};
    Vec3 ddeta_des_{};
    bool hgo_init_{false};
    float z_int_{0.0f};
    std::array<float, 4> motor_speeds_{0.0f, 0.0f, 0.0f, 0.0f};
};

/**
 * @brief Bộ điều phối Đội hình phân tán 4 Drone (Distributed Consensus Formation từ a_4_drone.m)
 */
class DistributedFormationCoordinator {
public:
    DistributedFormationCoordinator() {
        reset();
    }

    void reset() {
        // Độ lệch đội hình kim cương chuẩn a_4_drone.m
        offsets_[0] = { 4.0f,  0.0f, 0.0f};
        offsets_[1] = { 0.0f,  4.0f, 0.0f};
        offsets_[2] = {-4.0f,  0.0f, 0.0f};
        offsets_[3] = { 0.0f, -4.0f, 0.0f};

        // Khởi tạo p_des_i tại vị trí xuất phát thực tế của 4 drone
        p_des_[0] = { 3.0f,  2.0f, 0.12f};
        p_des_[1] = {-2.0f,  3.0f, 0.12f};
        p_des_[2] = {-3.0f, -2.0f, 0.12f};
        p_des_[3] = { 2.0f, -3.0f, 0.12f};

        for (int k = 0; k < 4; ++k) {
            dp_des_[k] = {};
            ddp_des_[k] = {};
            smc_hgo_[k].reset(p_des_[k]);
        }
        leader_pos_ = {0.0f, 0.0f, 0.12f};
        leader_vel_ = {};
        sim_time_ = 0.0f;
    }

    void sync_start_positions(const std::array<DroneState, 4>& st) {
        reset();
        for (int k = 0; k < 4; ++k) {
            p_des_[k] = {st[k].x, st[k].y, std::max(0.12f, st[k].z)};
            dp_des_[k] = {};
            ddp_des_[k] = {};
            smc_hgo_[k].reset(p_des_[k]);
        }
        leader_pos_ = {0.0f, 0.0f, std::max(0.12f, st[0].z)};
        leader_vel_ = {};
        sim_time_ = 0.0f;
    }

    void step_landing(float dt) {
        if (dt <= 0.0005f || dt > 0.1f) dt = 0.01f;
        sim_time_ += dt;
        for (int k = 0; k < 4; ++k) {
            p_des_[k].z = std::max(0.10f, p_des_[k].z - 0.60f * dt);
            dp_des_[k] = {0.0f, 0.0f, -0.60f};
            ddp_des_[k] = {};
        }
        leader_pos_.z = std::max(0.10f, leader_pos_.z - 0.60f * dt);
        leader_vel_ = {0.0f, 0.0f, -0.60f};
    }

    void step_consensus(float dt) {
        if (dt <= 0.0005f || dt > 0.1f) dt = 0.01f;
        sim_time_ += dt;

        // 1. Quỹ đạo Leader ảo p_0(t):
        //    0..4s: Cất cánh thẳng đứng tại chỗ lên Z=4.0m (chưa chạy đồng thuận ngang để tránh nghiêng trên mặt đất)
        //    4..10s: Lên Z=5.0m và xếp đội hình kim cương trên không
        //    >=10s: Bay vòng tròn bán kính 4.0m
        constexpr float z_cruise = 5.0f;
        if (sim_time_ < 4.0f) {
            float z_takeoff = 0.12f + (4.0f - 0.12f) * (sim_time_ / 4.0f);
            float vz_takeoff = (4.0f - 0.12f) / 4.0f;
            leader_pos_ = {0.0f, 0.0f, z_takeoff};
            leader_vel_ = {0.0f, 0.0f, vz_takeoff};
            for (int k = 0; k < 4; ++k) {
                p_des_[k].z   = z_takeoff;
                dp_des_[k].z  = vz_takeoff;
                ddp_des_[k].z = 0.0f;
                dp_des_[k].x  = 0.0f;
                dp_des_[k].y  = 0.0f;
                ddp_des_[k].x = 0.0f;
                ddp_des_[k].y = 0.0f;
            }
            return;
        } else if (sim_time_ < 10.0f) {
            float s = (sim_time_ - 4.0f) / 6.0f;
            leader_pos_ = {0.0f, 0.0f, 4.0f + (z_cruise - 4.0f) * s};
            leader_vel_ = {0.0f, 0.0f, (z_cruise - 4.0f) / 6.0f};
        } else {
            float t_orb = sim_time_ - 10.0f;
            constexpr float R = 4.0f;
            constexpr float w = 0.15f;
            leader_pos_ = {R * std::sin(w * t_orb), R * (std::cos(w * t_orb) - 1.0f), z_cruise};
            leader_vel_ = {R * w * std::cos(w * t_orb), -R * w * std::sin(w * t_orb), 0.0f};
        }

        // 2. Đồng thuận phân tán đồ thị 4 Drone (a_4_drone.m lines 186-195, tích phân sub-step 2ms)
        constexpr float k1i = 0.8f;
        constexpr float k2i = 1.8f;
        constexpr float b[4] = {1.0f, 0.0f, 1.0f, 0.0f}; // Leader nối với Drone 1 và Drone 3

        const int substeps = std::max(1, static_cast<int>(std::ceil(dt / 0.002f)));
        const float dt_sub = dt / static_cast<float>(substeps);

        for (int s = 0; s < substeps; ++s) {
            std::array<Vec3, 4> next_dp_des{};
            std::array<Vec3, 4> cur_ddp_des{};

            for (int k = 0; k < 4; ++k) {
                int prev_n = (k + 3) % 4;
                int next_n = (k + 1) % 4;

                Vec3 rel_k = p_des_[k] - offsets_[k];
                Vec3 rel_p = p_des_[prev_n] - offsets_[prev_n];
                Vec3 rel_n = p_des_[next_n] - offsets_[next_n];

                Vec3 consensus_p = (rel_k - rel_n) + (rel_k - rel_p) + (rel_k - leader_pos_) * b[k];
                Vec3 consensus_v = (dp_des_[k] - dp_des_[next_n]) + (dp_des_[k] - dp_des_[prev_n]) + (dp_des_[k] - leader_vel_) * b[k];

                next_dp_des[k] = dp_des_[k] - consensus_p * k1i;
                cur_ddp_des[k] = consensus_v * (-k2i);
            }

            for (int k = 0; k < 4; ++k) {
                ddp_des_[k] = cur_ddp_des[k];
                dp_des_[k]  = dp_des_[k] + cur_ddp_des[k] * dt_sub;
                p_des_[k]   = p_des_[k]  + next_dp_des[k] * dt_sub;
            }
        }
    }

    std::array<float, 4> compute_drone_motors(int idx, const DroneState& st, float dt, bool armed = true) {
        return smc_hgo_[idx].update(st, p_des_[idx], dp_des_[idx], ddp_des_[idx], dt, armed);
    }

    // Tính lệnh vận tốc (Body X/Y + World Z) để Drone 1 chạy HITL qua ESP32 thật bám p_des_[0]
    void compute_esp32_velocity_cmds(
        const DroneState& st0,
        float& out_cmd_vx_body,
        float& out_cmd_vy_body,
        float& out_cmd_vz_world
    ) const {
        const float kp_xy = 0.85f;
        const float kp_z  = 1.2f;
        const float vx_w = DroneMath::clamp(dp_des_[0].x + kp_xy * (p_des_[0].x - st0.x), -1.6f, 1.6f);
        const float vy_w = DroneMath::clamp(dp_des_[0].y + kp_xy * (p_des_[0].y - st0.y), -1.6f, 1.6f);
        out_cmd_vz_world = DroneMath::clamp(dp_des_[0].z + kp_z  * (p_des_[0].z - st0.z), -1.8f, 1.8f);

        const float cy = std::cos(st0.yaw);
        const float sy = std::sin(st0.yaw);
        out_cmd_vx_body =  cy * vx_w + sy * vy_w;
        out_cmd_vy_body = -sy * vx_w + cy * vy_w;
    }

    const Vec3& get_leader_pos() const { return leader_pos_; }
    const Vec3& get_p_des(int idx) const { return p_des_[idx]; }
    const Vec3& get_dp_des(int idx) const { return dp_des_[idx]; }
    float get_sim_time() const { return sim_time_; }

private:
    std::array<Vec3, 4> offsets_{};
    std::array<Vec3, 4> p_des_{};
    std::array<Vec3, 4> dp_des_{};
    std::array<Vec3, 4> ddp_des_{};
    std::array<SMCHGOController, 4> smc_hgo_{};
    Vec3 leader_pos_{};
    Vec3 leader_vel_{};
    float sim_time_{0.0f};
};

} // namespace DroneFormation
