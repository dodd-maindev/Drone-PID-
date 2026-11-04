/**
 * @file itsmc_controller.hpp
 * @brief Bộ điều khiển Trượt Tích phân Đầu cuối (Integral Terminal Sliding Mode Control - ITSMC)
 * 
 * Thuật toán chuyển đổi từ mô hình nghiên cứu MATLAB (itsmc.m):
 *   1. Mặt trượt phi tuyến:
 *      s = de + k11 * e + k12 * int(|e|^alpha * sign(e) dt)
 *   2. Thành phần điều khiển tương đương:
 *      u_eq = k11 * de + k12 * |e|^alpha * sign(e)
 *   3. Thành phần điều khiển chuyển mạch trơn với lớp biên (boundary layer):
 *      u_sw = k2 * s + k3 * tanh(s / phi_b)
 *   4. Tín hiệu điều khiển chuẩn hóa qua nghịch đảo ma trận điều khiển Be^-1:
 *      u = b_inv * (u_eq + u_sw)
 * 
 * Tối ưu hóa cho ESP32 & x500 Quadrotor:
 *   - Với alpha = 0.5, hàm |e|^0.5 * sign(e) được tính bằng sqrtf() trực tiếp trên phần cứng FPU của ESP32.
 *   - Tuyến tính hóa vùng lân cận |e| < eps_lin để triệt tiêu hiện tượng đạo hàm vô cùng khi e -> 0 (Khử hoàn toàn dao động tự kích hình sin 4Hz / Limit cycle).
 *   - Bổ sung hệ số b_inv (Be^-1 trong itsmc.m) để chuẩn hóa lực ngõ ra phù hợp với actuator limits của x500.
 *   - Bổ sung bề dày lớp biên phi_b trong hàm tanh(s / phi_b) nhằm triệt tiêu hoàn toàn rung chấn vi phân (chattering).
 *   - Tích hợp Anti-windup cho tích phân mặt trượt và kẹp bão hòa ngõ ra (Output Clamp).
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace DroneControl {

class ITSMCController {
public:
    struct Config {
        float alpha{0.5f};          // Số mũ phân số (alpha = 0.5 theo itsmc.m)
        float k11{12.0f};           // Hệ số sai số trạng thái k11
        float k12{0.30f};           // Hệ số tích phân mặt trượt k12
        float k2{8.0f};             // Hệ số chuyển mạch tuyến tính k2
        float k3{0.15f};            // Hệ số chuyển mạch phi tuyến tanh k3
        float b_inv{0.0040f};       // Nghịch đảo ma trận điều khiển Be^-1 (Chuẩn hóa thang mô-men mixer)
        float phi_b{0.15f};         // Độ dày lớp biên mặt trượt (Boundary layer) triệt tiêu chattering
        float eps_lin{0.035f};      // Vùng chuyển tiếp tuyến tính quanh e=0 khử dao động hình sin 4Hz (rad)
        float max_integral{0.20f};  // Giới hạn tích phân chống bão hòa mặt trượt (Anti-windup)
        float max_output{0.25f};    // Giới hạn mô-men / lực ngõ ra
    };

    ITSMCController() = default;
    explicit ITSMCController(const Config& cfg) : cfg_(cfg) {}

    ITSMCController(float k11, float k12, float k2, float k3, float b_inv, 
                    float phi_b = 0.15f, float eps_lin = 0.035f,
                    float alpha = 0.5f, float max_integral = 0.20f, float max_output = 0.25f) {
        cfg_.k11 = k11;
        cfg_.k12 = k12;
        cfg_.k2 = k2;
        cfg_.k3 = k3;
        cfg_.b_inv = b_inv;
        cfg_.phi_b = phi_b;
        cfg_.eps_lin = eps_lin;
        cfg_.alpha = alpha;
        cfg_.max_integral = max_integral;
        cfg_.max_output = max_output;
    }

    void set_config(const Config& cfg) { cfg_ = cfg; }
    const Config& get_config() const { return cfg_; }

    void reset() {
        integral_sign_ = 0.0f;
        s_last_ = 0.0f;
    }

    /**
     * @brief Cập nhật tính toán ITSMC từ giá trị đặt và giá trị đo
     * @param target Giá trị đặt mong muốn (Set-point)
     * @param current Giá trị thực tế đo từ cảm biến (Measurement)
     * @param current_rate Tốc độ biến thiên thực tế (p, q, r hoặc vz)
     * @param dt Thời gian chu kỳ (giây)
     * @return Tín hiệu điều khiển u (Torque hoặc Delta Thrust)
     */
    float update(float target, float current, float current_rate, float dt) {
        float error = target - current;
        return update_error(error, current_rate, dt);
    }

    /**
     * @brief Cập nhật tính toán ITSMC trực tiếp từ sai số (Dành cho góc Yaw đã chuẩn hóa [-pi, pi])
     * @param error Sai số e = target - current
     * @param current_rate Tốc độ biến thiên d(current)/dt
     * @param dt Thời gian chu kỳ (giây)
     * @return Tín hiệu điều khiển u
     */
    float update_error(float error, float current_rate, float dt) {
        if (dt <= 0.0001f || dt > 0.1f) dt = 0.005f;

        // 1. Đạo hàm sai số: de = d(target - current)/dt = -current_rate (với target tĩnh / biến thiên chậm)
        float de = -current_rate;

        // 2. Tính số hạng phi tuyến fractional: |e|^alpha * sign(e)
        // Với alpha = 0.5: tính bằng sqrtf() trực tiếp trên phần cứng FPU ESP32 chỉ mất 1 chu kỳ máy
        // Áp dụng kỹ thuật chuyển tiếp tuyến tính (Non-singular linear boundary) khi |e| < eps_lin
        // để tránh hiện tượng đạo hàm tiến tới vô cùng khi e -> 0 gây dao động sóng sin tự kích (Limit cycle).
        float abs_e = std::abs(error);
        float frac_e = 0.0f;
        if (abs_e > 1e-6f) {
            if (cfg_.eps_lin > 1e-4f && abs_e < cfg_.eps_lin) {
                // Vùng tuyến tính trơn: f(e) = e / sqrt(eps_lin)
                frac_e = error / std::sqrt(cfg_.eps_lin);
            } else {
                if (cfg_.alpha == 0.5f) {
                    frac_e = std::sqrt(abs_e) * ((error > 0.0f) ? 1.0f : -1.0f);
                } else {
                    frac_e = std::pow(abs_e, cfg_.alpha) * ((error > 0.0f) ? 1.0f : -1.0f);
                }
            }
        }

        // 3. Tích lũy tích phân mặt trượt với Anti-Windup
        integral_sign_ += frac_e * dt;
        integral_sign_ = std::clamp(integral_sign_, -cfg_.max_integral, cfg_.max_integral);

        // 4. Giá trị mặt trượt s: s = de + k11 * e + k12 * int(|e|^alpha * sign(e))
        float s = de + cfg_.k11 * error + cfg_.k12 * integral_sign_;
        s_last_ = s;

        // 5. Thành phần điều khiển tương đương u_eq (theo đúng công thức trong itsmc.m)
        float u_eq = cfg_.k11 * de + cfg_.k12 * frac_e;

        // 6. Thành phần điều khiển chuyển mạch u_sw (sử dụng tanh(s / phi_b) với lớp biên làm trơn chống chattering)
        float phi_b_safe = (cfg_.phi_b > 1e-4f) ? cfg_.phi_b : 0.15f;
        float u_sw = cfg_.k2 * s + cfg_.k3 * std::tanh(s / phi_b_safe);

        // 7. Tổng hợp tín hiệu điều khiển chuẩn hóa qua nghịch đảo ma trận điều khiển b_inv (Be^-1 trong itsmc.m)
        float u = cfg_.b_inv * (u_eq + u_sw);

        // Kẹp bão hòa ngõ ra an toàn
        return std::clamp(u, -cfg_.max_output, cfg_.max_output);
    }

    float get_sliding_surface() const { return s_last_; }
    float get_integral() const { return integral_sign_; }

private:
    Config cfg_;
    float integral_sign_{0.0f};
    float s_last_{0.0f};
};

/**
 * @brief Bộ điều khiển độ cao sử dụng thuật toán ITSMC (ITSMC Altitude Controller)
 */
class ITSMCAltitudeController {
public:
    ITSMCAltitudeController(float hover_thrust, float k11, float k12, float k2, float k3, float b_inv,
                            float min_thrust, float max_thrust, float phi_b = 0.20f, float eps_lin = 0.05f)
        : hover_thrust_(hover_thrust),
          min_thrust_(min_thrust),
          max_thrust_(max_thrust),
          itsmc_(k11, k12, k2, k3, b_inv, phi_b, eps_lin, 0.5f, 0.25f, 0.35f) {}

    void reset() {
        itsmc_.reset();
    }

    float compute_thrust(float target_alt, float current_alt, float vz, float dt) {
        // Tín hiệu delta thrust bù độ cao từ ITSMC (đã tích hợp giảm chấn vận tốc vz và khử sai số tĩnh)
        float delta_thrust = itsmc_.update(target_alt, current_alt, vz, dt);

        // Lực nâng tổng = Lực treo hover + Delta ITSMC
        float total_thrust = hover_thrust_ + delta_thrust;

        return std::clamp(total_thrust, min_thrust_, max_thrust_);
    }

    float get_sliding_surface() const { return itsmc_.get_sliding_surface(); }

private:
    float hover_thrust_{0.59f};
    float min_thrust_{0.10f};
    float max_thrust_{0.85f};
    ITSMCController itsmc_;
};

} // namespace DroneControl
