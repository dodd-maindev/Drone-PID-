#pragma once
#include <algorithm>
#include <chrono>

/**
 * @brief Bộ điều khiển PID (Proportional-Integral-Derivative) viết bằng C++ thuần.
 */
class PIDController {
public:
    PIDController(double kp, double ki, double kd, 
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
        // 1. Tính sai số: e(t) = target - actual
        double error = setpoint - measurement;

        if (first_run_) {
            prev_error_ = error;
            first_run_ = false;
        }

        if (dt <= 0.0) {
            dt = 0.001; // Tránh chia cho 0
        }

        // 2. Thành phần Tỉ lệ (P)
        double p_term = kp_ * error;

        // 3. Thành phần Tích phân (I) kèm Anti-Windup (chống bão hòa tích phân)
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -integral_max_, integral_max_);
        double i_term = ki_ * integral_;

        // 4. Thành phần Vi phân (D)
        double derivative = (error - prev_error_) / dt;
        double d_term = kd_ * derivative;

        // 5. Tổng hợp tín hiệu điều khiển: u(t) = P + I + D
        double output = p_term + i_term + d_term;
        prev_error_ = error;

        // 6. Giới hạn dải đầu ra (Clamping)
        return std::clamp(output, output_min_, output_max_);
    }

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double integral_max_;
    double integral_;
    double prev_error_;
    bool first_run_;
};
