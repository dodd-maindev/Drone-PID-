#pragma once
#include <algorithm>

namespace DroneControllers {

/**
 * @brief Bộ điều khiển PID thuần toán học (Proportional - Integral - Derivative)
 */
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

        if (dt <= 0.0) dt = 0.001;

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

    void set_gains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_{0.0}, ki_{0.0}, kd_{0.0};
    double output_min_{0.0}, output_max_{1.0};
    double integral_max_{0.5};
    double integral_{0.0};
    double prev_error_{0.0};
    bool first_run_{true};
};

} // namespace DroneControllers
