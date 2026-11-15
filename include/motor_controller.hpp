#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>

namespace rover {

/**
 * @brief Discrete-time PID controller for rover wheel velocity control.
 * Includes anti-windup clamping, derivative tracking, and convergence bounds.
 */
class MotorPIDController {
private:
    double target_speed_{0.0};
    double current_speed_{0.0};
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    
    double error_{0.0};
    double previous_error_{0.0};
    double integral_sum_{0.0};
    double max_integral_{50.0};
    double max_output_step_{30.0};

public:
    MotorPIDController(double target, double current, double p, double i, double d,
                       double max_integral = 50.0, double max_output_step = 30.0)
        : target_speed_(target),
          current_speed_(current),
          kp_(p),
          ki_(i),
          kd_(d),
          max_integral_(max_integral),
          max_output_step_(max_output_step)
    {
        error_ = target_speed_ - current_speed_;
        previous_error_ = error_;
        integral_sum_ = 0.0;
    }

    /**
     * @brief Computes a single discrete control step.
     * @param dt Time delta in seconds.
     * @return Updated current speed in RPM.
     */
    double step(double dt = 1.0) {
        if (dt <= 0.0) {
            dt = 1.0;
        }

        error_ = target_speed_ - current_speed_;

        // 1. Proportional term
        double p = kp_ * error_;

        // 2. Integral term with anti-windup clamp
        integral_sum_ += error_ * dt;
        integral_sum_ = std::clamp(integral_sum_, -max_integral_, max_integral_);
        double i = ki_ * integral_sum_;

        // 3. Derivative term
        double d_correction = kd_ * ((error_ - previous_error_) / dt);

        // 4. Combined control effort with output rate limiting
        double delta_v = p + i + d_correction;
        delta_v = std::clamp(delta_v, -max_output_step_, max_output_step_);

        // Apply velocity update
        current_speed_ += delta_v;
        previous_error_ = error_;
        error_ = target_speed_ - current_speed_;

        return current_speed_;
    }

    /**
     * @brief Executes PID steps until error falls within tolerance or max_iterations reached.
     * @param tolerance Error threshold for convergence.
     * @param max_iterations Maximum iterations to prevent runaway loops.
     * @param verbose Print step log if true.
     * @return true if stabilized within max_iterations, false otherwise.
     */
    bool run_until_stabilized(double tolerance = 0.1, int max_iterations = 100, bool verbose = true) {
        if (verbose) {
            std::cout << "--- Initializing Motor Node ---\n";
        }

        int iterations = 0;
        while (std::abs(error_) > tolerance && iterations < max_iterations) {
            step(1.0);
            iterations++;
            if (verbose) {
                std::cout << "Speed: " << current_speed_ << " RPM | Error: " << error_ << "\n";
            }
        }

        bool converged = std::abs(error_) <= tolerance;
        if (verbose) {
            if (converged) {
                std::cout << "Final Stabilized Speed: " << current_speed_ << " RPM\n";
            } else {
                std::cout << "Warning: PID did not converge within " << max_iterations << " iterations!\n";
            }
        }
        return converged;
    }

    // Getters and Setters
    double getCurrentSpeed() const { return current_speed_; }
    double getTargetSpeed() const { return target_speed_; }
    double getError() const { return error_; }
    double getIntegralSum() const { return integral_sum_; }

    void setTargetSpeed(double target) {
        target_speed_ = target;
        error_ = target_speed_ - current_speed_;
        previous_error_ = error_;
    }

    void reset(double current = 0.0) {
        current_speed_ = current;
        error_ = target_speed_ - current_speed_;
        previous_error_ = error_;
        integral_sum_ = 0.0;
    }
};

} // namespace rover
