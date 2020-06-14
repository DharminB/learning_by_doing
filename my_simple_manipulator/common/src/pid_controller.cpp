#include <my_simple_manipulator/pid_controller.h>
#include <math.h>
#include <iostream>

PIDController::PIDController(float proportional, float integral, float differential,
              float control_time, float tolerance, float i_clamp, bool debug):
    proportional_factor_(proportional),
    integral_factor_(integral),
    differential_factor_(differential),
    tolerance_(tolerance),
    i_clamp_(i_clamp),
    debug_(debug)
{
    delta_time_ = 1.0/control_time;
    integral_sum_ = 0.0;
    prev_error_ = 0.0;
}

float PIDController::control(float current, float target)
{
    float current_error = target - current;
    if (fabs(current_error) < tolerance_)
    {
        return 0.0;
    }

    integral_sum_ += current_error * delta_time_;
    integral_sum_ = fmax(-1.0 * i_clamp_, fmin(i_clamp_, integral_sum_));
    float change_in_error = (current_error - prev_error_) / delta_time_;
    prev_error_ = current_error;
    float control_value = proportional_factor_   * current_error
                          + integral_factor_     * integral_sum_
                          + differential_factor_ * change_in_error;
    if ( debug_ )
    {
        std::cout << "current_error " << current_error << std::endl;
        std::cout << "integral_sum " << integral_sum_ << std::endl;
        std::cout << "change_in_error " << change_in_error << std::endl;
        std::cout << "p " << proportional_factor_ * current_error << std::endl;
        std::cout << "i " << integral_factor_ * integral_sum_ << std::endl;
        std::cout << "d " << differential_factor_ * change_in_error << std::endl;
        std::cout << "control_value " << control_value << std::endl;
    }
    return control_value;
}

void PIDController::reset()
{
    prev_error_ = 0.0;
    integral_sum_ = 0.0;
    if ( debug_ )
    {
        std::cout << "Resetting" << std::endl;
    }
}
