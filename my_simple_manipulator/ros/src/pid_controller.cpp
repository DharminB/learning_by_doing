#include <math.h>
#include <iostream>

class PIDController
{
    private:
        float delta_time;
        float proportional_factor;
        float integral_factor;
        float differential_factor;
        float tolerance;
        float i_clamp;
        float integral_sum = 0.0;
        float prev_error = 0.0;

    public:
        PIDController(float proportional, float integral, float differential,
                      float control_time, float tolerance, float i_clamp)
        {
            this->proportional_factor = proportional;
            this->integral_factor = integral;
            this->differential_factor = differential;
            this->delta_time = 1.0/control_time;
            this->tolerance = tolerance;
            this->i_clamp = i_clamp;
        };
        
        float control(float current, float target)
        {
            float current_error = target - current;
            if (fabs(current_error) < this->tolerance)
                return 0.0;
            this->integral_sum += current_error * this->delta_time;
            this->integral_sum = fmax(-1.0 * this->i_clamp, fmin(this->i_clamp, this->integral_sum));
            float change_in_error = (current_error - this->prev_error) / this->delta_time;
            this->prev_error = current_error;
            float control_value = this->proportional_factor * current_error
                                    + this -> integral_factor * this->integral_sum
                                    + this->differential_factor * change_in_error;
            /* std::cout << "current_error " << current_error << std::endl; */
            /* std::cout << "integral_sum " << this->integral_sum << std::endl; */
            /* std::cout << "change_in_error " << change_in_error << std::endl; */
            /* std::cout << "p " << this->proportional_factor * current_error << std::endl; */
            /* std::cout << "i " << this->integral_factor * this->integral_sum << std::endl; */
            /* std::cout << "d " << this->differential_factor * change_in_error << std::endl; */
            /* std::cout << "control_value " << control_value << std::endl; */
            return control_value;
        };

        void reset()
        {
            this->prev_error = 0.0;
            this->integral_sum = 0.0;
        };
};
