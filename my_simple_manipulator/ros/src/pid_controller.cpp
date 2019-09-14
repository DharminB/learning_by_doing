#include <math.h>

class PIDController
{
    private:
        double delta_time;
        double proportional_factor;
        double integral_factor;
        double differential_factor;
        double tolerance;
        double integral_sum = 0.0;
        double prev_error = 0.0;

    public:
        PIDController(double proportional, double integral, double differential, double control_time, double tolerance)
        {
            this->proportional_factor = proportional;
            this->integral_factor = integral;
            this->differential_factor = differential;
            this->delta_time = 1.0/control_time;
            this->tolerance = tolerance;
        };
        
        double control(double current, double target)
        {
            double current_error = target - current;
            if (fabs(current_error) < this->tolerance)
                return 0.0;
            this->integral_sum += current_error * this->delta_time;
            double change_in_error = (current_error - this->prev_error) / this->delta_time;
            this->prev_error = current_error;
            double control_value = this->proportional_factor * current_error
                                    + this -> integral_factor * this->integral_sum
                                    + this->differential_factor * change_in_error;
            return control_value;
        };

        void reset()
        {
            this->prev_error = 0.0;
            this->integral_sum = 0.0;
        };
};
