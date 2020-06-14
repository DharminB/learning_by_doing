class PIDController
{
    private:
        float delta_time_;
        float proportional_factor_;
        float integral_factor_;
        float differential_factor_;
        float tolerance_;
        float i_clamp_;
        float integral_sum_;
        float prev_error_;
        bool debug_;

    public:
        PIDController(float proportional, float integral, float differential,
                      float control_time, float tolerance, float i_clamp, bool debug=false);
        
        float control(float current, float target);

        void reset();
};
