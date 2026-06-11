#pragma once
#include <chrono>
#include <cmath>


class PID{
    public:
    PID();

    double update(const double &error);
    
    void set_kp(const float &kp);
    void set_ki(const float &ki);
    void set_kd(const float &kd);
    void set_max_output(const float &max_output);

    // Last computed term contributions (for debug/plotting).
    double get_p() const { return last_p_; }
    double get_i() const { return i_; }
    double get_d() const { return last_d_; }

    private:
        std::chrono::steady_clock::time_point prev_time_;
        
        double kp_;
        double ki_;
        double kd_;
        double prev_error_;
        double max_output_;
        double i_;
        double last_p_{0.0};
        double last_d_{0.0};


};