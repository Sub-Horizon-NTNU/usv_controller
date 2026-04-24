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
    
    private:
        std::chrono::steady_clock::time_point prev_time_;
        
        double kp_;
        double ki_;
        double kd_;
        double prev_error_;
        double max_output_;
        double i_;


};