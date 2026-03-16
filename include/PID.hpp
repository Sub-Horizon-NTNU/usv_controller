#pragma once
#include <chrono>
#include <cmath>


class PID{
    public:
    PID(const double &kp,const double &ki,const double &kd, const double &max_output) 
    : prev_time_(std::chrono::steady_clock::now()), kp_(kp), ki_(ki), kd_(kd), prev_error_(0), max_output_(max_output), i_(0.0)
    {}

    double update(const double &error){
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now-prev_time_).count();
        prev_time_ = now;

        double p = kp_*error;
        i_ += ki_*error*dt;
        if(i_ > max_output_){
            i_ = max_output_;
        }
        if(i_ < -max_output_){
            i_ = -max_output_;
        }

        double d = kd_*(error-prev_error_)/dt;
        prev_error_ = error;
        double output = p+i_+d;

        //Check if out of bounds
        if(std::abs(output)> max_output_){
            if(output > 0){
                output = max_output_;
            }   
            if(output < 0){
                output = -max_output_;
            }
        }
        if(std::isnan(output)){
            return 0.0;
        }
        return output;
    }
    
    void set_kp(const float &kp){
        kp_ = kp;
    }
    void set_ki(const float &ki){
        ki_ = ki;
    }
    void set_kd(const float &kd){
        kd_ = kd;
    }
    
    private:
        std::chrono::steady_clock::time_point prev_time_;
        
        double kp_;
        double ki_;
        double kd_;
        double prev_error_;
        double max_output_;
        double i_;


};