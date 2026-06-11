#include "usv_controller/PID.hpp"

    PID::PID() 
    : prev_time_(std::chrono::steady_clock::now()), prev_error_(0), i_(0.0)
    {}

    double PID::update(const double &error){
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
        last_p_ = p;
        last_d_ = std::isnan(d) ? 0.0 : d;
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
    
    void PID::set_kp(const float &kp){
        kp_ = kp;
    }

    void PID::set_ki(const float &ki){
        ki_ = ki;
    }

    void PID::set_kd(const float &kd){
        kd_ = kd;
    }

    void PID::set_max_output(const float &max_output){
        max_output_ = max_output;
    }
