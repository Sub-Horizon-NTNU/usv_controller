#pragma once
#include <chrono>
#include <cmath>


class PID{
    public:
    PID(const double &kp,const double &ki,const double &kd, double max_output) 
    : prev_time_(std::chrono::steady_clock::now()), kp_(kp), ki_(ki), kd_(kd), prev_error_(0), max_output_(max_output), i_(0.0)
    {
    

    }


    void set_setpoint(double setpoint){
        setpoint_ = setpoint;
    }
    
    float update(double setpoint, double current){
        float error = setpoint_-current;
        float dt = (std::chrono::steady_clock::now()-prev_time_).count();
        prev_time_ = std::chrono::steady_clock::now();

        float p = kp_*error;
        i_ += ki_*error*dt;
        float d = kd_*(error-prev_error_)/dt;
        float output = p+i_+d;



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


    private:
        std::chrono::steady_clock::time_point prev_time_;
        
        float setpoint_{};
        float kp_;
        float ki_;
        float kd_;
        float prev_error_;
        float max_output_;
        float i_;


};