#pragma once

#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "Structures.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include "PID.hpp"

class USV{
    public:
    USV(const double &kp,const double &ki,const double &kd, const float &max_velocity){

        this->pid_x_ = std::make_unique<PID>(kp,ki,kd,max_velocity);
        this->pid_y_ = std::make_unique<PID>(kp,ki,kd,max_velocity);
        this->pid_heading_ = std::make_unique<PID>(kp,ki,kd,max_velocity);
    }
    void set_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose){
        current_states_.x = pose->pose.position.y;
        current_states_.y = pose->pose.position.x;

        tf2::Quaternion quat_tf;
        geometry_msgs::msg::Quaternion quat_msg = pose->pose.orientation;
        tf2::fromMsg(quat_msg, quat_tf);
        double r{}, p{}, y{};
        tf2::Matrix3x3 m(quat_tf);
        m.getRPY(r, p, y);

        current_states_.heading = y;
    }

    void set_target_pose(float x, float y, float heading){
        target_states_.x = x;
        target_states_.y = y;
        target_states_.heading = heading;
    }

    void update(){
        target_states_.vx = target_states_.x-current_states_.x



       velocity_x_ = pid_x_->update(target_states_.x, current_states_.x);
       velocity_y_ = pid_y_->update(target_states_.y, current_states_.y);
       angular_velocity_z_ =  pid_heading_->update(target_states_.heading, current_states_.heading);
    }

    geometry_msgs::msg::TwistStamped  get_velocity_cmd(){
        geometry_msgs::msg::TwistStamped vel_cmd;
        vel_cmd.twist.linear.x = velocity_y_;
        vel_cmd.twist.linear.y = velocity_x_;
        vel_cmd.twist.angular.z = angular_velocity_z_;
        return vel_cmd;
    }
    
    States get_states(){
        return current_states_;
    }

    double get_position_x(){
        return current_states_.x;
    }

    double get_position_y(){
        return current_states_.y;
    }

    private:
        States current_states_;
        States target_states_;

        float velocity_x_;
        float velocity_y_;
        float angular_velocity_z_;

        std::unique_ptr<PID> pid_x_;
        std::unique_ptr<PID> pid_y_;
        std::unique_ptr<PID> pid_heading_;
        


};