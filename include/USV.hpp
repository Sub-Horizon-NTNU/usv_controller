#pragma once

#include <cmath>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "Structures.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include "PID.hpp"

class USV{
    public:
    USV(const float &max_velocity, const double &kp,const double &ki,const double &kd, const float &max_angular_velocity, const float &braking_radius)
    : max_velocity_(max_velocity), braking_radius_(braking_radius)
    {
        this->pid_heading_ = std::make_unique<PID>(kp,ki,kd,max_angular_velocity);
    }
    void set_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose){
        //ENU TO NED
        current_states_.x = pose->pose.position.y;
        current_states_.y = pose->pose.position.x;

        // https://wiki.ros.org/tf2/Tutorials/Quaternions
        tf2::Quaternion quat_tf;
        geometry_msgs::msg::Quaternion quat_msg = pose->pose.orientation;
        tf2::fromMsg(quat_msg, quat_tf);
        //For rotating the quaternion..
        
        static tf2::Quaternion heading_rotate;
        heading_rotate.setRPY(0.0, 0.0, -M_PI/2);
        
        tf2::Quaternion heading_rot = heading_rotate*quat_tf;
        heading_rot.normalize();
        set_current_orientation(heading_rot);

        tf2::Matrix3x3 m_rot(heading_rot);
        double roll,pitch,yaw;
        m_rot.getRPY(roll, pitch, yaw);
        current_states_.heading = -yaw; // Sign introduced to follow NED conventions
        
    }

    void set_target_pose(float x, float y, float heading_rad, bool hold_position){
        target_states_.x = x;
        target_states_.y = y;
        target_states_.heading = heading_rad;
        hold_position_ = hold_position;
        target_orientation_.setRPY(0.0,0.0,heading_rad);

        
    }

    void update(){
        float diff_x = target_states_.x-current_states_.x;
        float diff_y = target_states_.y-current_states_.y;

        float distance = std::hypot(diff_x,diff_y);

        float velocity;
        float follow_velocity = max_velocity_;

        if(hold_position_ && distance <= braking_radius_){
            velocity = distance/braking_radius_ * follow_velocity;
        } else {
            velocity = follow_velocity;
        }

        target_states_.heading = std::atan2(diff_y,diff_x);


        float vx = diff_x/(distance + eplison) * velocity; 
        float vy = diff_y/(distance + eplison) * velocity;

        float error = angle_wrap(target_states_.heading-current_states_.heading);
        
        
        float angular_velocity =  pid_heading_->update(error);
        PID_OUT_TEMP = angular_velocity; //tf2::angleShortestPath(get_current_orientation(),target_orientation_)

        set_velocity_cmd(vx,vy,angular_velocity);
    }

    geometry_msgs::msg::TwistStamped  get_velocity_cmd(){
        geometry_msgs::msg::TwistStamped vel_cmd;
        //NED TO ENU
        vel_cmd.twist.linear.y = target_states_.vx;
        vel_cmd.twist.linear.x = target_states_.vy;
        vel_cmd.twist.angular.z = -angular_velocity_z_;
        return vel_cmd;
    }

    double angle_wrap(double radians) {
        while (radians > M_PI)  { radians -= 2 * M_PI; }
        while (radians < -M_PI) { radians += 2 * M_PI; }
        
        return radians;
    }
    
    float map(float x, float in_min, float in_max, float out_min, float out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    States get_states(){
        return current_states_;
    }
    States get_target_states(){
        return target_states_;
    }
    float get_pid_output(){
        return PID_OUT_TEMP;
    }

    double get_position_x(){
        return current_states_.x;
    }

    double get_position_y(){
        return current_states_.y;
    }

    private:

        void set_current_orientation(const tf2::Quaternion &q){
            current_orientation_ = q;
        }

        tf2::Quaternion get_current_orientation(){
            return current_orientation_;
        }

        void set_velocity_cmd(float velocity_x, float velocity_y, float angular_velocity_z){
            target_states_.vx = velocity_x;
            target_states_.vy = velocity_y;
            angular_velocity_z_ = angular_velocity_z;
        }

        float inline get_path_heading(States current, States target){
            float heading = -std::atan2(target.x-current.x,target.y-current.y);
            return heading;
        }

        States current_states_;
        States target_states_;
        bool hold_position_;
        float PID_OUT_TEMP;
        tf2::Quaternion current_orientation_;
        tf2::Quaternion target_orientation_;

        float eplison = 0.001;
        
        float angular_velocity_z_;
        float max_velocity_;
        float braking_radius_;

        std::unique_ptr<PID> pid_heading_;
        


};