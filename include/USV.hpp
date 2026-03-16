#pragma once

#include <cmath>
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
        this->pid_heading_ = std::make_shared<PID>(kp,ki,kd,max_angular_velocity);
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

        tf2::Quaternion heading_rot = heading_rotate*quat_tf; //transforms from ENU to NED
        heading_rot.normalize();
        set_current_orientation(heading_rot);

        tf2::Matrix3x3 m_rot(heading_rot);
        double roll,pitch,yaw;
        m_rot.getRPY(roll, pitch, yaw);
        current_states_.heading = -yaw; // Sign introduced to follow NED conventions

    }
    void set_control_cmd(const ControlCmd &control_cmd){
        control_command_.x = control_cmd.x;
        control_command_.y = control_cmd.y;
        control_command_.heading = control_cmd.heading;
        control_command_.hold_position = control_cmd.hold_position;
        control_command_.heading_on_path = control_cmd.heading_on_path;


        target_orientation_.setRPY(0.0,0.0,control_command_.heading);
    }

    void update(){
        float diff_x = control_command_.x-current_states_.x;
        float diff_y = control_command_.y-current_states_.y;

        float distance = std::hypot(diff_x,diff_y);

        float velocity;
        float follow_velocity = max_velocity_;

        if(control_command_.brake && distance <= braking_radius_){
            velocity = distance/braking_radius_ * follow_velocity;
        } else {
            velocity = follow_velocity;
        }

        control_command_.heading = std::atan2(diff_y,diff_x);

        float vx = diff_x/(distance + eplison) * velocity;
        float vy = diff_y/(distance + eplison) * velocity;

        float error = angle_wrap(control_command_.heading-current_states_.heading);

        float angular_velocity =  pid_heading_->update(error);
        set_velocity_cmd(vx,vy,angular_velocity);
    }

    geometry_msgs::msg::TwistStamped get_velocity_cmd(){
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

    std::shared_ptr<PID> pid_heading(){
        return pid_heading_;
    }

    States get_states(){
        return current_states_;
    }
    ControlCmd get_target_states() const {
        return control_command_;
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
        ControlCmd control_command_;
        tf2::Quaternion current_orientation_;
        tf2::Quaternion target_orientation_;

        float angular_velocity_z_;
        float max_velocity_;
        float braking_radius_;

        std::shared_ptr<PID> pid_heading_;

        static constexpr float eplison = 0.001f;
};
