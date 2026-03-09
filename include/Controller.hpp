
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_home.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include <chrono>
#include <cmath>
#include <memory>

#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>

#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>

#include <mavros_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <functional>
#include <tf2/LinearMath/Vector3.h>
#include "Structures.hpp"
#include "USV.hpp"
#include "PathHandler.hpp"
#include "PID.hpp"


class Controller : public rclcpp::Node
{
public:
    Controller() : Node("usv_controller")
    {
      
        param_wp_radius_ = 0.3;
        param_max_angular_velocity_ = 5.5;
        param_max_velocity_ = 2.0;
        param_kp_ = 1.0;
        param_ki_ = 0.1;
        param_kd_ = 0.01;
        
        this->path_handler_ = std::make_unique<PathHandler>(param_wp_radius_);
        this->usv_ = std::make_unique<USV>(param_max_velocity_,param_kp_,param_ki_,param_kd_, param_max_angular_velocity_);

        position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",  // Requires ArduPilot DDS /mavros/local_position/pose Ardupilot dds/ap/pose/filtered"  
            rclcpp::SensorDataQoS(),[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){

            this->usv_->set_pose_cb(msg);
            
            path_handler_->update(usv_->get_position_x(),usv_->get_position_y());
           
            usv_->set_target_pose(path_handler_->get_target_waypoint().x, path_handler_->get_target_waypoint().y, 0.0,path_handler_->get_target_waypoint().hold_at_point); //replace with wp
           
            usv_->update();
        
            RCLCPP_INFO(this->get_logger(), "Pose [%.2f, %.2f] | Target [%.2f,%.2f] | Dist: %.2f | Vel [%.2f, %.2f] Target/Current/diff angle: [%.2f,%.2f,%.2f], PID: %.2f ",
                usv_->get_position_x(),  usv_->get_position_y(),  path_handler_->get_target_waypoint().x,  path_handler_->get_target_waypoint().y, 
                std::hypot(path_handler_->get_target_waypoint().x-usv_->get_position_x(),path_handler_->get_target_waypoint().y-usv_->get_position_y()), usv_->get_target_states().vx, usv_->get_target_states().vy
                ,usv_->get_target_states().heading*180/M_PI, usv_->get_states().heading*180/M_PI,(usv_->get_states().heading-usv_->get_target_states().heading)*180/M_PI,usv_->get_pid_output());
        });
            
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

        update_timer_ = this->create_wall_timer(
              std::chrono::milliseconds(30),
              [this](){
                velocity_publisher_->publish(usv_->get_velocity_cmd());
            }
        );


    }

    inline float calculate_heading(const double &from_x, const double &from_y, const double &to_x, const double &to_y){
        float heading = std::atan2(to_y-from_y, to_x-from_x);
        return heading;
    }

   
private:

    std::unique_ptr<USV> usv_;
    std::unique_ptr<PathHandler> path_handler_;

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber_; // Local!
    rclcpp::TimerBase::SharedPtr velocity_pub_timer_;
    rclcpp::TimerBase::SharedPtr pose_publisher_timer_;

    //Parameters
    double param_max_velocity_;
    double param_max_angular_velocity_;
    double param_kp_;
    double param_ki_;
    double param_kd_;
    double param_wp_radius_;



};




