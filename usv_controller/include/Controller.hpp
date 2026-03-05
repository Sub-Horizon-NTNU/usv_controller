
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
        //this->waypoints_ = generate_circle_path(30,7.0,8.0,5.0);
        //current_waypoint_ = waypoints_[current_waypoint_index_]; 
        
      

        param_max_velocity_ = 3.0;
        param_wp_radius_ = 0.3;
        
        param_max_velocity_ = 2.0;
        param_kp_ = 1.0;
        param_ki_ = 0.0000
        param_kd_ = 0.0;
        
        this->path_handler_ = std::make_unique<PathHandler>(param_wp_radius_);
        this->usv_ = std::make_unique<USV>(param_kp_,param_ki_,param_kd_,param_max_velocity_);

        position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ap/pose/filtered",  // /mavros/local_position/pose"
            rclcpp::SensorDataQoS(),[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){

            this->usv_->set_pose_cb(msg);
            
            path_handler_->update(usv_->get_position_x(),usv_->get_position_y());
            usv_->set_target_pose(path_handler_->get_target_waypoint().x, path_handler_->get_target_waypoint().y, 0);
            usv_->update();
            RCLCPP_INFO(this->get_logger(), "Pose [%.2f, %.2f] | Target [%.2f,%.2f] | Dist: %.2f ",
                usv_->get_position_x(),  usv_->get_position_y(),  path_handler_->get_target_waypoint().x,  path_handler_->get_target_waypoint().y, 
                std::hypot(path_handler_->get_target_waypoint().x-usv_->get_position_x(),path_handler_->get_target_waypoint().y-usv_->get_position_y()));
        });
            
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

        update_timer_ = this->create_wall_timer(
              std::chrono::milliseconds(50),
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
    double param_kp_;
    double param_ki_;
    double param_kd_;
    double param_wp_radius_;



};




