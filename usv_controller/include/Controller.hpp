
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_home.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include <chrono>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <mavros_msgs/msg/detail/position_target__struct.hpp>
#include <mavros_msgs/srv/detail/set_mode__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>
#include <thread>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>

#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>






class Controller : public rclcpp::Node
{
public:
    Controller() : Node("usv_controller")
    {
        //init();
        //while(true){
        //    set_mode("MANUAL");
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //    set_mode("AUTO");
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //    set_mode("GUIDED");
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //}   
        //rclcpp::Rate(20.0);

        // https://medium.com/@sidharthmohannair/beyond-waypoints-mastering-ardupilot-offboard-control-with-mavros-guided-mode-56234cb867e5
        //velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);
        //velocity_pub_timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(10),
        //    std::bind(&Controller::publish_velocity_cmd, this));
        trajectory_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
        trajectory_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&Controller::publish_trajectory_local, this));
        //pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
        ////pose_publisher_timer_ = this->create_wall_timer(
        ////    std::chrono::milliseconds(100),
        ////    std::bind(&Controller::publish_position_setpoint, this));
        
    }

    void init(){
        command_home_ = this->create_client<mavros_msgs::srv::CommandHome>("/mavros/cmd/set_home");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/cmd/set_mode");
    }

    void publish_velocity_cmd(){
        geometry_msgs::msg::TwistStamped vel_cmd;
        vel_cmd.twist.linear.x = 0.0;
        vel_cmd.twist.linear.y = 1.0;
        vel_cmd.twist.linear.z = 0.0;
        vel_cmd.twist.angular.x = 0;
        vel_cmd.twist.angular.y = 0;
        vel_cmd.twist.angular.z = 0;

        velocity_pub_->publish(vel_cmd);
    }
    void publish_position_setpoint(){
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = 10.0;
        pose.pose.position.y = 2.0;
        pose.pose.position.z = -1;
        pose_publisher_->publish(pose);


    }

    void publish_trajectory_local(){
        using pt = mavros_msgs::msg::PositionTarget;
        pt target;
        target.header.stamp = this->now();
        target.header.frame_id = "map";
        target.coordinate_frame = pt::FRAME_LOCAL_NED;
        // Ignore acceleration, and yaw  https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html
        target.type_mask =
            pt::IGNORE_AFX |
            pt::IGNORE_AFY |
            pt::IGNORE_AFZ |
            pt::IGNORE_VZ |
            pt::IGNORE_YAW |
            pt::IGNORE_YAW_RATE;
        
        target.position.x = 0.0;
        target.position.y = 0.0;
        target.position.z = -1.0; 
        
        target.velocity.x = 0.0; 
        target.velocity.y = 0.5;
        target.velocity.z = 0.0;
    
        trajectory_pub_->publish(target);
    }

    bool set_home(){
        auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
        request->current_gps = true;
    
          while (!command_home_->wait_for_service(std::chrono::milliseconds(1000))) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Set home service failed");
            return false;
            }
            RCLCPP_INFO(this->get_logger(), "Set home service not available, retrying");
        }
        auto future = command_home_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->shared_from_this(),future) == rclcpp::FutureReturnCode::SUCCESS){
            if(future.get()->success){
                return true;
            }
            return false;
        }
        return false;
    }

    bool set_mode(std::string mode){
        if(mode != "MANUAL" && mode != "AUTO" && mode !="GUIDED"){
            RCLCPP_ERROR(this->get_logger(),"Set mode doesn't exist");
            return false;
        }
        while(!set_mode_client_->wait_for_service(std::chrono::milliseconds(1000))){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(),"Set mode failed");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Setting mode failed");
        }
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        auto future = set_mode_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->shared_from_this(),future) == rclcpp::FutureReturnCode::SUCCESS){
            auto result = future.get();
            if(result->mode_sent){
                return true;
            }
            return false;
        }
        return false;
    }

private:

    rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr command_home_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr trajectory_timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr velocity_pub_timer_;
    rclcpp::TimerBase::SharedPtr pose_publisher_timer_;

};




