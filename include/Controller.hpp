#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <waypoint_msgs/msg/detail/waypoint_hold__struct.hpp>
#include "PathScheduler.hpp"
#include "USV.hpp"

//TODO: Add cb for PathScheduler

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("usv_controller")
    {
        init_parameters();
        this->path_scheduler_ = std::make_unique<PathScheduler>();
        this->usv_ = std::make_unique<USV>(param_max_velocity_, param_kp_, param_ki_, param_kd_, param_max_angular_velocity_, param_braking_radius_);
        
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ap/pose/filtered",  // Ardupilot dds"  
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                
                usv_->set_pose_cb(msg); // Set current position and orientation 
                path_scheduler_->update(usv_->get_position_x(), usv_->get_position_y());
                usv_->set_control_cmd(path_scheduler_->get_control_cmd());
                usv_->update(); //Update the regulator etc
                RCLCPP_INFO(this->get_logger(),"Target [%.2f,%.2f] | Current [%.2f,%.2f]",path_scheduler_->get_control_cmd().x,path_scheduler_->get_control_cmd().y, usv_->get_position_x(),usv_->get_position_y());
            }
        );
    
        // Velocity publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        update_timer_ = this->create_wall_timer(
              std::chrono::milliseconds(30),
              [this](){
                  velocity_publisher_->publish(usv_->get_velocity_cmd());
            }
        );
        
        waypoint_hold_subscriber_ = this->create_subscription<waypoint_msgs::msg::WaypointHold>(
            "/selene/waypoint/hold",
            10, [this](const waypoint_msgs::msg::WaypointHold wp_hold){ path_scheduler_->add_to_path(wp_hold); });
        
        waypoint_pass_subscriber_ = this->create_subscription<waypoint_msgs::msg::WaypointPass>(
            "/selene/waypoint/pass",
            10, [this](const waypoint_msgs::msg::WaypointPass wp_pass){ path_scheduler_->add_to_path(wp_pass); });
        
        //path_scheduler_timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(100), std::bind(&Controller, this, Controller::)
        //)
        parameter_callback_handler_ = this->add_on_set_parameters_callback(std::bind(&Controller::handle_changed_parameters,this,std::placeholders::_1));  
    }
    
    ~Controller(){
        geometry_msgs::msg::TwistStamped null_vel{};
        velocity_publisher_->publish(null_vel);
    }
    
//Parameter mess
    void init_parameters(){
        this->declare_parameter<double>("wp_radius", 0.2);
        this->declare_parameter<double>("max_angular_velocity", 5.0);
        this->declare_parameter<double>("max_linear_velocity", 2.0);
        this->declare_parameter<double>("braking_radius", 3.0);

        this->declare_parameter<double>("yaw_kp", 1.0);
        this->declare_parameter<double>("yaw_ki", 0.0);
        this->declare_parameter<double>("yaw_kd", 0.0);

        param_wp_radius_ = this->get_parameter("wp_radius").get_value<double>();
        param_max_angular_velocity_ = this->get_parameter("max_angular_velocity").get_value<double>();
        param_max_velocity_ = this->get_parameter("max_linear_velocity").get_value<double>();
        param_braking_radius_ = this->get_parameter("braking_radius").get_value<double>();
        param_kp_ = this->get_parameter("yaw_kp").get_value<double>();
        param_ki_ = this->get_parameter("yaw_ki").get_value<double>();
        param_kd_ = this->get_parameter("yaw_kd").get_value<double>();
    }
    
    rcl_interfaces::msg::SetParametersResult handle_changed_parameters(const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters updated";
        
        for(const auto &parameter : parameters){
            if(parameter.get_name() == "wp_radius"){
                param_wp_radius_ = parameter.as_double();
                RCLCPP_INFO(this->get_logger(),"wp_radius set to: %.2f", param_wp_radius_);
            } 
            else if(parameter.get_name() == "max_angular_velocity"){
                param_max_angular_velocity_ = parameter.as_double();
                RCLCPP_INFO(this->get_logger(),"max_angular_velocity set to: %.4f [rad/s]", param_max_angular_velocity_);
            }
            else if(parameter.get_name() == "max_linear_velocity"){
                param_max_velocity_ = parameter.as_double();
                RCLCPP_INFO(this->get_logger(),"max_linear_velocity set to: %.4f [m/s]", param_max_velocity_);
            }
            else if(parameter.get_name() == "braking_radius"){
                param_braking_radius_ = parameter.as_double();
                RCLCPP_INFO(this->get_logger(),"braking_radius set to: %.4f [m/s]", param_braking_radius_);
            }
            else if(parameter.get_name() == "yaw_kp"){
                usv_->pid_heading()->set_kp(parameter.as_double());
                RCLCPP_INFO(this->get_logger(),"yaw Kp set to: %.4f [m/s]", param_kp_);
            }
            else if(parameter.get_name() == "yaw_ki"){
                usv_->pid_heading()->set_ki(parameter.as_double());
                RCLCPP_INFO(this->get_logger(),"yaw Kp set to: %.4f [m/s]", param_kp_);
            }
            else if(parameter.get_name() == "yaw_kd"){
                usv_->pid_heading()->set_kd(parameter.as_double());
                RCLCPP_INFO(this->get_logger(),"yaw Kp set to: %.4f [m/s]", param_kp_);
            }
            else {
                result.successful = false;
                result.reason = "Parameter set incorrectly";
                //RCLCPP_WARN(this->get_logger(), "Parameter unsupported: %s", parameter.get_name().c_str());
            }
        }
        return result;
    }

private:

    std::unique_ptr<USV> usv_;
    std::unique_ptr<PathScheduler> path_scheduler_;

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber_;
    
    rclcpp::TimerBase::SharedPtr velocity_pub_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;
    
    rclcpp::Subscription<waypoint_msgs::msg::WaypointHold>::SharedPtr waypoint_hold_subscriber_;
    rclcpp::Subscription<waypoint_msgs::msg::WaypointPass>::SharedPtr waypoint_pass_subscriber_;
    //rclcpp::TimerBase::SharedPtr path_scheduler_timer_;
    
    
    
    //Parameters
    double param_max_velocity_;
    double param_max_angular_velocity_;
    double param_kp_;
    double param_ki_;
    double param_kd_;
    double param_wp_radius_;
    double param_braking_radius_;



};




