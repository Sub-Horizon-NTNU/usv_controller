#pragma once
#include "Structures.hpp"
#include <cmath>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include "usv/PID.hpp"
#include <rclcpp/timer.hpp>
#include <iostream>
class PositionController{
    public:

    PositionController(rclcpp::Node::SharedPtr node): node_(node){
        init_parameters();

        this->pid_x_ = std::make_shared<PID>();
        this->pid_y_ = std::make_shared<PID>();
        this->pid_heading_ = std::make_shared<PID>();

        pid_x_->set_kp(node_->get_parameter("lin_kp").as_double());
        pid_x_->set_ki(node_->get_parameter("lin_ki").as_double());
        pid_x_->set_kd(node_->get_parameter("lin_kd").as_double());
        pid_x_->set_max_output(node_->get_parameter("max_linear_velocity").as_double());

        pid_y_->set_kp(node_->get_parameter("lin_kp").as_double());
        pid_y_->set_ki(node_->get_parameter("lin_ki").as_double());
        pid_y_->set_kd(node_->get_parameter("lin_kd").as_double());
        pid_y_->set_max_output(node_->get_parameter("max_linear_velocity").as_double());

        pid_heading_->set_kp(node_->get_parameter("yaw_kp").as_double());
        pid_heading_->set_ki(node_->get_parameter("yaw_ki").as_double());
        pid_heading_->set_kd(node_->get_parameter("yaw_kd").as_double());
        pid_heading_->set_max_output(node_->get_parameter("max_angular_velocity").as_double());
        max_velocity_ = node->get_parameter("max_linear_velocity").as_double();

        braking_radius_ = node->get_parameter("braking_radius").as_double();

        velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/ap/cmd_vel", 10);
    }

        void update(const States &current_states, const ControlCmd &control_commands){
            float diff_x = control_commands.x-current_states.x;
            float diff_y = control_commands.y-current_states.y;

            float velocity;
            float vx;
            float vy;
            float follow_velocity = max_velocity_;

            float distance = std::hypot(diff_x,diff_y);
            // x & y
            if(control_commands.brake && (distance <= braking_radius_) ){
                vx = pid_x_->update(diff_x);
                vy = pid_y_->update(diff_y);
                std::cout << "Here1" << std::endl;
            } else {
                velocity = follow_velocity;
                vx = diff_x/(distance + epsilon) * velocity;
                vy = diff_y/(distance + epsilon) * velocity;
                std::cout << "Here2" << std::endl;
            }
            if(control_commands.heading_on_path){
                heading_ = std::atan2(diff_y,diff_x);
            }

            // heading
            float error = angle_wrap(control_commands.heading-current_states.heading);

            float angular_velocity =  pid_heading_->update(error);

            //send cmd
            set_velocity_cmd(vx,vy,angular_velocity);
            velocity_publisher_->publish(get_velocity_cmd());
        }

        geometry_msgs::msg::TwistStamped get_velocity_cmd()const{
            return vel_cmd_;
        }
    private:
        double angle_wrap(double radians) {
            while (radians > M_PI)  { radians -= 2 * M_PI; }
            while (radians < -M_PI) { radians += 2 * M_PI; }

            return radians;
        }
        void set_velocity_cmd(const float &vx, const float &vy, const float &vz){
            std::cout << "Here3" << std::endl;
            vel_cmd_.twist.linear.x = vy;
            vel_cmd_.twist.linear.y = vx;
            vel_cmd_.twist.angular.z = -vz;
        }

        void init_parameters(){
            node_->declare_parameter<double>("yaw_kp", 1.0);
            node_->declare_parameter<double>("yaw_ki", 0.0);
            node_->declare_parameter<double>("yaw_kd", 0.0);

            node_->declare_parameter<double>("lin_kp", 1.0);
            node_->declare_parameter<double>("lin_ki", 0.0);
            node_->declare_parameter<double>("lin_kd", 0.0);

            node_->declare_parameter<double>("braking_radius", 0.0);
            node_->declare_parameter<double>("max_linear_velocity", 0.0);
            node_->declare_parameter<double>("max_angular_velocity", 0.0);

        }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PID> pid_x_;
    std::shared_ptr<PID> pid_y_;
    std::shared_ptr<PID> pid_heading_;

    float braking_radius_;
    float max_velocity_;
    float heading_;

    geometry_msgs::msg::TwistStamped vel_cmd_;
    rclcpp::TimerBase::SharedPtr publish_velocity_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;

    static constexpr float epsilon = 0.001f;
    //parameters
    //rclcpp::ParameterCallbackHandle::SharedPtr parameter_callback_;

};
