#pragma once
#include "Structures.hpp"
#include <cmath>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include "usv_controller/PID.hpp"
#include <rclcpp/timer.hpp>
#include <iostream>
#include <mavros_msgs/msg/position_target.hpp>
class PositionController{
    public:

    PositionController(rclcpp::Node::SharedPtr node);

        void update(const States &current_states, const ControlCmd &control_commands);

        geometry_msgs::msg::TwistStamped get_velocity_cmd() const;

        double get_distance();

        void update_control_cmd(const ControlCmd &new_control_cmd);

    private:
        double angle_wrap(double radians);

        void send_velocity_cmd(const float &vx, const float &vy, const float &vz);

        void init_parameters();


    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PID> pid_x_;
    std::shared_ptr<PID> pid_y_;
    std::shared_ptr<PID> pid_heading_;
    ControlCmd prev_control_cmd_;
    ControlCmd last_control_cmd;

    float braking_radius_;
    float max_velocity_;
    float heading_;


    geometry_msgs::msg::TwistStamped vel_cmd_;
    rclcpp::TimerBase::SharedPtr publish_velocity_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_publisher_;

    static constexpr float epsilon = 0.001f;
    //parameters
    //rclcpp::ParameterCallbackHandle::SharedPtr parameter_callback_;

};
