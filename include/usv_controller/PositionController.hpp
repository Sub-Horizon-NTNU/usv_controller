#pragma once
#include "Structures.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "usv_controller/PID.hpp"
#include <waypoint_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/timer.hpp>
#include <iostream>


class PositionController{
    public:

    PositionController(rclcpp::Node::SharedPtr node);

        void update(const States &current_states);

        void compute_los_control(const States &current_states);
        void compute_pid_control(const States &current_states);

        void set_waypoint(const waypoint_msgs::msg::Waypoint &wp, const waypoint_msgs::msg::Waypoint &last_wp);

        double get_distance();
        geometry_msgs::msg::TwistStamped get_velocity_cmd() const;

        void update_los();

    private:
        double angle_wrap(double radians);

        void send_velocity_cmd(const float &vx, const float &vy, const float &vz);

        void init_parameters();


    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PID> pid_x_;
    std::shared_ptr<PID> pid_y_;
    std::shared_ptr<PID> pid_heading_;

    float braking_radius_;
    float max_velocity_;
    float heading_;

    waypoint_msgs::msg::Waypoint target_waypoint;
    waypoint_msgs::msg::Waypoint last_waypoint;

    geometry_msgs::msg::TwistStamped vel_cmd_;
    rclcpp::TimerBase::SharedPtr publish_velocity_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;

    static constexpr float epsilon = 0.001f;
    //parameters
    //rclcpp::ParameterCallbackHandle::SharedPtr parameter_callback_;

};
