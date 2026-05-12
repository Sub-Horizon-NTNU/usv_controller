#pragma once
#include "Structures.hpp"
#include <cmath>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include "usv_controller/PID.hpp"
#include <waypoint_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <waypoint_msgs/msg/waypoint_status.hpp>
#include <iostream>


class PositionController{
    public:

    PositionController(rclcpp::Node::SharedPtr node);

        void update(const States &current_states);

        void set_waypoint(const waypoint_msgs::msg::Waypoint &wp, const waypoint_msgs::msg::Waypoint &last_wp);

        void update_los();
        
        double get_distance();

        waypoint_msgs::msg::WaypointStatus get_waypoint_status();

        geometry_msgs::msg::TwistStamped get_velocity_cmd() const;
        std::vector<waypoint_msgs::msg::Waypoint>  get_waypoints();

        private:
        void send_velocity_cmd_world(const float &vx, const float &vy, const float &vz);
        void send_velocity_cmd_body(const float &vx, const float &vy, const float &vz);



        double angle_wrap(double radians);
        
        void send_velocity_cmd(const float &vx, const float &vy, const float &vz);
        
        void init_parameters();
        
        rcl_interfaces::msg::SetParametersResult handle_changed_parameters(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PID> pid_x_;
    std::shared_ptr<PID> pid_y_;
    std::shared_ptr<PID> pid_heading_;

    float max_velocity_{};
    float heading_{};
    double lookahead_distance_{};
    double filtered_heading_{};
    double alpha_{};
    double position_x_{};
    double position_y_{};
    double target_heading_{};

    waypoint_msgs::msg::Waypoint target_waypoint;
    waypoint_msgs::msg::Waypoint last_waypoint;

    geometry_msgs::msg::TwistStamped vel_cmd_;
    rclcpp::TimerBase::SharedPtr publish_velocity_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    
};
