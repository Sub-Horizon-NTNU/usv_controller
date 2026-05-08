#pragma once
#include "waypoint_msgs/msg/waypoint.hpp"
#include "waypoint_msgs/msg/waypoints.hpp"
#include <waypoint_msgs/msg/waypoint_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include "Structures.hpp"
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <optional>


/*
 * General flow that this class manages is to:
 *
 * 1. receive waypoints from subscribers
 * 2. add waypoint to list of waypoints_
 * 3. Go throught the waypoints_ and update the control_cmd_ based on the current waypoint
 *  each waypoint has its own handling function which changs the control cmd based on the waypoint type
 *
 */

class WaypointManager {
    public:
    WaypointManager(rclcpp::Node::SharedPtr node);
    
    //Message containing target position and control mode(s)
    
    
    void update(const float &current_x, const float &current_y, const float &heading);
    
    waypoint_msgs::msg::Waypoint get_current_waypoint();
    
    waypoint_msgs::msg::Waypoint get_previous_waypoint();
    
    waypoint_msgs::msg::WaypointStatus  get_waypoint_status();
    private:
    
    void add_to_path(const waypoint_msgs::msg::Waypoint &waypoint);
    const std::vector<waypoint_msgs::msg::Waypoint>  get_waypoints();

    void add_list_to_path(const waypoint_msgs::msg::Waypoints &waypoints);
    
    void clear_path();
    
    void update_path();

    bool waypoint_reached();

    void handle_none_waypoint();

    bool handle_waypoint(const waypoint_msgs::msg::Waypoint &wp);
    void print_waypoint(const waypoint_msgs::msg::Waypoint &waypoint);

    void move_to_next_waypoint();

    bool handle_waypoint_hold(const waypoint_msgs::msg::Waypoint &wp_hold);

    bool handle_waypoint_pass(const waypoint_msgs::msg::Waypoint &wp_pass);

    void publish_waypoint_status();

    rclcpp::Node::SharedPtr node_;
    float position_x_{};
    float position_y_{};
    float heading_{};
    bool waypoint_reached_{};
    bool prev_waypoint_reached_{};
    ControlCmd control_cmd_{};
    unsigned int waypoint_index_{};
    std::vector<waypoint_msgs::msg::Waypoint> waypoints_;
    waypoint_msgs::msg::Waypoint previous_waypoint_;
    waypoint_msgs::msg::Waypoint most_recent_waypoint_;
    std::chrono::steady_clock::time_point current_waypoint_time_start;
    bool hold_timer_started_{};
    bool hold_position_{};

    rclcpp::Subscription<waypoint_msgs::msg::Waypoint>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<waypoint_msgs::msg::Waypoints>::SharedPtr waypoint_list_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_waypoints_subscriber_;

    rclcpp::Publisher<waypoint_msgs::msg::Waypoint>::SharedPtr waypoint_status_publisher_;
    rclcpp::TimerBase::SharedPtr waypoint_status_publisher_timer_;
};
