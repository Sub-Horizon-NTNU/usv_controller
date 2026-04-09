#pragma once
#include "Waypoint.hpp"
#include "Structures.hpp"
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

/*
 * General flow that this class manages is to:
 *
 * 1. receive waypoints from multiple subscribers
 * callbacks (one for each waypoint type)
 * 2. add waypoint to list of waypoints_
 * 3. Go throught the waypoints_ and update the control_cmd_ based on the current waypoint
 *  each waypoint has its own handling function which changs the control cmd based on the waypoint type
 *
 */

class PathScheduler {
    public:
    PathScheduler(rclcpp::Node::SharedPtr node);
    void add_to_path(const WaypointData &waypoint);

    void add_to_path(const std::vector<WaypointData> &waypoint_types);
    //Message containing target position and control mode(s)
    ControlCmd get_control_cmd() const;

    void clear_path();

    void update(const float &current_x, const float &current_y);
private:
    void update_path();

    bool waypoint_reached();

    void handle_none_waypoint();

    bool handle_waypoint(const waypoint_msgs::msg::Waypoint &wp);

    void move_to_next_waypoint();

    bool handle_waypoint_hold(const waypoint_msgs::msg::WaypointHold &wp_hold);

    bool handle_waypoint_pass(const waypoint_msgs::msg::WaypointPass wp_pass);

    //Must be updated by overwriting the existing control command variable ("control_cmd_").
    void update_control_cmd(const ControlCmd cmd);

    rclcpp::Node::SharedPtr node_;
    float position_x_{};
    float position_y_{};

    float last_position_x_{};
    float last_position_y_{};

    bool waypoint_reached_{};
    bool prev_waypoint_reached_{};
    ControlCmd control_cmd_{};
    unsigned int waypoint_index_{};
    std::vector<Waypoint> waypoints_;
    std::vector<Waypoint> null_waypoints_;
    std::chrono::steady_clock::time_point current_waypoint_time_start;

    rclcpp::Subscription<waypoint_msgs::msg::WaypointHold>::SharedPtr waypoint_hold_subscriber_;
    rclcpp::Subscription<waypoint_msgs::msg::WaypointPass>::SharedPtr waypoint_pass_subscriber_;

};
