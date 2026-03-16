#pragma once
#include <variant>
#include "waypoint_msgs/msg/waypoint_hold.hpp"
#include "waypoint_msgs/msg/waypoint_pass.hpp"

using WaypointData = std::variant<
        waypoint_msgs::msg::WaypointHold, // HOLD
        waypoint_msgs::msg::WaypointPass> // PASS
        ;

//index corresponds to declaration order in WaypointData
enum WaypointTypeIndex {
    HOLD = 0,
    PASS = 1
};

struct Waypoint{
    WaypointData Data;
};