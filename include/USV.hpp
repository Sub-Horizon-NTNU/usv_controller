#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "usv_controller/WaypointManager.hpp"
#include "usv_controller/Structures.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "usv_controller/PositionController.hpp"
//#include "usv_controller/USVStates.hpp"
#include "usv_controller/USVTransformHandler.hpp"
#include <chrono>
#include <waypoint_msgs/msg/waypoint_status.hpp>

class USV{
    public:
    USV(rclcpp::Node::SharedPtr node);

    void update();

    void publish_status();
    
    private:
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        rclcpp::Node::SharedPtr node_;
        std::unique_ptr<USVTransformHandler> usv_transform_handler_;
        std::unique_ptr<PositionController> position_controller_;
        rclcpp::Publisher<waypoint_msgs::msg::WaypointStatus>::SharedPtr status_publisher_;
        rclcpp::TimerBase::SharedPtr status_publisher_timer_;
        
};
