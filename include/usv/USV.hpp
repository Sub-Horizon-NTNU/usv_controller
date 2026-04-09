#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "PathScheduler.hpp"
#include "Structures.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "usv/PositionController.hpp"
#include "usv/USVStates.hpp"
#include <chrono>

class USV{
    public:
    USV(rclcpp::Node::SharedPtr node);

    void update();
    
    private:
        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        rclcpp::Node::SharedPtr node_;
        std::unique_ptr<USVStates> usv_states_;
        std::unique_ptr<PositionController> position_controller_;
        std::unique_ptr<PathScheduler> path_scheduler_;
};
