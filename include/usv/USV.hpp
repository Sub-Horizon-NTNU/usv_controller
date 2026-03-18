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
    USV(rclcpp::Node::SharedPtr node): node_(node){
        this->position_controller_ = std::make_unique<PositionController>(node_);
        this->usv_states_ = std::make_unique<USVStates>(node_);
        this->path_scheduler_ = std::make_unique<PathScheduler>(node_);
        control_loop_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&USV::update, this) );
    }

    void update(){
        path_scheduler_->update(usv_states_->get_states().x, usv_states_->get_states().y);
        position_controller_->update(usv_states_->get_states(), path_scheduler_->get_control_cmd());
        RCLCPP_INFO(node_->get_logger(),"Current position [%.2f %.2f], Target position [%.2f, %.2f]", usv_states_->get_states().x,usv_states_->get_states().y,path_scheduler_->get_control_cmd().x,path_scheduler_->get_control_cmd().y);
        RCLCPP_INFO(node_->get_logger(),"vel cmd: [%.2f,%.2f]", position_controller_->get_velocity_cmd().twist.linear.x, position_controller_->get_velocity_cmd().twist.linear.y);
    }
    
    private:
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
        rclcpp::Node::SharedPtr node_;
        std::unique_ptr<USVStates> usv_states_;
        std::unique_ptr<PositionController> position_controller_;
        std::unique_ptr<PathScheduler> path_scheduler_;

};
