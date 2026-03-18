#pragma once

#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "usv/USV.hpp"

class USVNode : public rclcpp::Node
{
public:
    Controller() : Node("usv_controller")
    {}
    
    void init(){
        this->usv_ = std::make_unique<USV>(this->shared_from_this());

    }

private:
    std::unique_ptr<USV> usv_;
};
