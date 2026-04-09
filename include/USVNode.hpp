#pragma once

#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "usv/USV.hpp"

class USVNode : public rclcpp::Node
{
public:
    USVNode();
    
    void init();
    

private:
    std::unique_ptr<USV> usv_;
};
