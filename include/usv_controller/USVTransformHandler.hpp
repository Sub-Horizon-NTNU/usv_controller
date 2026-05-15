

#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <geometry_msgs/msg/vector3.hpp>

class USVTransformHandler{
    public:
    USVTransformHandler(rclcpp::Node::SharedPtr node);
    
    void update_usv_position();

    geometry_msgs::msg::Transform get_usv_position();

    double get_heading();
    
    private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr get_pose_timer_;
    geometry_msgs::msg::Transform pose_{};
    double heading_;
    double pitch_;
    double roll_;
};

