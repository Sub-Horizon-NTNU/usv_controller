#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "Structures.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


class USVStates {
public:

  USVStates(rclcpp::Node::SharedPtr node);

  States get_states() const;

private:

  void set_velocity_cb(const geometry_msgs::msg::TwistStamped::SharedPtr twist);
  void set_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose) ;

  States current_states_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscriber_;
};

