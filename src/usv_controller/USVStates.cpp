#include "usv_controller/USVStates.hpp"

  USVStates::USVStates(rclcpp::Node::SharedPtr node) : node_(node) {

    position_subscriber_ =
        node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ap/pose/filtered", // Ardupilot dds"
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
              set_pose_cb(msg); // Set current position and orientation
            });
    velocity_subscriber_ =
        node_->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/ap/twist/filtered", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
              set_velocity_cb(msg); // Set the current velocity
            });
  }

  States USVStates::get_states() const { return current_states_; }


  void USVStates::set_velocity_cb(const geometry_msgs::msg::TwistStamped::SharedPtr twist) {
    // ENU to NED
    current_states_.vx = twist->twist.linear.y;
    current_states_.vy = twist->twist.linear.x;
    current_states_.angular_velocity = -twist->twist.angular.z;
  }
  void USVStates::set_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {

    // ENU TO NED
    current_states_.x = pose->pose.position.y;
    current_states_.y = pose->pose.position.x;

    // https://wiki.ros.org/tf2/Tutorials/Quaternions
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = pose->pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    // For rotating the quaternion..
    static tf2::Quaternion heading_rotate;
    heading_rotate.setRPY(0.0, 0.0, -M_PI / 2.0);

    tf2::Quaternion heading_rot =
        heading_rotate * quat_tf; // transforms from ENU to NED
    heading_rot.normalize();

    tf2::Matrix3x3 m_rot(heading_rot);
    double roll, pitch, yaw;
    m_rot.getRPY(roll, pitch, yaw);
    current_states_.heading = -yaw; // Sign introduced to follow NED conventions
  }


