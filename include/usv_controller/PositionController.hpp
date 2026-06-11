#pragma once
#include "Structures.hpp"
#include <cmath>
#include <cstdint>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include "usv_controller/PID.hpp"
#include <waypoint_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <waypoint_msgs/msg/waypoint_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include "WaypointManager.hpp"
#include "usv_controller/OmniXAllocator.hpp"
#include <array>
#include <iostream>


class PositionController{
    public:

    PositionController(rclcpp::Node::SharedPtr node);

        void update(const States &current_states);

        void set_waypoint(const waypoint_msgs::msg::Waypoint &wp, const waypoint_msgs::msg::Waypoint &last_wp);

        void update_los();
        
        double get_distance();

        waypoint_msgs::msg::WaypointStatus get_waypoint_status();

        std::vector<waypoint_msgs::msg::Waypoint>  get_waypoints();

        private:
        // LOS guidance output -> picks manual vs auto source and drives the PX4 offboard stream.
        void send_velocity_cmd_world(const float &vx, const float &vy, const float &vz);

        double angle_wrap(double radians);

        // PX4 offboard helpers (direct-actuator: omni-X allocation done here on the companion).
        void publish_offboard_control_mode();
        void publish_actuators(const std::array<float, 4> &thrusts, bool servos_centered);
        void publish_vehicle_command(uint16_t command, double param1 = 0.0, double param2 = 0.0);
        void arm();
        void set_offboard_mode();

        // Manual override + arm callbacks.
        void handle_manual_override(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_manual_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
        void handle_manual_estop(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_arm(const std_msgs::msg::Bool::SharedPtr msg);

        void init_parameters();
        
        rcl_interfaces::msg::SetParametersResult handle_changed_parameters(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PID> pid_x_;
    std::shared_ptr<PID> pid_y_;
    std::shared_ptr<PID> pid_heading_;

    float max_velocity_{};
    float heading_{};
    double lookahead_distance_{};
    double filtered_heading_{};
    double alpha_{};
    double position_x_{};
    double position_y_{};
    double target_heading_{};
    waypoint_msgs::msg::Waypoint target_waypoint;

    std::unique_ptr<WaypointManager> waypoint_manager_;

    // PX4 direct-actuator offboard interface (omni-X: 4 thrusters + 4 static azimuth servos).
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    uint64_t offboard_setpoint_counter_{0};

    // Omni-X allocation + body-wrench mapping.
    std::unique_ptr<OmniXAllocator> allocator_;
    double surge_force_gain_{};   // body surge velocity [m/s] -> force [N]
    double sway_force_gain_{};    // body sway velocity  [m/s] -> force [N]
    double yaw_force_gain_{};     // yaw command -> yaw moment [N·m]
    // Static azimuth servo commands (PX4 normalized -1..1), ±45° X-pattern [+,-,-,+].
    std::array<float, 4> servo_pattern_{};

    // Manual override (joystick over DDS). The controller stays the single offboard publisher;
    // when override is active it streams the manual command instead of the LOS output.
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_override_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_estop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
    bool manual_override_{false};
    bool estopped_{false};
    bool armed_{false};
    bool arm_requested_{false};   // explicit arm/disarm via selene/arm (no auto-arm)
    uint64_t loop_count_{0};
    geometry_msgs::msg::Twist manual_cmd_;

    // AUX outputs on the OrangeCube, addressed by ActuatorServos.control[] index
    // (= PX4 "Servo N+1" function; map AUX3->Servo7, AUX4->Servo8, AUX5->Servo5,
    // AUX6->Servo6). AUX3 safety relay gates power to ALL thrusters + azimuth servos:
    // it MUST be driven high or the boat is dead even when armed. Exactly one light is
    // on: red = unarmed/relay-off, amber = manual, green = guided.
    int relay_servo_index_{6};        // AUX3 safety relay
    int light_red_servo_index_{7};    // AUX4 red   = unarmed / relay off
    int light_amber_servo_index_{4};  // AUX5 amber = manual
    int light_green_servo_index_{5};  // AUX6 green = guided/auto

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
