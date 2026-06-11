#include "usv_controller/PositionController.hpp"
#include "usv_controller/WaypointManager.hpp"
#include <waypoint_msgs/msg/detail/waypoint__struct.hpp>


    PositionController::PositionController(rclcpp::Node::SharedPtr node): node_(node){
        init_parameters();
        double initial_heading_deg = node_->get_parameter("initial_heading").as_double();
        filtered_heading_ = initial_heading_deg * M_PI / 180.0;

        // PX4 direct-actuator offboard publishers (uXRCE-DDS bridge). Replaces MAVROS cmd_vel.
        offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        actuator_motors_pub_ = node_->create_publisher<px4_msgs::msg::ActuatorMotors>(
            "/fmu/in/actuator_motors", 10);
        actuator_servos_pub_ = node_->create_publisher<px4_msgs::msg::ActuatorServos>(
            "/fmu/in/actuator_servos", 10);
        vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Manual override (joystick teleop over DDS).
        manual_override_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "selene/manual/override", 10,
            std::bind(&PositionController::handle_manual_override, this, std::placeholders::_1));
        manual_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "selene/manual/cmd", 10,
            std::bind(&PositionController::handle_manual_cmd, this, std::placeholders::_1));
        manual_estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "selene/manual/estop", 10,
            std::bind(&PositionController::handle_manual_estop, this, std::placeholders::_1));
        arm_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "selene/arm", 10,
            std::bind(&PositionController::handle_arm, this, std::placeholders::_1));

        parameter_callback_ = node_->add_on_set_parameters_callback(std::bind(&PositionController::handle_changed_parameters,this,std::placeholders::_1));
        waypoint_manager_ = std::make_unique<WaypointManager>(node,initial_heading_deg * M_PI / 180.0);
    }
        void PositionController::update(const States &current_states){

            double x = current_states.x;
            double y = current_states.y;
            heading_ = current_states.heading;
  
            // Origin reset: zero the position frame at the arm spot (x stays north).
            if (armed_ && !origin_set_) { ox_ = x; oy_ = y; origin_set_ = true; }
            if (!armed_) origin_set_ = false;
            if (origin_set_) { x -= ox_; y -= oy_; }

            // On the arm edge with no mission: hold the current spot + heading.
            if (armed_ && !was_armed_) {
                if (waypoint_manager_->get_waypoints().empty())
                    waypoint_manager_->hold_here(x, y, heading_);
            }       
            was_armed_ = armed_;
              
            position_x_ = x;
            position_y_ = y;
            double R = lookahead_distance_; // Lookahead
            double delta{};
            double vx;
            double vy;

            waypoint_manager_->update(x, y,heading_);
            
            waypoint_msgs::msg::Waypoint prev_waypoint = waypoint_manager_->get_previous_waypoint();
            waypoint_msgs::msg::Waypoint next_waypoint = waypoint_manager_->get_current_waypoint();

            double alpha_k = atan2(next_waypoint.y-prev_waypoint.y,next_waypoint.x-prev_waypoint.x);  // αk := atan2 (yk+1 − yk, xk+1 − xk) ∈ S (10.55)
            double s = (x-prev_waypoint.x)*cos(alpha_k) +(y-prev_waypoint.y)*sin(alpha_k);    
            double e = -(x-prev_waypoint.x)*sin(alpha_k) + (y-prev_waypoint.y)*cos(alpha_k);   
            double p = std::hypot(prev_waypoint.x - next_waypoint.x,prev_waypoint.y-next_waypoint.y); 
            
            waypoint_manager_->set_remaining_distance(p-s);
            
            if(std::abs(e) < R) {
                delta = sqrt(R*R - e*e);
            } else {
                delta = 0;
            }
            
            if(delta + s > p)
            {
                std::vector<waypoint_msgs::msg::Waypoint> waypoints = waypoint_manager_->get_waypoints();

                const unsigned int current_idx =  waypoint_manager_->get_current_waypoint_index();

                if(waypoints.size() >= 2){
                    for(unsigned int i = current_idx; i+1  < waypoints.size(); ++i)
                    {
                        prev_waypoint = waypoints[i];
                        next_waypoint = waypoints[i + 1];

                        alpha_k = atan2(next_waypoint.y - prev_waypoint.y,next_waypoint.x - prev_waypoint.x);
                        s = (x-prev_waypoint.x)*cos(alpha_k)+(y-prev_waypoint.y)*sin(alpha_k);
                        e = -(x-prev_waypoint.x)*sin(alpha_k)+(y-prev_waypoint.y)*cos(alpha_k);
                        p = std::hypot(next_waypoint.x - prev_waypoint.x, next_waypoint.y - prev_waypoint.y);

                        if(std::abs(e)<R)
                            delta = sqrt(R*R-e*e);
                        else
                            delta = 0.0;
                        if(delta + s <= p)
                            break;
                    }
                } else {
                    R  = 0.0;
                    delta = 0.0;
                }
            }
                        
            target_waypoint = next_waypoint;

            double desired_velocity = next_waypoint.velocity;

            if(desired_velocity <=0.01){
                desired_velocity = 2.0;
            }
            pid_x_->set_max_output(desired_velocity);
            pid_y_->set_max_output(desired_velocity);

            double x_los = prev_waypoint.x + (s+delta)*cos(alpha_k);
            double y_los = prev_waypoint.y + (s+delta)*sin(alpha_k);

            target_waypoint.x = x_los;
            target_waypoint.y = y_los;

            double X_d = atan2(y_los-y,x_los-x);

            //double e_x = last_waypoint.x+s*cos(alpha_k); // cross track coordinates
            //double e_y = last_waypoint.y+s*sin(alpha_k); 

            if(next_waypoint.hold && (((p-s < R) || (p-s < next_waypoint.radius)) || (next_waypoint.type == waypoint_msgs::msg::Waypoint::HOLD))){
                double error_x_world = next_waypoint.x-x;
                double error_y_world = next_waypoint.y-y;

                vx = pid_x_->update(error_x_world);
                vy = pid_y_->update(error_y_world);
            } else {
                vx = desired_velocity*cos(X_d);
                vy = desired_velocity*sin(X_d);
            }

            double target_heading;
            if(next_waypoint.keep_on_track) {
                target_heading = X_d;
            } else {
                target_heading = next_waypoint.heading;
            }

            filtered_heading_ = target_heading * alpha_ + (1-alpha_)*filtered_heading_;
            //Logging 
            target_heading_ = filtered_heading_;
            
            double yaw_vel = pid_heading_->update(angle_wrap(filtered_heading_ - current_states.heading));

            send_velocity_cmd_world(vx, vy, yaw_vel);
            
        }

        double PositionController::angle_wrap(double radians) {
            while (radians > M_PI)  { radians -= 2 * M_PI; }
            while (radians < -M_PI) { radians += 2 * M_PI; }

            return radians;
        }

        // Called every control tick (20 Hz). Drives the PX4 direct-actuator offboard stream:
        // maps the guidance/joystick velocity command to a body wrench, runs the omni-X
        // allocator, and publishes OffboardControlMode + ActuatorMotors/Servos every tick so
        // PX4 never drops OFFBOARD. Also runs the one-shot auto-arm state machine.
        // Inputs: vx, vy = desired WORLD-NED velocity; vz = yaw command (rate-ish from PID).
        void PositionController::send_velocity_cmd_world(const float &vx, const float &vy, const float &vz){
            // Explicit arm/disarm (NO auto-arm): only arm when selene/arm requests it,
            // and only after setpoints have streamed long enough for PX4 to accept
            // OFFBOARD. Re-assert OFFBOARD+ARM ~2 Hz while requested so a transient
            // rejection eventually takes (PX4 ignores redundant arm commands). E-stop
            // clears the request and forces disarm.
            if (arm_requested_ && !estopped_ && offboard_setpoint_counter_ >= 10) {
                if (loop_count_ % 10 == 0) {
                    set_offboard_mode();
                    arm();
                }
                armed_ = true;
            } else if (armed_ && (!arm_requested_ || estopped_)) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                armed_ = false;
                RCLCPP_INFO(node_->get_logger(), "Disarm command sent");
            }
            loop_count_++;

            // Desired body-frame surge/sway + yaw command.
            double surge, sway, yaw_cmd;
            if (manual_override_) {
                // Joystick command is already body-frame velocity (surge, sway) + yaw rate.
                surge = manual_cmd_.linear.x;
                sway = manual_cmd_.linear.y;
                yaw_cmd = manual_cmd_.angular.z;
            } else {
                // LOS gives world-NED velocity; rotate into the body frame.
                surge = cos(heading_) * vx + sin(heading_) * vy;
                sway = -sin(heading_) * vx + cos(heading_) * vy;
                yaw_cmd = vz;
            }

            // Feedforward velocity -> body wrench (drag model; gains tuned on water), then
            // omni-X allocation to 4 signed normalized thrusts.
            std::array<float, 4> thrusts{};
            if (!estopped_) {
                const double Fx = surge_force_gain_ * surge;
                const double Fy = sway_force_gain_ * sway;
                const double Mz = yaw_force_gain_ * yaw_cmd;
                thrusts = allocator_->allocate(Fx, Fy, Mz);
            } // else: leave thrusts at zero.

            publish_offboard_control_mode();
            publish_actuators(thrusts, /*servos_centered=*/estopped_);

            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        }

        void PositionController::publish_offboard_control_mode(){
            px4_msgs::msg::OffboardControlMode msg{};
            msg.position = false;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            msg.direct_actuator = true;  // omni-X allocation is done here on the companion
            msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_pub_->publish(msg);
        }

        // Publish the 4 thruster magnitudes (ActuatorMotors) and the static azimuth servo
        // pattern (ActuatorServos). PX4 normalized: motors signed -1..1 (bidirectional ESCs),
        // servos -1..1. Unused channels are NaN so PX4 leaves them untouched.
        void PositionController::publish_actuators(const std::array<float, 4> &thrusts, bool servos_centered){
            const uint64_t ts = node_->get_clock()->now().nanoseconds() / 1000;

            px4_msgs::msg::ActuatorMotors motors{};
            for (auto &c : motors.control) c = std::nanf("");
            for (int i = 0; i < 4; ++i) motors.control[i] = thrusts[i];
            motors.reversible_flags = 0x0F; // motors 0..3 are bidirectional
            motors.timestamp = ts;
            motors.timestamp_sample = ts;
            actuator_motors_pub_->publish(motors);

            px4_msgs::msg::ActuatorServos servos{};
            for (auto &c : servos.control) c = std::nanf("");
            for (int i = 0; i < 4; ++i) servos.control[i] = servos_centered ? 0.0f : servo_pattern_[i];

            // AUX relay + status lights. "active" = armed and not e-stopped; the
            // relay must be HIGH for any thruster/servo to have power.
            const bool active = armed_ && !estopped_;
            auto set_aux = [&](int idx, bool on) {
                if (idx >= 0 && idx < static_cast<int>(servos.control.size()))
                    servos.control[idx] = on ? 1.0f : -1.0f;
            };
            set_aux(relay_servo_index_, active);                       // AUX3 safety relay
            set_aux(light_red_servo_index_, !active);                  // AUX4 red = unarmed / relay off
            set_aux(light_amber_servo_index_, active && manual_override_);   // AUX5 amber = manual
            set_aux(light_green_servo_index_, active && !manual_override_);  // AUX6 green = guided/auto

            servos.timestamp = ts;
            servos.timestamp_sample = ts;
            actuator_servos_pub_->publish(servos);
        }

        void PositionController::publish_vehicle_command(uint16_t command, double param1, double param2){
            px4_msgs::msg::VehicleCommand msg{};
            msg.command = command;
            msg.param1 = param1;
            msg.param2 = param2;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_pub_->publish(msg);
        }

        void PositionController::arm(){
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            RCLCPP_INFO(node_->get_logger(), "Arm command sent");
        }

        void PositionController::set_offboard_mode(){
            // base_mode custom enabled (1), PX4 custom main mode OFFBOARD (6).
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            RCLCPP_INFO(node_->get_logger(), "OFFBOARD mode command sent");
        }

        void PositionController::handle_manual_override(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data != manual_override_) {
                RCLCPP_INFO(node_->get_logger(), "Manual override %s", msg->data ? "ENGAGED" : "released");
            }
            manual_override_ = msg->data;
        }

        void PositionController::handle_manual_cmd(const geometry_msgs::msg::Twist::SharedPtr msg){
            manual_cmd_ = *msg;
        }

        void PositionController::handle_manual_estop(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data && !estopped_) {
                estopped_ = true;
                armed_ = false;
                arm_requested_ = false;   // e-stop clears any arm request
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                RCLCPP_WARN(node_->get_logger(), "E-STOP: disarm command sent");
            } else if (!msg->data) {
                estopped_ = false;
            }
        }

        void PositionController::handle_arm(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data && estopped_) {
                RCLCPP_WARN(node_->get_logger(), "Arm ignored: e-stopped");
                return;
            }
            if (msg->data != arm_requested_) {
                RCLCPP_INFO(node_->get_logger(), "Arm request: %s", msg->data ? "ARM" : "DISARM");
            }
            arm_requested_ = msg->data;
        }

        void PositionController::init_parameters(){
            node_->declare_parameter<double>("yaw_kp", 0.9);
            node_->declare_parameter<double>("yaw_ki", 0.05);
            node_->declare_parameter<double>("yaw_kd", 0.0);

            node_->declare_parameter<double>("lin_kp", 1.2);
            node_->declare_parameter<double>("lin_ki", 0.1);
            node_->declare_parameter<double>("lin_kd", 0.05);

            node_->declare_parameter<double>("lookahead_distance", 0.5);
            node_->declare_parameter<double>("max_linear_velocity", 3.0);
            node_->declare_parameter<double>("max_angular_velocity", 3.0);
            node_->declare_parameter<double>("heading_reference_filter",0.5);

            this->pid_x_ = std::make_shared<PID>();
            this->pid_y_ = std::make_shared<PID>();
            this->pid_heading_ = std::make_shared<PID>();

            pid_x_->set_kp(node_->get_parameter("lin_kp").as_double());
            pid_x_->set_ki(node_->get_parameter("lin_ki").as_double());
            pid_x_->set_kd(node_->get_parameter("lin_kd").as_double());
            pid_x_->set_max_output(node_->get_parameter("max_linear_velocity").as_double());
            lookahead_distance_ = node_->get_parameter("lookahead_distance").as_double();
            alpha_ = node_->get_parameter("heading_reference_filter").as_double();

            pid_y_->set_kp(node_->get_parameter("lin_kp").as_double());
            pid_y_->set_ki(node_->get_parameter("lin_ki").as_double());
            pid_y_->set_kd(node_->get_parameter("lin_kd").as_double());
            pid_y_->set_max_output(node_->get_parameter("max_linear_velocity").as_double());

            pid_heading_->set_kp(node_->get_parameter("yaw_kp").as_double());
            pid_heading_->set_ki(node_->get_parameter("yaw_ki").as_double());
            pid_heading_->set_kd(node_->get_parameter("yaw_kd").as_double());
            
            pid_heading_->set_max_output(node_->get_parameter("max_angular_velocity").as_double());
            max_velocity_ = node_->get_parameter("max_linear_velocity").as_double();

            // --- Omni-X geometry + actuator mapping -----------------------------------------
            // Pods locked at a fixed ±45° X (the ArduSub Lua parked the steering servos here).
            // Layout from the boat (top-down, bow forward): T1 front-port, T2 rear-port,
            // T3 front-stbd, T4 rear-stbd; thrust angle pattern [+,-,-,+] · servo_angle.
            // Body frame: x forward+, y starboard+. SET pod_half_length/width to real metres.
            node_->declare_parameter<double>("pod_half_length", 0.5);  // |x| of pods [m] (a)
            node_->declare_parameter<double>("pod_half_width", 0.5);   // |y| of pods [m] (b)
            node_->declare_parameter<double>("servo_angle_deg", 45.0); // fixed azimuth
            node_->declare_parameter<double>("max_thrust", 30.0);      // per-pod thrust at ±1.0 [N]
            // Velocity-command -> body-force feedforward gains (drag model; tune on water).
            node_->declare_parameter<double>("surge_force_gain", 10.0);
            node_->declare_parameter<double>("sway_force_gain", 10.0);
            node_->declare_parameter<double>("yaw_force_gain", 10.0);
            // Static servo command in PX4 normalized -1..1 for the ±45° offset (depends on the
            // PX4 servo min/max config; default matches a 500-2500us/±180° servo -> 0.25).
            node_->declare_parameter<double>("servo_command_norm", 0.25);

            const double a = node_->get_parameter("pod_half_length").as_double();
            const double b = node_->get_parameter("pod_half_width").as_double();
            const double th = node_->get_parameter("servo_angle_deg").as_double() * M_PI / 180.0;
            surge_force_gain_ = node_->get_parameter("surge_force_gain").as_double();
            sway_force_gain_ = node_->get_parameter("sway_force_gain").as_double();
            yaw_force_gain_ = node_->get_parameter("yaw_force_gain").as_double();

            std::array<OmniXAllocator::Pod, 4> pods = {{
                {+a, -b, +th},   // T1 front-port,  +45° (NE)
                {-a, -b, -th},   // T2 rear-port,   -45° (NW)
                {+a, +b, -th},   // T3 front-stbd,  -45° (NW)
                {-a, +b, +th},   // T4 rear-stbd,   +45° (NE)
            }};
            allocator_ = std::make_unique<OmniXAllocator>(
                pods, node_->get_parameter("max_thrust").as_double());
            if (!allocator_->ok()) {
                RCLCPP_ERROR(node_->get_logger(),
                    "Omni-X allocator geometry is singular/uncontrollable - check pod params");
            }

            const float sn = static_cast<float>(node_->get_parameter("servo_command_norm").as_double());
            servo_pattern_ = {+sn, -sn, -sn, +sn}; // S1,S2,S3,S4 (matches the Lua offsets)

            // AUX relay + status lights (ActuatorServos indices). -1 disables a channel.
            relay_servo_index_ = node_->declare_parameter<int>("relay_servo_index", 6);        // AUX3
            light_red_servo_index_ = node_->declare_parameter<int>("light_red_servo_index", 7);   // AUX4
            light_amber_servo_index_ = node_->declare_parameter<int>("light_amber_servo_index", 4); // AUX5
            light_green_servo_index_ = node_->declare_parameter<int>("light_green_servo_index", 5); // AUX6
        }

        waypoint_msgs::msg::WaypointStatus PositionController::get_waypoint_status() {
            waypoint_msgs::msg::WaypointStatus wp_status;
            wp_status.header.stamp = node_->now();
            wp_status.current_x = position_x_;
            wp_status.current_y = position_y_;
            wp_status.target_waypoint = target_waypoint;
            wp_status.distance_x = position_x_- target_waypoint.x;
            wp_status.distance_y = position_y_- target_waypoint.y;
            wp_status.heading = heading_;
            wp_status.distance = std::hypot(wp_status.distance_x,wp_status.distance_y);
            wp_status.desired_heading = target_heading_;
            return wp_status;
    }

        rcl_interfaces::msg::SetParametersResult PositionController::handle_changed_parameters(const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters updated";
        
        for(const auto &parameter : parameters){
            double param_val = parameter.as_double();
            if(parameter.get_name() == "lookahead_distance"){
                lookahead_distance_ = param_val;
                RCLCPP_INFO(node_->get_logger(),"Lookahead distance set to: %.2f", param_val);
            } 
            else if(parameter.get_name() == "max_angular_velocity"){
                pid_heading_->set_max_output(param_val);
                RCLCPP_INFO(node_->get_logger(),"max_angular_velocity set to: %.4f [rad/s]", param_val);
            }
            else if(parameter.get_name() == "max_linear_velocity"){
                pid_x_->set_max_output(param_val);
                pid_y_->set_max_output(param_val);
                RCLCPP_INFO(node_->get_logger(),"max_linear_velocity set to: %.4f [m/s]", param_val);
            }
            else if(parameter.get_name() == "yaw_kp"){
                pid_heading_->set_kp(parameter.as_double());
                RCLCPP_INFO(node_->get_logger(),"yaw Kp set to: %.4f ", param_val);
            }
            else if(parameter.get_name() == "yaw_ki"){
                pid_heading_->set_ki(parameter.as_double());
                RCLCPP_INFO(node_->get_logger(),"yaw Ki set to: %.4f", param_val);
            }
            else if(parameter.get_name() == "yaw_kd"){
                pid_heading_->set_kd(parameter.as_double());
                RCLCPP_INFO(node_->get_logger(),"yaw Kd set to: %.4f", param_val);
            }
            else if(parameter.get_name() == "lin_kp"){
                pid_x_->set_kp(parameter.as_double());
                pid_y_->set_kp(parameter.as_double());

                RCLCPP_INFO(node_->get_logger(),"PID X/Y Kp set to: %.4f", param_val);
            }
            else if(parameter.get_name() == "lin_ki"){
                pid_x_->set_ki(parameter.as_double());
                pid_y_->set_ki(parameter.as_double());
                RCLCPP_INFO(node_->get_logger(),"PID X/Y Ki set to: %.4f", param_val);
            }
            else if(parameter.get_name() == "lin_kd"){
                pid_x_->set_kd(parameter.as_double());
                pid_y_->set_kd(parameter.as_double());
                RCLCPP_INFO(node_->get_logger(),"PID X/Y Kd set to: %.4f", param_val);
            }
            else {
                result.successful = false;
                result.reason = "Parameter set incorrectly";
                RCLCPP_WARN(node_->get_logger(), "Parameter unsupported: %s", parameter.get_name().c_str());
            }
        }
        return result;
    }
