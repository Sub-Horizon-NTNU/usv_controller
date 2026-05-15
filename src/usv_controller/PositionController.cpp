#include "usv_controller/PositionController.hpp"


    PositionController::PositionController(rclcpp::Node::SharedPtr node): node_(node){
        init_parameters();
        double initial_heading_deg = node_->get_parameter("initial_heading").as_double();
        filtered_heading_ = initial_heading_deg * M_PI / 180.0;
        velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
        parameter_callback_ = node_->add_on_set_parameters_callback(std::bind(&PositionController::handle_changed_parameters,this,std::placeholders::_1));
    }
        void PositionController::update(const States &current_states){
            const double &x = current_states.x;
            const double &y = current_states.y;
            position_x_ = current_states.x;
            position_y_ = current_states.y;
            heading_ = current_states.heading;
            double desired_velocity = target_waypoint.velocity;
            double R = lookahead_distance_; // Lookahead
            double delta{};
            double vx;
            double vy;
            if(desired_velocity <=0.01){
                desired_velocity = 2.0;
            }
            pid_x_->set_max_output(desired_velocity);
            pid_y_->set_max_output(desired_velocity);

            double yaw_vel{};
            double alpha_k = atan2(target_waypoint.y-last_waypoint.y,target_waypoint.x-last_waypoint.x);  // αk := atan2 (yk+1 − yk, xk+1 − xk) ∈ S (10.55)
            double s = (x-last_waypoint.x)*cos(alpha_k) +(y-last_waypoint.y)*sin(alpha_k);                // along-track distance  s(t) = [x(t) − xk] cos(αk) + [y(t) − yk] sin(αk) (10.58)
            double e =-(x-last_waypoint.x)*sin(alpha_k) + (y-last_waypoint.y)*cos(alpha_k);     //cross track error:  e(t) = −[x(t) − xk] sin(αk) + [y(t) − yk] cos(αk) (10.59)
            double p = std::hypot(last_waypoint.x - target_waypoint.x,last_waypoint.y-target_waypoint.y);
            if(s >= p){
                s=p;
                R = 0.0;
            }
            if(std::abs(e) < R){
                delta = sqrt(R*R-e*e);
            } else {
                delta = 0;
            }

            double x_los = last_waypoint.x + (s+delta)*cos(alpha_k);
            double y_los = last_waypoint.y + (s+delta)*sin(alpha_k);

            double X_d = atan2(y_los-y,x_los-x);

            //double e_x = last_waypoint.x+s*cos(alpha_k); // cross track coordinates
            //double e_y = last_waypoint.y+s*sin(alpha_k); 

            if(target_waypoint.hold && (((p-s < R) || (p-s < target_waypoint.radius)) || (target_waypoint.type == waypoint_msgs::msg::Waypoint::HOLD))){
                double error_x_world = target_waypoint.x-x;
                double error_y_world = target_waypoint.y-y;

                vx = pid_x_->update(error_x_world);
                vy = pid_y_->update(error_y_world);
            } else {
                vx = desired_velocity*cos(X_d);
                vy = desired_velocity*sin(X_d);
            }

            double target_heading;
            if(target_waypoint.keep_on_track) {
                target_heading = X_d;
            } else {
                target_heading = target_waypoint.heading;
            }

            filtered_heading_ = target_heading * alpha_ + (1-alpha_)*filtered_heading_;
            //Logging 
            target_heading_ = filtered_heading_;

            yaw_vel = pid_heading_->update(angle_wrap(filtered_heading_ - current_states.heading));

            send_velocity_cmd_world(vx, vy, yaw_vel);
        }

        geometry_msgs::msg::TwistStamped PositionController::get_velocity_cmd() const{
            return vel_cmd_;
        }

        void PositionController::set_waypoint(const waypoint_msgs::msg::Waypoint &wp, const waypoint_msgs::msg::Waypoint &last_wp){
            target_waypoint = wp;
            last_waypoint = last_wp;
        }
        
        double PositionController::angle_wrap(double radians) {
            while (radians > M_PI)  { radians -= 2 * M_PI; }
            while (radians < -M_PI) { radians += 2 * M_PI; }

            return radians;
        }

        void PositionController::send_velocity_cmd_world(const float &vx, const float &vy, const float &vz){
            vel_cmd_.header.stamp =  node_->now();
            vel_cmd_.header.frame_id = "map"; // World reference frame
            // NED to ENU
            vel_cmd_.twist.linear.x = vy;
            vel_cmd_.twist.linear.y = vx;
            vel_cmd_.twist.angular.z = -vz;
            velocity_publisher_->publish(vel_cmd_);
        }

         void PositionController::send_velocity_cmd_body(const float &vx, const float &vy, const float &vz){
            //  BODY to NED world
            double vx_world = cos(heading_)*vx-sin(heading_)*vy;
            double vy_world = sin(heading_)*vx+cos(heading_)*vy;
            vel_cmd_.header.stamp =  node_->now();
            vel_cmd_.header.frame_id = "map"; // World reference frame
            // NED to ENU
            vel_cmd_.twist.linear.x = vy_world;
            vel_cmd_.twist.linear.y = vx_world;
            vel_cmd_.twist.angular.z = -vz;
            velocity_publisher_->publish(vel_cmd_);
        }

        void PositionController::init_parameters(){
            node_->declare_parameter<double>("yaw_kp", 1.0);
            node_->declare_parameter<double>("yaw_ki", 0.01);
            node_->declare_parameter<double>("yaw_kd", 0.01);

            node_->declare_parameter<double>("lin_kp", 1.0);
            node_->declare_parameter<double>("lin_ki", 0.1);
            node_->declare_parameter<double>("lin_kd", 0.05);

            node_->declare_parameter<double>("lookahead_distance", 1.0);
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
