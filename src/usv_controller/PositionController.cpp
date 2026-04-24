#include "usv_controller/PositionController.hpp"


    PositionController::PositionController(rclcpp::Node::SharedPtr node): node_(node){
        init_parameters();
        velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    }
        void PositionController::update(const States &current_states, const ControlCmd &control_commands){
            if(control_commands.x != last_control_cmd.x || control_commands.y != last_control_cmd.y){
                last_control_cmd = prev_control_cmd_;
                prev_control_cmd_ = control_commands;
            }

            float diff_x = control_commands.x-current_states.x;
            float diff_y = control_commands.y-current_states.y;

            float velocity;
            float vx;
            float vy;
            float follow_velocity = max_velocity_;
            float heading_error;

            float distance = std::hypot(diff_x,diff_y);

            // x & y
            if(control_commands.hold_position && (distance <= braking_radius_) ){
                // Brake 
                vx = pid_x_->update(diff_x);
                vy = pid_y_->update(diff_y);
            } 
            else {
                //Brake off
                velocity = follow_velocity;
                vx = diff_x/(distance + epsilon) * velocity;
                vy = diff_y/(distance + epsilon) * velocity;
            }
            // heading
            if(control_commands.heading_on_path){
                heading_ = std::atan2(diff_y,diff_x);
                //TODO: improve heading
                heading_error = angle_wrap(heading_-current_states.heading);
            } 
            else {
                heading_error = angle_wrap(control_commands.heading-current_states.heading);
            }
            float angular_velocity =  pid_heading_->update(heading_error);
            
            //send cmd
            send_velocity_cmd(vx,vy,angular_velocity);

            //RCLCPP_INFO(node_->get_logger(),"Velocity %.2f",angular_velocity);
            prev_control_cmd_ = control_commands;
        }

        geometry_msgs::msg::TwistStamped PositionController::get_velocity_cmd() const{
            return vel_cmd_;
        }

        double PositionController::angle_wrap(double radians) {
            while (radians > M_PI)  { radians -= 2 * M_PI; }
            while (radians < -M_PI) { radians += 2 * M_PI; }

            return radians;
        }
        void PositionController::send_velocity_cmd(const float &vx, const float &vy, const float &vz){
            vel_cmd_.header.stamp =  node_->now();
            vel_cmd_.header.frame_id = "map"; // World reference frame
            // NED to ENU
            vel_cmd_.twist.linear.x = vy;
            vel_cmd_.twist.linear.y = vx;
            vel_cmd_.twist.angular.z = -vz;
            velocity_publisher_->publish(vel_cmd_);
        }

        void PositionController::init_parameters(){
            node_->declare_parameter<double>("yaw_kp", 1.0);
            node_->declare_parameter<double>("yaw_ki", 0.01);
            node_->declare_parameter<double>("yaw_kd", 0.01);

            node_->declare_parameter<double>("lin_kp", 1.0);
            node_->declare_parameter<double>("lin_ki", 0.1);
            node_->declare_parameter<double>("lin_kd", 0.0);

            node_->declare_parameter<double>("braking_radius", 2.0);
            node_->declare_parameter<double>("max_linear_velocity", 0.5);
            node_->declare_parameter<double>("max_angular_velocity", 0.5);

            this->pid_x_ = std::make_shared<PID>();
            this->pid_y_ = std::make_shared<PID>();
            this->pid_heading_ = std::make_shared<PID>();

            pid_x_->set_kp(node_->get_parameter("lin_kp").as_double());
            pid_x_->set_ki(node_->get_parameter("lin_ki").as_double());
            pid_x_->set_kd(node_->get_parameter("lin_kd").as_double());
            pid_x_->set_max_output(node_->get_parameter("max_linear_velocity").as_double());

            pid_y_->set_kp(node_->get_parameter("lin_kp").as_double());
            pid_y_->set_ki(node_->get_parameter("lin_ki").as_double());
            pid_y_->set_kd(node_->get_parameter("lin_kd").as_double());
            pid_y_->set_max_output(node_->get_parameter("max_linear_velocity").as_double());

            pid_heading_->set_kp(node_->get_parameter("yaw_kp").as_double());
            pid_heading_->set_ki(node_->get_parameter("yaw_ki").as_double());
            pid_heading_->set_kd(node_->get_parameter("yaw_kd").as_double());
            pid_heading_->set_max_output(node_->get_parameter("max_angular_velocity").as_double());
            max_velocity_ = node_->get_parameter("max_linear_velocity").as_double();

            braking_radius_ = node_->get_parameter("braking_radius").as_double();
        }