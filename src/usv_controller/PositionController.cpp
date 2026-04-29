#include "usv_controller/PositionController.hpp"


    PositionController::PositionController(rclcpp::Node::SharedPtr node): node_(node){
        init_parameters();
        velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    }
        void PositionController::update(const States &current_states){
            const double &x = current_states.x;
            const double &y = current_states.y;
            const double &desired_velocity = target_waypoint.velocity;
            
            double vx;
            double vy;

            double R = 1.0; // Lookahead

            double yaw_vel{};
            double alpha_k = atan2(target_waypoint.y-last_waypoint.y,target_waypoint.x-last_waypoint.x);  // αk := atan2 (yk+1 − yk, xk+1 − xk) ∈ S (10.55)
            double s = (x-last_waypoint.x)*cos(alpha_k) +(y-last_waypoint.y)*sin(alpha_k);                // along-track distance  s(t) = [x(t) − xk] cos(αk) + [y(t) − yk] sin(αk) (10.58)
            double e =-(x-last_waypoint.x)*sin(alpha_k) + (y-last_waypoint.y)*cos(alpha_k);     //cross track error:  e(t) = −[x(t) − xk] sin(αk) + [y(t) − yk] cos(αk) (10.59)
            double p = std::hypot(last_waypoint.x - target_waypoint.x,last_waypoint.y-target_waypoint.y);
            if(s >= p){
                s=p;
                R = 0.0;
            }

            double x_los = last_waypoint.x + (s+R)*cos(alpha_k);
            double y_los = last_waypoint.y + (s+R)*sin(alpha_k);

            double X_d = atan2(y_los-y,x_los-x);

            //double e_x = last_waypoint.x+s*cos(alpha_k); // cross track coordinates
            //double e_y = last_waypoint.y+s*sin(alpha_k); 

            if(target_waypoint.hold && ((p-s < R) || (p-s < target_waypoint.radius))){
                vx = pid_x_->update(target_waypoint.x-x);
                vy = pid_y_->update(target_waypoint.y-y);
            } else {
                vx = cos(X_d)*max_velocity_;
                vy = sin(X_d)*max_velocity_;
            }

            if(target_waypoint.keep_on_track) {
                yaw_vel = pid_heading_->update(angle_wrap(X_d - current_states.heading));
            } else {
                yaw_vel = pid_heading_->update(angle_wrap(target_waypoint.heading - current_states.heading));
            }
            //RCLCPP_INFO(node_->get_logger(),"Heading X_d: %.2f  Angular vel cmd: %.2f", X_d, yaw_vel);
            send_velocity_cmd(vx, vy, yaw_vel);
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