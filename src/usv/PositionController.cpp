#include "usv/PositionController.hpp"


    PositionController::PositionController(rclcpp::Node::SharedPtr node): node_(node){
        init_parameters();

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
        max_velocity_ = node->get_parameter("max_linear_velocity").as_double();

        braking_radius_ = node->get_parameter("braking_radius").as_double();

        velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/ap/cmd_vel", 10);
    }

        void PositionController::update(const States &current_states, const ControlCmd &control_commands){
            float diff_x = control_commands.x-current_states.x;
            float diff_y = control_commands.y-current_states.y;

            float velocity;
            float vx;
            float vy;
            float follow_velocity = max_velocity_;

            float distance = std::hypot(diff_x,diff_y);
            // x & y
            if(control_commands.brake && (distance <= braking_radius_) ){
                vx = pid_x_->update(diff_x);
                vy = pid_y_->update(diff_y);
                std::cout << "Here1" << std::endl;
            } else {
                velocity = follow_velocity;
                vx = diff_x/(distance + epsilon) * velocity;
                vy = diff_y/(distance + epsilon) * velocity;
                std::cout << "Here2" << std::endl;
            }
            if(control_commands.heading_on_path){
                heading_ = std::atan2(diff_y,diff_x);
            }

            // heading
            float error = angle_wrap(control_commands.heading-current_states.heading);

            float angular_velocity =  pid_heading_->update(error);

            //send cmd
            set_velocity_cmd(vx,vy,angular_velocity);
            velocity_publisher_->publish(get_velocity_cmd());
        }

        geometry_msgs::msg::TwistStamped PositionController::get_velocity_cmd() const{
            return vel_cmd_;
        }
        double PositionController::angle_wrap(double radians) {
            while (radians > M_PI)  { radians -= 2 * M_PI; }
            while (radians < -M_PI) { radians += 2 * M_PI; }

            return radians;
        }
        void PositionController::set_velocity_cmd(const float &vx, const float &vy, const float &vz){
            vel_cmd_.header.stamp.sec =  node_->now().seconds();
            vel_cmd_.header.stamp.nanosec = node_->now().nanoseconds();
            vel_cmd_.header.frame_id = "map"; // World reference frame
            
            vel_cmd_.twist.linear.x = vy;
            vel_cmd_.twist.linear.y = vx;
            vel_cmd_.twist.angular.z = -vz;
        }

        void PositionController::init_parameters(){
            node_->declare_parameter<double>("yaw_kp", 1.0);
            node_->declare_parameter<double>("yaw_ki", 0.0);
            node_->declare_parameter<double>("yaw_kd", 0.0);

            node_->declare_parameter<double>("lin_kp", 1.0);
            node_->declare_parameter<double>("lin_ki", 0.0);
            node_->declare_parameter<double>("lin_kd", 0.0);

            node_->declare_parameter<double>("braking_radius", 0.0);
            node_->declare_parameter<double>("max_linear_velocity", 0.0);
            node_->declare_parameter<double>("max_angular_velocity", 0.0);
        }