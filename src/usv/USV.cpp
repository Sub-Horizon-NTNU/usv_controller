#include "usv/USV.hpp"


    USV::USV(rclcpp::Node::SharedPtr node): node_(node){
        this->position_controller_ = std::make_unique<PositionController>(node_);
        this->usv_states_ = std::make_unique<USVStates>(node_);
        this->path_scheduler_ = std::make_unique<PathScheduler>(node_);
        control_loop_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&USV::update, this));

        status_publisher_ = node->create_publisher<waypoint_msgs::msg::WaypointStatus>("selene/controller/status",10);
        
        status_publisher_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&USV::publish_status,this));

    }

    void USV::update(){
        path_scheduler_->update(usv_states_->get_states().x, usv_states_->get_states().y);
        std::optional<ControlCmd> new_control_cmd = path_scheduler_->get_control_cmd();
        if(new_control_cmd){
            position_controller_->update(usv_states_->get_states(), *new_control_cmd);
        }
        
        //RCLCPP_INFO(node_->get_logger(),"Current position [%.2f %.2f], Target position [%.2f, %.2f]", usv_states_->get_states().x,usv_states_->get_states().y,path_scheduler_->get_control_cmd().x,path_scheduler_->get_control_cmd().y);
        //RCLCPP_INFO(node_->get_logger(),"vel cmd: [%.2f,%.2f]", position_controller_->get_velocity_cmd().twist.linear.x, position_controller_->get_velocity_cmd().twist.linear.y);
    }

    void USV::publish_status(){
        waypoint_msgs::msg::WaypointStatus wp_status;

        if(!path_scheduler_->get_current_waypoint()){
            return;
        }
        wp_status.header.stamp = node_->now();
        wp_status.target_waypoint = *path_scheduler_->get_current_waypoint();
        double x_diff = path_scheduler_->get_control_cmd().x - usv_states_->get_states().x;
        double y_diff = path_scheduler_->get_control_cmd().y - usv_states_->get_states().y;
        wp_status.distance = std::hypot(x_diff,y_diff);
        wp_status.distance_x=x_diff;
        wp_status.distance_y=y_diff;
        wp_status.heading = usv_states_->get_states().heading;
        

        status_publisher_->publish(wp_status);
    }