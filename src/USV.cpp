#include "USV.hpp"


    USV::USV(rclcpp::Node::SharedPtr node): node_(node){
        this->position_controller_ = std::make_unique<PositionController>(node_);
        this->usv_states_ = std::make_unique<USVStates>(node_);
        this->waypoint_manager_ = std::make_unique<WaypointManager>(node_);
        control_loop_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&USV::update, this));

        status_publisher_ = node->create_publisher<waypoint_msgs::msg::WaypointStatus>("selene/controller/status",10);
        
        status_publisher_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&USV::publish_status,this));

    }

    void USV::update(){
        
        waypoint_manager_->update(usv_states_->get_states().x, usv_states_->get_states().y);
        
        position_controller_->update(usv_states_->get_states(), waypoint_manager_->get_control_cmd());
        
        
        //RCLCPP_INFO(node_->get_logger(),"Current position [%.2f %.2f], Target position [%.2f, %.2f]", usv_states_->get_states().x,usv_states_->get_states().y,path_scheduler_->get_control_cmd().x,path_scheduler_->get_control_cmd().y);
        //RCLCPP_INFO(node_->get_logger(),"vel cmd: [%.2f,%.2f]", position_controller_->get_velocity_cmd().twist.linear.x, position_controller_->get_velocity_cmd().twist.linear.y);
    }

    void USV::publish_status(){
        waypoint_msgs::msg::WaypointStatus wp_status;
        wp_status.header.stamp = node_->now();
        wp_status.target_waypoint = waypoint_manager_->get_current_waypoint();
        double x_diff = waypoint_manager_->get_control_cmd().x - usv_states_->get_states().x;
        double y_diff = waypoint_manager_->get_control_cmd().y - usv_states_->get_states().y;
        wp_status.distance = std::hypot(x_diff,y_diff);
        wp_status.distance_x=x_diff;
        wp_status.distance_y=y_diff;
        wp_status.heading = usv_states_->get_states().heading;
        

        status_publisher_->publish(wp_status);
    }