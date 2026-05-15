#include "USV.hpp"


    USV::USV(rclcpp::Node::SharedPtr node): node_(node){
        node_->declare_parameter<double>("initial_heading", 0.0);
        double initial_heading_deg = node_->get_parameter("initial_heading").as_double();
        float initial_heading_rad = static_cast<float>(initial_heading_deg * M_PI / 180.0);

        this->usv_transform_handler_ = std::make_unique<USVTransformHandler>(node_);
        this->position_controller_ = std::make_unique<PositionController>(node_);
        this->waypoint_manager_ = std::make_unique<WaypointManager>(node_, initial_heading_rad);
        control_loop_timer_ = node->create_wall_timer(std::chrono::milliseconds(50), std::bind(&USV::update, this));

        status_publisher_ = node->create_publisher<waypoint_msgs::msg::WaypointStatus>("selene/controller/status",10);
        
        status_publisher_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&USV::publish_status,this)
        );
    }

    void USV::update(){
        waypoint_manager_->update(usv_transform_handler_->get_usv_position().translation.x, usv_transform_handler_->get_usv_position().translation.y,usv_transform_handler_->get_heading());
        position_controller_->set_waypoint(waypoint_manager_->get_current_waypoint(),waypoint_manager_->get_previous_waypoint());
        States states{};
        states.x = usv_transform_handler_->get_usv_position().translation.x;
        states.y = usv_transform_handler_->get_usv_position().translation.y;
        states.heading = usv_transform_handler_->get_heading();
        position_controller_->update(states);
        
    }

    void USV::publish_status(){
        status_publisher_->publish(position_controller_->get_waypoint_status());
    }