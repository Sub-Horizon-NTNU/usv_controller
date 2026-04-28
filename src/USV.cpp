#include "USV.hpp"


    USV::USV(rclcpp::Node::SharedPtr node): node_(node){
        this->position_controller_ = std::make_unique<PositionController>(node_);
        this->usv_states_ = std::make_unique<USVStates>(node_);
        this->waypoint_manager_ = std::make_unique<WaypointManager>(node_);
        control_loop_timer_ = node->create_wall_timer(std::chrono::milliseconds(50), std::bind(&USV::update, this));

        status_publisher_ = node->create_publisher<waypoint_msgs::msg::WaypointStatus>("selene/controller/status",10);
        
        status_publisher_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&USV::publish_status,this)
        );
    }

    void USV::update(){
        waypoint_manager_->update(usv_states_->get_states().x, usv_states_->get_states().y,usv_states_->get_states().heading);
        position_controller_->set_waypoint(waypoint_manager_->get_current_waypoint(),waypoint_manager_->get_previous_waypoint());
        position_controller_->update(usv_states_->get_states());
        
    }

    void USV::publish_status(){
        status_publisher_->publish(waypoint_manager_->get_waypoint_status());
    }