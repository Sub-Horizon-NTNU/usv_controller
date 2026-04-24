#include "usv_controller/WaypointManager.hpp"

    WaypointManager::WaypointManager(rclcpp::Node::SharedPtr node): node_(node){
        waypoint_subscriber_ = node_->create_subscription<waypoint_msgs::msg::Waypoint>(
            "selene/controller/waypoint",
            10, 
            [this](const waypoint_msgs::msg::Waypoint waypoint){ add_to_path(waypoint); });
        
        waypoint_list_subscriber_ = node_->create_subscription<waypoint_msgs::msg::Waypoints>(
            "selene/controller/waypoints",
            10, 
            [this](const waypoint_msgs::msg::Waypoints waypoints){ add_list_to_path(waypoints); });
        
        clear_waypoints_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
            "selene/controller/clear_waypoints",
            10, 
            [this](const std_msgs::msg::Bool){ clear_path(); });
    }
    
    void WaypointManager::add_to_path(const waypoint_msgs::msg::Waypoint &waypoint){
        //TODO: add logic to check wp first before adding it
        waypoints_.push_back(waypoint);
    }

    void WaypointManager::add_list_to_path(const waypoint_msgs::msg::Waypoints &waypoints){
        for(const waypoint_msgs::msg::Waypoint &waypoint : waypoints.waypoints){
            waypoints_.push_back(waypoint);
        }
    }
    //Message containing target position and control mode(s)
    ControlCmd WaypointManager::get_control_cmd() const {
        return control_cmd_;
    }

    void WaypointManager::clear_path(){
        waypoints_.clear();
        waypoints_.shrink_to_fit();
        waypoint_index_ = 0;
    }

    void WaypointManager::update(const float &current_x, const float &current_y){
        bool updated_position{};
        if( (position_x_ != current_x) || (position_y_!= current_y ) ){
            updated_position = true;
        }
        //Update current position
        position_x_ = current_x;
        position_y_ = current_y;

        if(updated_position){
            update_path();
        }
    }

    waypoint_msgs::msg::Waypoint WaypointManager::get_current_waypoint(){
       return most_recent_waypoint_;
    }

    void WaypointManager::update_path(){
        if(waypoints_.empty()){
            handle_none_waypoint();
            return;
        }
        waypoint_msgs::msg::Waypoint &target_wp = waypoints_[waypoint_index_];

        int waypoint_type = target_wp.type;

        bool waypoint_ok{};

        switch(waypoint_type){
            case waypoint_msgs::msg::Waypoint::PASS:
                waypoint_ok = handle_waypoint_pass(target_wp);
                break;
            case waypoint_msgs::msg::Waypoint::HOLD:
                waypoint_ok = handle_waypoint_hold(target_wp);
                break;
            default:
                break;
        }

        if(waypoint_ok){
            move_to_next_waypoint();
        }
    }

    bool WaypointManager::waypoint_reached() {
        return waypoint_reached_ && !prev_waypoint_reached_;
    }

    void WaypointManager::handle_none_waypoint(){
        ControlCmd cmd;
        cmd.x = last_position_x_;
        cmd.y = last_position_y_;
        cmd.hold_position = true;
        cmd.brake = true;
        update_control_cmd(cmd);
    }

    bool WaypointManager::handle_waypoint(const waypoint_msgs::msg::Waypoint &wp){
        float distance = std::hypot(wp.x-position_x_,wp.y-position_y_);
        if(distance <= wp.radius){
            prev_waypoint_reached_ = waypoint_reached_;
            waypoint_reached_ = true;
            return true;
        }
        return false;
    }

    void WaypointManager::move_to_next_waypoint(){
        waypoint_index_+=1;
        most_recent_waypoint_ = waypoints_[waypoint_index_];
        // clear if all waypoints are completed
        if(waypoint_index_ >= waypoints_.size()){
            clear_path();
        }
        waypoint_reached_ = false;
        prev_waypoint_reached_ = false;
    }

    bool WaypointManager::handle_waypoint_hold(const waypoint_msgs::msg::Waypoint &wp_hold){
        //Check general waypoint conditions
        bool wp_check = handle_waypoint(wp_hold);

        bool wp_check_hold{};
       
        //start the clock if the usv has arrived at the wp
        if(waypoint_reached()){
            current_waypoint_time_start = std::chrono::steady_clock::time_point::clock::now();
        }
        //Check how long waypoint has been held
        float hold_time_passed = std::chrono::duration<float>(std::chrono::steady_clock::time_point::clock::now()-current_waypoint_time_start).count();
        if( (hold_time_passed >= wp_hold.time_to_hold)){
            wp_check_hold = true;
        }

        //update control cmd with regards to the waypoint type;
        ControlCmd control_cmd;
        control_cmd.x = wp_hold.x;
        control_cmd.y = wp_hold.y;
        control_cmd.heading_on_path = wp_hold.keep_on_track;
        control_cmd.hold_position = wp_hold.hold;
        
        update_control_cmd(control_cmd);

        return wp_check && wp_check_hold;
    }

    bool WaypointManager::handle_waypoint_pass(const waypoint_msgs::msg::Waypoint &wp_pass){
        bool wp_check = handle_waypoint(wp_pass);

        ControlCmd cmd;
        cmd.x = wp_pass.x;
        cmd.y = wp_pass.y;
        cmd.heading_on_path = wp_pass.keep_on_track;
        
        update_control_cmd(cmd);
        
        last_position_x_ = cmd.x;
        last_position_y_ = cmd.y;

        return wp_check;
    }
    //Must be updated by overwriting the existing control command variable.
    void WaypointManager::update_control_cmd(const ControlCmd cmd){
        control_cmd_ = cmd;
    }

 
