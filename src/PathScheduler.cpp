#include "PathScheduler.hpp"



    PathScheduler::PathScheduler(rclcpp::Node::SharedPtr node): node_(node){

        waypoint_hold_subscriber_ = node_->create_subscription<waypoint_msgs::msg::WaypointHold>(
            "/selene/waypoint/hold",
            10, [this](const waypoint_msgs::msg::WaypointHold wp_hold){ add_to_path(wp_hold); });

        waypoint_pass_subscriber_ = node_->create_subscription<waypoint_msgs::msg::WaypointPass>(
            "/selene/waypoint/pass",
            10, [this](const waypoint_msgs::msg::WaypointPass wp_pass){ add_to_path(wp_pass); });
    }
    void PathScheduler::add_to_path(const WaypointData &waypoint){
        waypoints_.push_back(Waypoint{waypoint});
    }

    void PathScheduler::add_to_path(const std::vector<WaypointData> &waypoint_types){
        for(const auto &waypoint : waypoint_types){
            waypoints_.push_back(Waypoint{waypoint});
        }
    }
    //Message containing target position and control mode(s)
    ControlCmd PathScheduler::get_control_cmd() const {
        return control_cmd_;
    }

    void PathScheduler::clear_path(){
        waypoints_.clear();
        waypoint_index_ = 0;
    }

    void PathScheduler::update(const float &current_x, const float &current_y){
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

    void PathScheduler::update_path(){
        if(waypoints_.empty()){
            handle_none_waypoint();
            return;
        }

        Waypoint &target_wp = waypoints_[waypoint_index_];

        int wp_type_index = target_wp.Data.index(); // get index of the variant type

        bool waypoint_ok{};

        switch(wp_type_index){
            case WaypointTypeIndex::HOLD:
                waypoint_ok = handle_waypoint_hold(std::get<waypoint_msgs::msg::WaypointHold>(target_wp.Data));
                break;

            case WaypointTypeIndex::PASS:
                waypoint_ok = handle_waypoint_pass(std::get<waypoint_msgs::msg::WaypointPass>(target_wp.Data));
                break;

            default:
                break;
        }
        if(waypoint_ok){

            move_to_next_waypoint();
        }
    }

    bool PathScheduler::waypoint_reached() {
        return waypoint_reached_ && !prev_waypoint_reached_;
    }

    void PathScheduler::handle_none_waypoint(){
        ControlCmd cmd;
        cmd.x = last_position_x_;
        cmd.y = last_position_x_;
        cmd.hold_position = true;
        cmd.brake = true;
        update_control_cmd(cmd);
    }

    bool PathScheduler::handle_waypoint(const waypoint_msgs::msg::Waypoint &wp){
        float distance = std::hypot(wp.x-position_x_,wp.y-position_y_);
        if(distance <= wp.radius){
            waypoint_reached_ = true;
            return true;
        }
        prev_waypoint_reached_ = waypoint_reached_;

        return false;
    }

    void PathScheduler::move_to_next_waypoint(){
        waypoint_index_+=1;
        // clear if all waypoints are completed
        if(waypoint_index_ >= waypoints_.size()){
            clear_path();
        }
        waypoint_reached_ = false;
        prev_waypoint_reached_ = false;
    }

    bool PathScheduler::handle_waypoint_hold(const waypoint_msgs::msg::WaypointHold &wp_hold){
        //Check general waypoint conditions
        bool wp_check = handle_waypoint(wp_hold.waypoint);
        if(!wp_check){
            return false;
        }
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
        control_cmd.x = wp_hold.waypoint.x;
        control_cmd.y = wp_hold.waypoint.y;
        control_cmd.heading_on_path = true;
        control_cmd.hold_position = true;
        update_control_cmd(control_cmd);

        return wp_check && wp_check_hold;
    }

    bool PathScheduler::handle_waypoint_pass(const waypoint_msgs::msg::WaypointPass wp_pass){
        bool wp_check = handle_waypoint(wp_pass.waypoint);
        ControlCmd cmd;
        cmd.x = wp_pass.waypoint.x;
        cmd.y = wp_pass.waypoint.y;
        cmd.heading_on_path = true;
        update_control_cmd(cmd);
        last_position_x_ = cmd.x;
        last_position_y_ = cmd.y;

        return wp_check;
    }

    //Must be updated by overwriting the existing control command variable.
    void PathScheduler::update_control_cmd(const ControlCmd cmd){
        control_cmd_ = cmd;
    }
