#pragma once
#include "Waypoint.hpp"
#include "Structures.hpp"
#include <cmath>
#include <chrono>

/*
 * General flow that this class manages is to:
 *
 * 1. receive waypoints from multiple subscribers
 * callbacks (one for each waypoint type)
 * 2. add waypoint to list of waypoints_
 * 3. Go throught the waypoints_ and update the control_cmd_ based on the current waypoint
 *  each waypoint has its own handling function which changs the control cmd based on the waypoint type
 * 
 */

class PathScheduler {
    public:
    PathScheduler(){

    }
    void add_to_path(const WaypointData &waypoint){
        waypoints_.push_back(Waypoint{waypoint});
    }

    void add_to_path(const std::vector<WaypointData> &waypoint_types){
        for(const auto &waypoint : waypoint_types){
            waypoints_.push_back(Waypoint{waypoint});
        }
    }
    //Message containing target position and control mode(s)
    ControlCmd get_control_cmd() const {
        return control_cmd_;
    }

    void clear_path(){
        waypoints_.clear();
        waypoint_index_ = 0;
    }

    void update(const float &current_x, const float &current_y){
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
private:
    void update_path(){
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
    
    bool waypoint_reached() {
        return waypoint_reached_ && !prev_waypoint_reached_;
    }
    void handle_none_waypoint(){
        ControlCmd cmd;
        cmd.x = last_position_x_;
        cmd.y = last_position_x_;
        cmd.hold_position = true;
        cmd.brake = true;
        update_control_cmd(cmd);
    }

    bool handle_waypoint(const waypoint_msgs::msg::Waypoint &wp){
        float distance = std::hypot(wp.x-position_x_,wp.y-position_y_);
        if(distance <= wp.radius){
            waypoint_reached_ = true;
            return true;
        }
        prev_waypoint_reached_ = waypoint_reached_;
        
        return false;
    }

    void move_to_next_waypoint(){
        waypoint_index_+=1;
        // clear if all waypoints are completed
        if(waypoint_index_ >= waypoints_.size()){
            clear_path();
        }
        waypoint_reached_ = false;
        prev_waypoint_reached_ = false;
    }
    
    bool handle_waypoint_hold(const waypoint_msgs::msg::WaypointHold &wp_hold){
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

    bool handle_waypoint_pass(const waypoint_msgs::msg::WaypointPass wp_pass){
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
    void update_control_cmd(const ControlCmd cmd){
        control_cmd_ = cmd;
    }

    float position_x_{};
    float position_y_{};
    
    float last_position_x_{};
    float last_position_y_{};
    
    bool waypoint_reached_{};
    bool prev_waypoint_reached_{};
    ControlCmd control_cmd_{};
    unsigned int waypoint_index_{};
    std::vector<Waypoint> waypoints_;
    std::vector<Waypoint> null_waypoints_;
    std::chrono::steady_clock::time_point current_waypoint_time_start;

};
