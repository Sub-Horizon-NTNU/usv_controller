#include "usv_controller/WaypointManager.hpp"
#include <rclcpp/qos.hpp>
#include <waypoint_msgs/msg/detail/waypoint__struct.hpp>
#include <waypoint_msgs/msg/detail/waypoint_status__struct.hpp>

    WaypointManager::WaypointManager(rclcpp::Node::SharedPtr node, float current_heading): node_(node){
        waypoint_subscriber_ = node_->create_subscription<waypoint_msgs::msg::Waypoint>(
            "selene/controller/waypoint",
            rclcpp::QoS(10).transient_local(), 
            [this](const waypoint_msgs::msg::Waypoint waypoint){ add_to_path(waypoint); });
        
        waypoint_list_subscriber_ = node_->create_subscription<waypoint_msgs::msg::Waypoints>(
            "selene/controller/waypoints",
            rclcpp::QoS(10).transient_local(), 
            [this](const waypoint_msgs::msg::Waypoints waypoints){ add_list_to_path(waypoints); });
        
        clear_waypoints_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
            "selene/controller/clear_waypoints",
            rclcpp::QoS(10).transient_local().reliable(), 
            [this](const std_msgs::msg::Bool){ 
                RCLCPP_INFO(node_->get_logger(), "Waypoints cleared, holding last commanded position");
                clear_path();
            });

            heading_ = current_heading;
            handle_none_waypoint();
    }
    
    void WaypointManager::add_to_path(const waypoint_msgs::msg::Waypoint &waypoint){
        hold_position_ = false;
        if(waypoints_.empty()){
            previous_waypoint_.x = position_x_;
            previous_waypoint_.y = position_y_;
            previous_waypoint_.heading = heading_;
            most_recent_waypoint_ = waypoint; 
        }
        waypoints_.push_back(waypoint);
        print_waypoint(waypoint);
    }
    
    void WaypointManager::print_waypoint(const waypoint_msgs::msg::Waypoint &waypoint){
        RCLCPP_INFO(
            node_->get_logger(),
            "WAYPOINT RECEIVED: POSITION [%.2f, %.2f] | HEADING: %.2f | VELOCITY: %.2f | KEEP ON TRACK: %d | HOLD: %d",
            waypoint.x,
            waypoint.y,
            waypoint.heading,
            waypoint.velocity,
            waypoint.keep_on_track,
            waypoint.hold
        );
    }

    void WaypointManager::add_list_to_path(const waypoint_msgs::msg::Waypoints &waypoints){
        hold_position_ = false;
        RCLCPP_INFO(node_->get_logger(), "Received %zu waypoints", waypoints.waypoints.size());

        if(waypoints_.empty()){
            previous_waypoint_.x = position_x_;
            previous_waypoint_.y = position_y_;
            previous_waypoint_.heading = heading_;
            most_recent_waypoint_ = waypoints.waypoints[0];
        }
        for(const waypoint_msgs::msg::Waypoint &waypoint : waypoints.waypoints){
            waypoints_.push_back(waypoint);
            print_waypoint(waypoint);
        }
    }

    void WaypointManager::clear_path(){
        waypoints_.clear();
        waypoints_.shrink_to_fit();
        waypoint_index_ = 0;
        handle_none_waypoint();
    }

    void WaypointManager::update(const float &current_x, const float &current_y, const float &heading){
        bool updated_position{};
        if( (position_x_ != current_x) || (position_y_!= current_y ) ){
            updated_position = true;
        }
        //Update current position
        position_x_ = current_x;
        position_y_ = current_y;
        heading_ = heading;

        if(updated_position){
            update_path();
        }
    }

    const std::vector<waypoint_msgs::msg::Waypoint>  WaypointManager::get_waypoints() {
        std::vector<waypoint_msgs::msg::Waypoint> waypoints;
        waypoints.assign(waypoints_.begin()+waypoint_index_,waypoints_.end());
        return waypoints;
    }

    waypoint_msgs::msg::Waypoint WaypointManager::get_current_waypoint(){
       return most_recent_waypoint_;
    }

    waypoint_msgs::msg::Waypoint WaypointManager::get_previous_waypoint(){
       return previous_waypoint_;
    }

    void WaypointManager::update_path(){

        if(waypoints_.empty()){ 
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
        if(!hold_position_){
            most_recent_waypoint_ = previous_waypoint_;
            most_recent_waypoint_.hold = true;
            //most_recent_waypoint_.x = position_x_;
            //most_recent_waypoint_.y = position_y_;
            most_recent_waypoint_.heading = heading_;
            most_recent_waypoint_.radius = 0.1;
            most_recent_waypoint_.keep_on_track = false;
            hold_position_=true;
            RCLCPP_INFO(node_->get_logger(),"NO WAYPOINTS LEFT TO NAVIGATE TOWARDS, HOLDING LAST POSITION");
        }
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
        previous_waypoint_ = waypoints_[waypoint_index_];
        waypoint_index_+=1;
        // clear if all waypoints are completed
        if(waypoint_index_ >= waypoints_.size()){
            clear_path();
            return;
        }

        most_recent_waypoint_ = waypoints_[waypoint_index_];
        waypoint_reached_ = false;
        prev_waypoint_reached_ = false;
    }

    bool WaypointManager::handle_waypoint_hold(const waypoint_msgs::msg::Waypoint &wp_hold){
        //Check general waypoint conditions
        bool wp_check = handle_waypoint(wp_hold);

        //start the clock if the usv has arrived at the wp
        if(wp_check && !hold_timer_started_){
            current_waypoint_time_start = std::chrono::steady_clock::time_point::clock::now();
            hold_timer_started_ = true;
        }
        //Check how long waypoint has been held
        if(hold_timer_started_){
            float hold_time_passed = std::chrono::duration<float>(std::chrono::steady_clock::time_point::clock::now()-current_waypoint_time_start).count();
            if( (hold_time_passed >= wp_hold.time_to_hold)){
                hold_timer_started_ = false;
                return true;
            }
        }
        return false;
    }

    bool WaypointManager::handle_waypoint_pass(const waypoint_msgs::msg::Waypoint &wp_pass){
        bool wp_check = handle_waypoint(wp_pass);

        return wp_check;
    }

    waypoint_msgs::msg::WaypointStatus WaypointManager::get_waypoint_status() {
        waypoint_msgs::msg::WaypointStatus wp_status;
        wp_status.header.stamp = node_->now();
        wp_status.current_x = position_x_;
        wp_status.current_y = position_y_;
        wp_status.target_waypoint = get_current_waypoint();
        wp_status.distance_x = position_x_- get_current_waypoint().x;
        wp_status.distance_y = position_y_- get_current_waypoint().y;
        wp_status.heading = heading_;
        wp_status.distance = std::hypot(wp_status.distance_x,wp_status.distance_y);
        return wp_status;
    }
 

 
