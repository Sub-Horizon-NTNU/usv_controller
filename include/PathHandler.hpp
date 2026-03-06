#include <Structures.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class PathHandler{
public:
    PathHandler(const float &wp_radius): wp_radius_(wp_radius) {
        waypoints_ = generate_circle_path(100, 6.0, 0, 0.0);
    }

    //Test functions for generating path
   std::vector<Waypoint> generate_circle_path(const int number_of_points, const double radius, float x, float y){
        std::vector<Waypoint> waypoints;
        for(int i = 0; i < number_of_points; i++){
            double angle = static_cast<double>(i)*2*M_PI/number_of_points;
            Waypoint waypoint{};
            waypoint.x = std::cos(angle)*radius+x;
            waypoint.y = std::sin(angle)*radius+y;
            waypoints.push_back(waypoint);
        }
        return waypoints;
    } 


    Waypoint get_target_waypoint(){
        return current_target_waypoint_;
    }

    void update(float x, float y){

        //Check if position is new
        bool updated_position{};
        if(current_position_x_ != x || current_position_y_ != y){
            updated_position = true;
        }

        //update current position
        current_position_x_ = x;
        current_position_y_ = y;

        //Check if the new targeted waypoint can be updated.
        double delta_p = std::hypot(current_position_x_-current_target_waypoint_.x,current_position_y_-current_target_waypoint_.y);
        if( (std::abs(delta_p) < wp_radius_ && updated_position)){
           current_waypoint_index_+=1;
           //This should rather default to clearing the list of waypoints!
           if(current_waypoint_index_> waypoints_.size()){
               current_waypoint_index_ = 0;
           }
           //update target waypoint
           current_target_waypoint_ = waypoints_[current_waypoint_index_];
           if(current_target_waypoint_.hold_at_point){
                
           }

        }
    }

    


private:

    float current_position_x_;
    float current_position_y_;
    float heading_;
    float wp_radius_;
    std::vector<Waypoint> waypoints_;
    Waypoint current_target_waypoint_;
    int current_waypoint_index_{};
    
    

};