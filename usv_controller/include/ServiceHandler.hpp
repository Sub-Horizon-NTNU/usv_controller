
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_home.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include <chrono>
#include <mavros_msgs/srv/detail/set_mode__struct.hpp>
#include <rclcpp/executors.hpp>
#include <thread>


class ServiceHandler{
public:

ServiceHandler(){
    command_home_ = this->create_client<mavros_msgs::srv::CommandHome>("/mavros/cmd/set_home");

}


  bool set_home(){
        auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
        request->current_gps = true;
    
          while (!command_home_->wait_for_service(std::chrono::milliseconds(1000))) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Set home service failed");
            return false;
            }
            RCLCPP_INFO(this->get_logger(), "Set home service not available, retrying");
        }
        auto future = command_home_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->shared_from_this(),future) == rclcpp::FutureReturnCode::SUCCESS){
            if(future.get()->success){
                return true;
            }
            return false;
        }
        return false;
    }


    bool set_mode(std::string mode){
        if(mode != "MANUAL" && mode != "AUTO" && mode !="GUIDED"){
            RCLCPP_ERROR(this->get_logger(),"Set mode doesn't exist");
            return false;
        } 
        while(!set_mode_client_->wait_for_service(std::chrono::milliseconds(1000))){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(),"Set mode failed");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Setting mode failed");
        }
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        auto future = set_mode_client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->shared_from_this(),future) == rclcpp::FutureReturnCode::SUCCESS){
            auto result = future.get();
            if(result->mode_sent){
                return true;
            }
            return false;
        }
        return false;
    }

private:
    rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr command_home_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;



};