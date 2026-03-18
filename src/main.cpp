

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "Controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Controller> controller = std::make_shared<Controller>();
    controller->init();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
