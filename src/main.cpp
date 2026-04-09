

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "USVNode.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<USVNode> controller = std::make_shared<USVNode>();
    controller->init();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
