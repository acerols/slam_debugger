#include <orb_slam_debugger/orb_slam_debugger.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    auto node = std::make_shared<orb_slam_debugger>("orb_slam_debugger", options);
    node->init();

    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}