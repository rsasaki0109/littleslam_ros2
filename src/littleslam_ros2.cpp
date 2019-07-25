#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "littleslam_ros2/littleslam_ros2_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto littleslam = std::make_shared<littleslam_ros2::Littleslam>();
    rclcpp::spin(littleslam);
    rclcpp::shutdown();
    return 0;
}

