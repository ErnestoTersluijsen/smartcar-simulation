#include "smartcar_simulation/wheel_odometer.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometer>());
    rclcpp::shutdown();
    
    return 0;
}
