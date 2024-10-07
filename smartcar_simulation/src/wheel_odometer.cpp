#include "smartcar_simulation/wheel_odometer.hpp"

#include <rclcpp/rclcpp.hpp>

WheelOdometer::WheelOdometer() : Node("wheel_odometer")
{
    subscription_ = this->create_subscription<smartcar_msgs::msg::Status>("/smartcar/vehicle_status", 10, std::bind(&WheelOdometer::vehicle_status_callback, this, std::placeholders::_1));
}

void WheelOdometer::vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg)
{
    RCLCPP_INFO(this->get_logger(), "status message rpm: %i", status_msg->engine_speed_rpm);
}
