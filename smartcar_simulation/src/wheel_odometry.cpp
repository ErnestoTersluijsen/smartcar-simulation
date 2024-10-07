#include "smartcar_simulation/wheel_odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cmath>

WheelOdometry::WheelOdometry() : Node("wheel_odometry")
{
    subscription_ = this->create_subscription<smartcar_msgs::msg::Status>("/smartcar/vehicle_status", 10, std::bind(&WheelOdometry::vehicle_status_callback, this, std::placeholders::_1));
}

void WheelOdometry::vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg)
{
    RCLCPP_INFO(this->get_logger(), "status message rpm: %i", status_msg->engine_speed_rpm);
}

double WheelOdometry::calc_angular_velocity(int32_t rpm, double steering_angle)
{
    double angular_velocity = (calc_linear_velocity(rpm, wheel_diameter_) / wheelbase_) / std::tan(steering_angle);
    return angular_velocity;
}


double WheelOdometry::calc_linear_velocity(int32_t rpm, double wheel_diameter)
{
    double linear_velocity = (rpm * M_PI * wheel_diameter) / 60;
    return linear_velocity;
}

