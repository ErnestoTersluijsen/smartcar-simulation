#include "smartcar_simulation/wheel_odometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>

WheelOdometry::WheelOdometry() : Node("wheel_odometry"), wheelbase_(0.257), wheel_diameter_(0.064)
{
    subscription_ = this->create_subscription<smartcar_msgs::msg::Status>("/smartcar/vehicle_status", 10, std::bind(&WheelOdometry::vehicle_status_callback, this, std::placeholders::_1));
}

void WheelOdometry::vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg)
{
    RCLCPP_INFO(this->get_logger(), "status message rpm: %i", status_msg->engine_speed_rpm);
    position_.timestamp = this->get_clock()->now();
}

void WheelOdometry::update_position(int32_t rpm, double steering_angle, rclcpp::Time current_time)
{
    rclcpp::Duration time_step = current_time - position_.timestamp;

    position_.phi + calc_angular_velocity(rpm, steering_angle) * time_step.seconds();

    position_.x + calc_linear_velocity(rpm, wheel_diameter_) * std::cos(calc_angular_velocity(rpm, steering_angle)) * time_step.seconds();

    position_.y + calc_linear_velocity(rpm, wheel_diameter_) * std::sin(calc_angular_velocity(rpm, steering_angle)) * time_step.seconds();
}

// double WheelOdometry::calc_phi(int32_t rpm, double steering_angle, double time_step)
// {
//     return prev_phi_ + calc_angular_velocity(rpm, steering_angle) * time_step;
// }

// double WheelOdometry::calc_x(int32_t rpm, double steering_angle, double time_step)
// {
//     return prev_x_ + calc_linear_velocity(rpm, wheel_diameter_) * std::cos(calc_angular_velocity(rpm, steering_angle)) * time_step;
// }

// double WheelOdometry::calc_y(int32_t rpm, double steering_angle, double time_step)
// {
//     return prev_y_ + calc_linear_velocity(rpm, wheel_diameter_) * std::sin(calc_angular_velocity(rpm, steering_angle)) * time_step;
// }

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
