#include "smartcar_simulation/wheel_odometry.hpp"

#include "smartcar_msgs/msg/status.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

WheelOdometry::WheelOdometry()
    : Node("wheel_odometry"), wheelbase_(0.257), wheel_diameter_(0.064), pose_covariance_({1.0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 100000.0, 0, 0, 0, 0, 0, 0, 100000.0, 0, 0, 0, 0, 0, 0, 100000.0, 0, 0, 0, 0, 0, 0, 0.5}),
      twist_covariance_({0.1, 0, 0, 0, 0, 0, 0, 100000.0, 0, 0, 0, 0, 0, 0, 100000.0, 0, 0, 0, 0, 0, 0, 100000.0, 0, 0, 0, 0, 0, 0, 100000.0, 0, 0, 0, 0, 0, 0, 0.1})
{
    subscription_ = this->create_subscription<smartcar_msgs::msg::Status>("/smartcar/vehicle_status", 10, std::bind(&WheelOdometry::vehicle_status_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/smartcar/wheel/odom", 10);
}

void WheelOdometry::vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg)
{
    update_position(status_msg->engine_speed_rpm, status_msg->steering_angle_rad, rclcpp::Time(this->get_clock()->now(), RCL_SYSTEM_TIME));
}

void WheelOdometry::update_position(int32_t rpm, double steering_angle, rclcpp::Time current_time)
{
    if (position_.timestamp.nanoseconds() == 0)
    {
        position_.timestamp = rclcpp::Time(this->get_clock()->now(), RCL_SYSTEM_TIME);
        return;
    }

    rclcpp::Duration time_step = current_time - position_.timestamp;
    position_.timestamp = current_time;

    double linear_velocity = calc_linear_velocity(rpm, steering_angle);
    double angular_velocity = calc_angular_velocity(rpm, steering_angle);

    double phi = position_.phi + angular_velocity * time_step.seconds();
    double x = position_.x + linear_velocity * std::cos(phi) * time_step.seconds();
    double y = position_.y + linear_velocity * std::sin(phi) * time_step.seconds();

    position_.phi = phi;
    position_.x = x;
    position_.y = y;

    nav_msgs::msg::Odometry message;

    message.header.stamp = get_clock()->now();
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";

    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, phi);
    message.pose.pose.orientation = tf2::toMsg(q);

    message.pose.covariance = pose_covariance_;

    message.twist.twist.linear.x = linear_velocity;
    message.twist.twist.linear.y = 0.0;
    message.twist.twist.linear.z = 0.0;

    message.twist.twist.angular.z = angular_velocity;

    message.twist.covariance = twist_covariance_;

    publisher_->publish(message);
}

double WheelOdometry::calc_angular_velocity(int32_t rpm, double steering_angle)
{
    return (calc_linear_velocity(rpm, wheel_diameter_) / wheelbase_) / std::tan(steering_angle);
}

double WheelOdometry::calc_linear_velocity(int32_t rpm, double wheel_diameter)
{
    return (rpm * M_PI * wheel_diameter) / 60;
}
