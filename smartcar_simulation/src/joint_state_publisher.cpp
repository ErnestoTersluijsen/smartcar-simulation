#include "smartcar_simulation/joint_state_publisher.hpp"

#include "smartcar_msgs/msg/status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

JointStatePublisher::JointStatePublisher() : Node("joint_state_publisher")
{
    subscription_ = this->create_subscription<smartcar_msgs::msg::Status>("/smartcar/vehicle_status", 10, std::bind(&JointStatePublisher::vehicle_status_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    msg_.header.stamp = get_clock()->now();
    msg_.name = {"front_left_wheel_steer_joint", "front_left_wheel_joint", "front_right_wheel_steer_joint", "front_right_wheel_joint", "back_left_wheel_joint", "back_right_wheel_joint"};
    msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void JointStatePublisher::vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg)
{
    update_wheel_position(status_msg->engine_speed_rpm, status_msg->steering_angle_rad);

    publisher_->publish(msg_);
}

void JointStatePublisher::update_wheel_position(int32_t engine_speed, double steering_angle)
{
    rclcpp::Time current_time = get_clock()->now();
    rclcpp::Duration duration = static_cast<rclcpp::Time>(msg_.header.stamp) - current_time;
    msg_.header.stamp = current_time;

    double radian_diff = (engine_speed / 60) * (M_PI * 2) * (static_cast<double>(duration.nanoseconds()) / 1e-9);

    for (size_t i = 0; i < msg_.position.size(); ++i)
    {
        if (i == 0 || i == 2)
        {
            msg_.position.at(i) = steering_angle;
        }
        else
        {
            msg_.position.at(i) = std::fmod(msg_.position.at(i) + radian_diff, M_PI * 2);
        }
    }
}
