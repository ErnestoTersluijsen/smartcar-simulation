#ifndef SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER_HPP
#define SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER_HPP

#include "smartcar_msgs/msg/status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStatePublisher : public rclcpp::Node
{
  public:
    JointStatePublisher();

  private:
    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr subscription_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    void vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg);

    void update_wheel_position(int32_t engine_speed, double steering_angle);

    sensor_msgs::msg::JointState msg_;
};

#endif // SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER_HPP
