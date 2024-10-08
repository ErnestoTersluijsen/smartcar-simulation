#ifndef SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
#define SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP

#include "smartcar_msgs/msg/status.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

struct Position
{
    double phi;
    double x;
    double y;
    rclcpp::Time timestamp;
};

class WheelOdometry : public rclcpp::Node
{
  public:
    WheelOdometry();

  private:
    void vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg);

    double calc_angular_velocity(int32_t rpm, double steering_angle);

    double calc_linear_velocity(int32_t rpm, double wheel_diameter);

    void update_position(int32_t rpm, double steering_angle, rclcpp::Time current_time);

    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr subscription_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    Position position_;

    double wheelbase_;

    double wheel_diameter_;

    std::array<double, 36> pose_covariance_;

    std::array<double, 36> twist_covariance_;
};

#endif // SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
