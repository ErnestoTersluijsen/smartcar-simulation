#ifndef SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
#define SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP

#include "smartcar_msgs/msg/status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

struct Position
{
    double phi = 0;
    double x = 0;
    double y = 0;
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

    void update_position(int32_t rpm, double steering_angle, double delta_time);


    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr subscription_;

    // rclcpp::Publisher<>::SharedPtr publisher_;

    Position position_;

    double wheelbase_;

    double wheel_diameter_;
};

#endif // SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
