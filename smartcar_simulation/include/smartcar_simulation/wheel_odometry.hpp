#ifndef SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
#define SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP

#include "smartcar_msgs/msg/status.hpp"

#include <rclcpp/rclcpp.hpp>

class WheelOdometry : public rclcpp::Node
{
  public:
    WheelOdometry();

  private:
    void vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg);

    double calc_angular_velocity(int32_t rpm, double steering_angle);
    
    double calc_linear_velocity(int32_t rpm, double wheel_diameter);
    
    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr subscription_;

    double wheelbase_;
    
    double wheel_diameter_;
};

#endif // SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
