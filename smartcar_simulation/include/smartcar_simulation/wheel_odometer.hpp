#ifndef SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
#define SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP

#include "smartcar_msgs/msg/status.hpp"

#include <rclcpp/rclcpp.hpp>

class WheelOdometer : public rclcpp::Node
{
  public:
    WheelOdometer();

  private:
    void vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg);

    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr subscription_;
};

#endif // SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
