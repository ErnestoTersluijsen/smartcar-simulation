#ifndef SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
#define SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP

#include "smartcar_msgs/msg/status.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Struct that contains the previous values
 *
 */
struct Position
{
    double phi;
    double x;
    double y;
    rclcpp::Time timestamp;
};

/**
 * @brief Odometry class that uses smartcar status messages to calculate the new position.
 *
 */
class WheelOdometry : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new Wheel Odometry object.
     *
     */
    WheelOdometry();

  private:
    /**
     * @brief Callback function when receiving a smartcar status messages.
     *
     * @param status_msg Smartcar status messages
     */
    void vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg);

    /**
     * @brief Calulates the angular velocity.
     *
     * @param rpm RPM of the smartcar.
     * @param steering_angle Steering angle of the smartcar.
     * @return double Angular velocity of the smartcar.
     */
    double calc_angular_velocity(int32_t rpm, double steering_angle);

    /**
     * @brief Calulates the linear velocity.
     *
     * @param rpm RPM of the smartcar.
     * @param steering_angle Steering angle of the smartcar.
     * @return double Linear velocity of the smartcar.
     */
    double calc_linear_velocity(int32_t rpm, double wheel_diameter);

    /**
     * @brief Updates the position of the smartcar by publishing on the /smartcar/wheel/odom topic.
     *
     * @param rpm RPM of the smartcar.
     * @param steering_angle Steering angle of the smartcar.
     * @param current_time Current simulation time.
     */
    void update_position(int32_t rpm, double steering_angle, rclcpp::Time current_time);

    /**
     * @brief Subscriber for the smartcar status messages.
     *
     */
    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr subscription_;

    /**
     * @brief Publisher for the odometry messages.
     *
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    /**
     * @brief Struct for the previous variables for angular velocity, linear velocity and previous timestamp.
     *
     */
    Position position_;

    /**
     * @brief Length of the smartcar.
     *
     */
    double wheelbase_;

    /**
     * @brief Diameter of the wheels.
     *
     */
    double wheel_diameter_;

    /**
     * @brief Covariance matrix for the pose.
     *
     */
    std::array<double, 36> pose_covariance_;

    /**
     * @brief Covariance matrix for the twist.
     *
     */
    std::array<double, 36> twist_covariance_;
};

#endif // SMARTCAR_SIMULATION_WHEEL_ODOMETER_HPP
