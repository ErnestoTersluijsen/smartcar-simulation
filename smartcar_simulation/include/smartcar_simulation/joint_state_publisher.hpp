#ifndef SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER_HPP
#define SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER_HPP

#include "smartcar_msgs/msg/status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

/**
 * @brief Joint state publisher for the smartcar's different joints.
 *
 */
class JointStatePublisher : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new Joint State Publisher object
     *
     */
    JointStatePublisher();

  private:
    /**
     * @brief Callback function for when a status message is received.
     *
     * @param status_msg Status message content
     */
    void vehicle_status_callback(const smartcar_msgs::msg::Status::SharedPtr status_msg);

    /**
     * @brief Updates the wheel position based on the status message content.
     *
     * @param engine_speed Engine speed of the smartcar.
     * @param steering_angle Steering angle of the smartcar.
     */
    void update_wheel_position(int32_t engine_speed, double steering_angle);

    /**
     * @brief Subscription for the smartcar status messages.
     *
     */
    rclcpp::Subscription<smartcar_msgs::msg::Status>::SharedPtr subscription_;

    /**
     * @brief Publisher for the joint states.
     *
     */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    /**
     * @brief Message containing the states of the joints.
     *
     */
    sensor_msgs::msg::JointState msg_;
};

#endif // SMARTCAR_SIMULATION_JOINT_STATE_PUBLISHER_HPP
