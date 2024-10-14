#include "smartcar_simulation/joint_state_publisher.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

JointStatePublisher::JointStatePublisher() : Node("joint_state_publisher")
{

}