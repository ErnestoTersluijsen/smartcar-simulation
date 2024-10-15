#include <smartcar_simulation/joint_state_publisher.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();

    return 0;
}
