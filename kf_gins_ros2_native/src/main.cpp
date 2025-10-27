#include <rclcpp/rclcpp.hpp>
#include "kf_gins_ros2_native/node_factory.hpp"
int main(int argc, char** argv){ rclcpp::init(argc, argv); auto node = kf_gins_ros2_native::make_node(); rclcpp::spin(node); rclcpp::shutdown(); return 0; }
