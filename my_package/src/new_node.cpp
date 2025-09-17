#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("new_node");
  RCLCPP_INFO(node->get_logger(), "Hello from new_node!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}