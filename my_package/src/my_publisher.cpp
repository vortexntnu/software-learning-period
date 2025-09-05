#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Make a node
  auto node = rclcpp::Node::make_shared("my_publisher_node");

  // Create a timer: every 1 second call this lambda
  auto timer = node->create_wall_timer(
    1s,
    [node]() {
      RCLCPP_INFO(node->get_logger(), "alive");
    });

  // Spin keeps the node running so the timer can fire
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
