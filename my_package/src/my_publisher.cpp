#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = rclcpp::Node::make_shared("my_publisher_node");

  // Create a publisher on topic "chatter"
  auto publisher = node->create_publisher<std_msgs::msg::String>("chatter", 10);

  // Create a timer that publishes "alive" every second
  auto timer = node->create_wall_timer(
    1s,
    [publisher]() {
      auto msg = std_msgs::msg::String();
      msg.data = "alive";
      publisher->publish(msg);
    });

  // Spin the node so the timer runs
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
