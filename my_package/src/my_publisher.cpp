#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("my_publisher_node");
  auto pub  = node->create_publisher<std_msgs::msg::String>("chatter", 10);

  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
    500ms,  // or std::chrono::milliseconds(500)
    [pub]() {
      std_msgs::msg::String msg;
      msg.data = "hello";
      pub->publish(msg);
    }
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
