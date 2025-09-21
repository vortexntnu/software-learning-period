#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HonestSubscriber : public rclcpp::Node {
public:
  HonestSubscriber() : Node("honest_subscriber") {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HonestSubscriber>());
  rclcpp::shutdown();
  return 0;
}

