#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
using namespace std::chrono_literals;

class HonestPublisher : public rclcpp::Node {
public:
  HonestPublisher() : Node("honest_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(500ms, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "Hello from honest_publisher!";
      publisher_->publish(msg);
    });
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HonestPublisher>());
  rclcpp::shutdown();
  return 0;
}

