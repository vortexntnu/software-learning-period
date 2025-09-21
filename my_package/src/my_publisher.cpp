#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher() : Node("my_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MyPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello from my C++ publisher!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}
