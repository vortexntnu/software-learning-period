#include "my_publisher/my_publisher.hpp"

using namespace std::chrono_literals;

MyPublisher::MyPublisher() : Node("my_publisher")
{
    // Fixed topic and value
    std::string topic = "/chatter";
    publish_value_ = "Andreas er goat";
    i = 0;

    // QoS: Keep only the last message, best-effort reliability
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic, qos_profile);

    // Create timer to call callback every second
    timer_ = this->create_wall_timer(1s, std::bind(&MyPublisher::timer_callback, this));
}

void MyPublisher::timer_callback()
{
    i = i + 1;
    auto message = std_msgs::msg::String();
    message.data = publish_value_;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s' '%d'", message.data.c_str(), i);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}
