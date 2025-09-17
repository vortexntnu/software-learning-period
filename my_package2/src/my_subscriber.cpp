#include "my_subscriber/my_subscriber.hpp"

MySubscriber::MySubscriber() : Node("my_subscriber")

{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter",
        qos_profile,
        std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1));
}

void MySubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}
