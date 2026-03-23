#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("my_subscriber_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "my_topic",
            10,
            std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscriber node started!");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}