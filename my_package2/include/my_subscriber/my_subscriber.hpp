#ifndef MY_SUBSCRIBER_HPP
#define MY_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber();

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif
