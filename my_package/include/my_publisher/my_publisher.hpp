#ifndef MY_PUBLISHER_HPP
#define MY_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <string>

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher();

private:
    void timer_callback();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string publish_value_;
    int i;
};

#endif // MY_PUBLISHER_HPP
