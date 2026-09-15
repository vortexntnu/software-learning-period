#ifndef SIMPLE_PUBLISHER_ROS_HPP
#define SIMPLE_PUBLISHER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher();

private:
    void timer_callback();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string publish_value_;
};

#endif // SIMPLE_PUBLISHER_ROS_HPP
