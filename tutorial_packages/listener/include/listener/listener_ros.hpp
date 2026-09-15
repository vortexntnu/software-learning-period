#ifndef LISTENER_HPP
#define LISTENER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Listener : public rclcpp::Node
{
public:
    Listener();

private:
    void pose_cb(const geometry_msgs::msg::Pose::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_listener_;
};

#endif // LISTENER_HPP
