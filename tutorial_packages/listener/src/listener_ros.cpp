#include "listener/listener_ros.hpp"

Listener::Listener() : Node("listener")
{
    pose_listener_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "talker_pose", 10, std::bind(&Listener::pose_cb, this, std::placeholders::_1));
}

void Listener::pose_cb(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
        "Received pose: position(x=%.2f, y=%.2f, z=%.2f), orientation(x=%.2f, y=%.2f, z=%.2f, w=%.2f)",
        msg->position.x, msg->position.y, msg->position.z,
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
