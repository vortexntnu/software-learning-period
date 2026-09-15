#include "talker/talker_ros.hpp"


Talker::Talker() : Node("talker_node") {
    seq = 0;
    pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("talker_pose", 10);
    seq_pub = this->create_publisher<std_msgs::msg::Int64>("talker_seq", 10);
}

void Talker::spin() {
    rclcpp::Rate loop_rate(frequency);
    while (rclcpp::ok()) {

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pos[0];
        pose_msg.position.y = pos[1];
        pose_msg.position.z = pos[2];
        pose_msg.orientation.x = q[0];
        pose_msg.orientation.y = q[1];
        pose_msg.orientation.z = q[2];
        pose_msg.orientation.w = q[3];
        pose_pub->publish(pose_msg);

        std_msgs::msg::Int64 seq_msg;
        seq_msg.data = seq;
        seq_pub->publish(seq_msg);
        seq++;

        rclcpp::spin_some(shared_from_this());
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Talker>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}