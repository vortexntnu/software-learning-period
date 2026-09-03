#ifndef TRANSIT_STARTER_SIGNAL_NODE_ROS_HPP
#define TRANSIT_STARTER_SIGNAL_NODE_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <transit_msgs/msg/signal_state.hpp>

#include <string>

/**
 * @brief Skeleton traffic-light node — fill in the TODOs in
 * signal_node_ros.cpp.
 *
 * See ../../CURRICULUM.md for the task list this class is built around;
 * TODOs there and in the .cpp file are numbered to match.
 */
class SignalNode : public rclcpp::Node {
public:
    SignalNode();

private:
    /** @brief Called at tick_hz_. Decides and publishes this light's state. */
    void tick();

    // TODO (Task 3): which lane's approach are you lighting first? The map
    // has one junction with four approaches: lanes 1, 2 (north/south) and
    // 3, 4 (east/west).
    uint16_t lane_id_ = 1;
    std::string signal_id_ = "junction1_lane1";

    // TODO (Task 3): how long each phase lasts, in seconds. Tune these
    // however you like once the cycle itself is correct.
    double green_seconds_ = 9.0;
    double yellow_seconds_ = 2.0;
    double all_red_seconds_ = 1.0;

    double tick_hz_ = 5.0;
    double elapsed_ = 0.0;

    rclcpp::Publisher<transit_msgs::msg::SignalState>::SharedPtr signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // TRANSIT_STARTER_SIGNAL_NODE_ROS_HPP
