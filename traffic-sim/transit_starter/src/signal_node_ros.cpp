#include "transit_starter/signal_node_ros.hpp"

#include <chrono>
#include <stdexcept>

SignalNode::SignalNode() : Node("signal_node") {
    signal_pub_ = this->create_publisher<transit_msgs::msg::SignalState>(
        "/signal_state", 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / tick_hz_),
        std::bind(&SignalNode::tick, this));
}

void SignalNode::tick() {
    // TODO (Task 3): advance elapsed_ by 1.0 / tick_hz_, use it as your
    //     clock, and cycle RED -> GREEN -> YELLOW -> RED for lane_id_
    //     against green_seconds_ / yellow_seconds_ / all_red_seconds_.
    //     Publish one SignalState per tick with the state you land on —
    //     transit_msgs::msg::SignalState::RED / YELLOW / GREEN.
    //
    // TODO (Task 4): once a single lane cycles correctly, drive all four
    //     approaches (1, 2, 3, 4) from here instead of just lane_id_, with
    //     opposing pairs sharing a phase: 1 & 2 green together, then 3 & 4
    //     green together, never all four green at once. You'll publish
    //     four SignalState messages per tick (one per lane) instead of one.
    throw std::runtime_error("fill in Task 3 above");
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalNode>());
    rclcpp::shutdown();
    return 0;
}
