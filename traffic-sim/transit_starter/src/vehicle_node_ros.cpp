#include "transit_starter/vehicle_node_ros.hpp"

#include <chrono>
#include <stdexcept>

VehicleNode::VehicleNode() : Node("vehicle_node") {
    vehicle_pub_ = this->create_publisher<transit_msgs::msg::VehicleState>(
        "/vehicle_state", 10);

    // TODO (Task 5): uncomment once Tasks 1-2 work and you're ready to obey
    // the light. This subscribes to every light on the map; your job in
    // on_signal() is to ignore the ones that aren't your lane.
    // signal_sub_ = this->create_subscription<transit_msgs::msg::SignalState>(
    //     "/signal_state", 10,
    //     std::bind(&VehicleNode::on_signal, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / tick_hz_),
        std::bind(&VehicleNode::tick, this));
}

void VehicleNode::on_signal(const transit_msgs::msg::SignalState::SharedPtr msg) {
    // TODO (Task 5): record the state of the light governing your lane. A
    // SignalState with a different lane_id belongs to a different
    // approach — only act on the one that matches lane_id_.
    (void)msg;
    throw std::runtime_error(
        "fill in Task 5, or leave the subscription commented out for now");
}

void VehicleNode::tick() {
    // TODO (Task 1): construct and publish a VehicleState using
    //     vehicle_id_ / lane_id_ / progress_ / color_, with moving=false.
    //     That alone should show a parked car on the map.
    //
    // TODO (Task 2): advance progress_ each call instead of leaving it at
    //     0.0, e.g. `progress_ += speed_ * (1.0 / tick_hz_) / lane_length_;`,
    //     wrapping back to 0.0 once it passes 1.0. Set moving=true and a
    //     real velocity while it's actually moving.
    //
    // TODO (Task 5): before publishing, use light_state_ to hold
    //     moving=false (and progress_ unchanged) while your light is RED
    //     or YELLOW instead of driving straight through the junction —
    //     compare against transit_msgs::msg::SignalState::RED / YELLOW /
    //     GREEN. You'll need to know roughly where your lane's stop line
    //     sits as a `progress` fraction; find it the same way as
    //     lane_length_ above.
    throw std::runtime_error("fill in Task 1 above");
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleNode>());
    rclcpp::shutdown();
    return 0;
}
