#include "transit_starter/signal_node_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SignalNode>());

    rclcpp::shutdown();

    return 0;
}
