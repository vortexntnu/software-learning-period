#include "ashish_starter/vehicle_node_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VehicleNode>());

    rclcpp::shutdown();

    return 0;
}
