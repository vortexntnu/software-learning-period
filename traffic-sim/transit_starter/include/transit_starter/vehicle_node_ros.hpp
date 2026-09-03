#ifndef TRANSIT_STARTER_VEHICLE_NODE_ROS_HPP
#define TRANSIT_STARTER_VEHICLE_NODE_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <transit_msgs/msg/signal_state.hpp>
#include <transit_msgs/msg/vehicle_state.hpp>

#include <string>

/**
 * @brief Skeleton vehicle node — fill in the TODOs in vehicle_node_ros.cpp.
 *
 * See ../../CURRICULUM.md for the task list this class is built around;
 * TODOs there and in the .cpp file are numbered to match.
 */
class VehicleNode : public rclcpp::Node {
public:
    VehicleNode();

private:
    /** @brief Called at tick_hz_. Builds and publishes this vehicle's state. */
    void tick();

    /** @brief TODO (Task 5): records the state of the light on lane_id_. */
    void on_signal(const transit_msgs::msg::SignalState::SharedPtr msg);

    // TODO (Task 1): pick a unique vehicle_id and the lane you want to
    // drive. Lane numbers are on the map in Foxglove/RViz2, and in
    // transit_sim/README.md's "The map" table.
    std::string vehicle_id_ = "car_yourname";
    uint16_t lane_id_ = 1;
    std::string color_ = "red"; /** full list in transit_sim/transit_sim/colors.py */

    // TODO (Task 2): how fast your car moves along its lane (m/s), and how
    // long the lane is (m). There's no C++ accessor for the sim's map — it's
    // a Python-internal detail transit_sim owns — so measure the lane's
    // length yourself by publishing with `ros2 topic pub` while sweeping
    // progress, per transit_sim/README.md's "Testing without writing a
    // node" section, and hardcode what you find here.
    double speed_ = 8.0;
    double lane_length_ = 80.0;

    double tick_hz_ = 20.0;
    double progress_ = 0.0;
    int8_t light_state_ = -1; /** TODO (Task 5): set by on_signal() below */

    rclcpp::Publisher<transit_msgs::msg::VehicleState>::SharedPtr vehicle_pub_;
    rclcpp::Subscription<transit_msgs::msg::SignalState>::SharedPtr signal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // TRANSIT_STARTER_VEHICLE_NODE_ROS_HPP
