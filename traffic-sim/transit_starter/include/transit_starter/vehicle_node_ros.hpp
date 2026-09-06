#ifndef TRANSIT_STARTER__VEHICLE_NODE_ROS_HPP_
#define TRANSIT_STARTER__VEHICLE_NODE_ROS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <transit_msgs/msg/signal_state.hpp>
#include <transit_msgs/msg/vehicle_state.hpp>

#include <string>

/**
 * @brief The vehicle node. Fill in the TODOs in vehicle_node_ros.cpp.
 *
 * You do not need to change anything in this file. Every function you have
 * to write is already declared here and already called from somewhere, so
 * your job is only to fill in what goes inside them.
 *
 * The tasks are in ../../CURRICULUM.md. The TODO numbers here, in the .cpp
 * file and in the curriculum all match.
 */
class VehicleNode : public rclcpp::Node {
public:
    VehicleNode();

private:
    /** @brief Reads config/transit_params.yaml. Written for you. */
    void set_parameters();

    /** @brief TODO (Task 1 and Task 5): create the publisher, timer and subscription. */
    void set_subscribers_and_publisher();

    /** @brief Runs tick_hz_ times a second. Written for you. */
    void tick();

    /** @brief TODO (Task 1): build one VehicleState and publish it. */
    void publish_state();

    /** @brief TODO (Task 2): move the car a little further along its lane. */
    void advance_progress();

    /** @brief TODO (Task 5): true when the car has to wait at the stop line. */
    bool must_stop_for_light();

    /** @brief TODO (Task 5): remember the state of the light on our lane. */
    void on_signal(const transit_msgs::msg::SignalState::SharedPtr msg);

    // ---- Values from config/transit_params.yaml. Do not edit them here. ----

    std::string vehicle_topic_;  /**< topic we publish VehicleState on */
    std::string signal_topic_;   /**< topic we listen for SignalState on */

    std::string vehicle_id_;     /**< our name, unique in the whole city */
    uint16_t lane_id_{};         /**< the lane we drive on */
    std::string color_;          /**< names listed in transit_sim/transit_sim/colors.py */

    double speed_{};             /**< how fast we drive, in metres per second */
    double lane_length_{};       /**< how long our lane is, in metres */
    double stop_progress_{};     /**< where the stop line is, between 0.0 and 1.0 */
    double tick_hz_{};           /**< how many times a second tick() runs */

    // ---- Values the node changes while it runs. ----

    double progress_ = 0.0;      /**< how far along the lane we are, 0.0 to 1.0 */
    bool moving_ = false;        /**< false parks the car on the map */
    double velocity_ = 0.0;      /**< speed we report, in metres per second */
    int light_state_ = -1;       /**< last light seen; -1 means none yet */

    // ---- The ROS objects you create in set_subscribers_and_publisher(). ----

    rclcpp::Publisher<transit_msgs::msg::VehicleState>::SharedPtr vehicle_pub_;
    rclcpp::Subscription<transit_msgs::msg::SignalState>::SharedPtr signal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TRANSIT_STARTER__VEHICLE_NODE_ROS_HPP_
