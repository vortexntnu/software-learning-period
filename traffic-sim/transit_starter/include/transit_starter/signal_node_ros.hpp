#ifndef TRANSIT_STARTER__SIGNAL_NODE_ROS_HPP_
#define TRANSIT_STARTER__SIGNAL_NODE_ROS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <transit_msgs/msg/signal_state.hpp>

#include <string>

/**
 * @brief The traffic light node. Fill in the TODOs in signal_node_ros.cpp.
 *
 * Same idea as VehicleNode: you do not need to change anything in this
 * file. Every function you have to write is already declared here and
 * already called from somewhere, so your job is only to fill in what goes
 * inside them.
 *
 * The tasks are in ../../CURRICULUM.md. The TODO numbers here, in the .cpp
 * file and in the curriculum all match.
 */
class SignalNode : public rclcpp::Node {
public:
    SignalNode();

private:
    /** @brief Reads config/transit_params.yaml. Written for you. */
    void set_parameters();

    /** @brief TODO (Task 3): create the publisher and the timer. */
    void set_publisher();

    /** @brief Runs tick_hz_ times a second. Written for you. */
    void tick();

    /** @brief TODO (Task 3, then Task 4): publish the lights for this moment. */
    void publish_lights();

    /** @brief TODO (Task 3, then Task 4): which colour a lane shows right now. */
    uint8_t state_for_lane(uint16_t lane);

    // ---- Values from config/transit_params.yaml. Do not edit them here. ----

    std::string signal_topic_;   /**< topic we publish SignalState on */

    uint16_t lane_id_{};         /**< the lane we light up first, in Task 3 */
    std::string signal_id_;      /**< our name, unique in the whole city */

    double green_seconds_{};     /**< how long a lane stays green */
    double yellow_seconds_{};    /**< how long it then stays yellow */
    double all_red_seconds_{};   /**< pause with everything red, between phases */
    double tick_hz_{};           /**< how many times a second tick() runs */

    // ---- Values the node changes while it runs. ----

    double elapsed_ = 0.0;       /**< seconds since the node started */

    // ---- The ROS objects you create in set_publisher(). ----

    rclcpp::Publisher<transit_msgs::msg::SignalState>::SharedPtr signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TRANSIT_STARTER__SIGNAL_NODE_ROS_HPP_
