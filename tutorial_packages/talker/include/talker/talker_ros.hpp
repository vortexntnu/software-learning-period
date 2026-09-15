#ifndef TALKER_H
#define TALKER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <array>

/**
 * @brief This is a "talker" class, which is a term coined by
 * the ROS tutorial for something that periodically publishes
 * messages without any other inputs.
*/
class Talker : public rclcpp::Node {
public:
    /**
     * @brief The class constructor
     *
     * In ROS2, you initialize the node by calling the base class constructor with the node name.
     * The constructor is normally where you create publishers and subscribers.
     * In this case, we create two publishers: one for geometry_msgs::msg::Pose
     * and one for std_msgs::msg::Int64.
     * The first publisher publishes a pose message on the "pose_cpp" topic,
     * and the second publisher publishes a sequence number on the "seq_cpp" topic.
     * The sequence number is incremented with each loop iteration in the spin() method.
     * 
    */
    Talker();

    /**
     * @brief Performs the node tasks, spins once, then sleeps
     * for a set amount of time.
     *
     * @note This is a blocking call on rclcpp::ok()
    */
    void spin();

private:
    int seq;            /** Incremented by one for every loop in spin() */
    int frequency = 2;  /** The frequency of the loop in spin()*/

    std::array<double, 3> pos = {5, 1, 0};  /** The position of the published pose */
    std::array<double, 4> q = {0, 0, 0, 1}; /** The orientation of the published pose */

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr seq_pub;  /** Publisher for seq */
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub; /** Publisher for pose */
};

#endif // TALKER_H