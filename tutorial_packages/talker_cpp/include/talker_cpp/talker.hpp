#ifndef TALKER_H
#define TALKER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include <array>

/**
 * @brief This is a "talker" class, which is a term coined by
 * the ROS tutorial for something that periodically publishes 
 * messages without any other inputs.
*/
class Talker {
public:
    /**
     * @brief The class constructor
     * 
     * @param nh A unique identifier for the node that this will run on
    */
    Talker(ros::NodeHandle nh);

    /**
     * @brief Performs the node tasks, spins once, then sleeps
     * for a set amount of time.
     * 
     * @note This is a blocking call on ros::ok()
    */
    void spin();

private:
    int seq;            /** Incremented by one for every loop in spin() */
    int frequency = 2;  /** The frequency of the loop in spin()*/

    std::array<double, 3> pos = {5, 1, 0};  /** The position of the published pose */
    std::array<double, 4> q = {0, 0, 0, 1}; /** The orientation of the published pose */

    ros::Publisher seq_pub;  /** Publisher for seq */
    ros::Publisher pose_pub; /** Publisher for pose */
};

#endif // TALKER_H