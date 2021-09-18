#ifndef TALKER_H
#define TALKER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include <array>

class Talker {
public:
    Talker(ros::NodeHandle nh);

    void spin();

private:
    int seq;
    int frequency = 2;

    std::array<double, 3> pos = {5, 1, 0};
    std::array<double, 4> q = {0, 0, 0, 1};

    ros::Publisher seq_pub;
    ros::Publisher random_pose_pub;

    ros::Subscriber pose_reply_sub;


    void odom_reply_cb(const geometry_msgs::Pose &odom_msg);
};

#endif // TALKER_H