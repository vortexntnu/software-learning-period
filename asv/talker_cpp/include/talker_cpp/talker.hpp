#ifndef TALKER_H
#define TALKER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/Odometry.h>

class Talker {
public:
    Talker(ros::NodeHandle nh);

    void spin();

private:
    int seq;
    int frequency = 2;

    ros::Publisher seq_pub;
    ros::Publisher random_odom_pub;

    ros::Subscriber odom_reply_sub;


    void odom_reply_cb(const nav_msgs::Odometry &odom_msg);
};

#endif // TALKER_H