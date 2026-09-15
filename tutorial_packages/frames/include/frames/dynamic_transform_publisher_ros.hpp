#ifndef DYNAMIC_TRANSFORM_PUBLISHER_ROS_HPP
#define DYNAMIC_TRANSFORM_PUBLISHER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


class DynamicTransformPublisher : public rclcpp::Node
{
public:
  DynamicTransformPublisher();

private:
  void timer_callback();

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};
#endif // DYNAMIC_TRANSFORM_PUBLISHER_ROS_HPP
