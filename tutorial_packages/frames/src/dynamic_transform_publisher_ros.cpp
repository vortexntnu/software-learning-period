#include "frames/dynamic_transform_publisher_ros.hpp"
#include <rclcpp/rclcpp.hpp>

DynamicTransformPublisher::DynamicTransformPublisher()
: Node("dynamic_transform_publisher")
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  start_time_ = this->get_clock()->now();
  
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&DynamicTransformPublisher::timer_callback, this));
}

void DynamicTransformPublisher::timer_callback()
{
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform;

    // Calculate the time elapsed since the node started
    double time_since_start = (this->get_clock()->now() - start_time_).seconds();

    // The tutorial uses theta for rotation calculation in later steps
    double theta = time_since_start / 2.0;

    // The timestamp for the transform is the current time
    transform.header.stamp = this->get_clock()->now();

    // TODO: populate the transform.header.frame_id and transform.child_frame_id according to spec

    // TODO: populate the transform.transform.translation. fields (x, y, z)

    // TODO: construct a quaternion object and set the rotation values of the quaternion

    // TODO: populate the transform.transform.rotation fields (x, y, z, w) with the quaternion from last step

    // TODO: publish transform
    // tf_broadcaster_->sendTransform(transform);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicTransformPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
