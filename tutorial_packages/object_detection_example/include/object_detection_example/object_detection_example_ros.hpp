#ifndef OBJECT_DETECTION_EXAMPLE_ROS_HPP
#define OBJECT_DETECTION_EXAMPLE_ROS_HPP

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

namespace object_detection_example {

class ObjectDetectionExampleNode : public rclcpp::Node {
public:
    explicit ObjectDetectionExampleNode(const rclcpp::NodeOptions& options);
    virtual ~ObjectDetectionExampleNode() = default;

private:
    void setup_sync();

    /**
     * @brief Callback function for synchronized depth image, color image, and
     * 2D detections.
     *
     * This function is triggered when synchronized messages for a depth image,
     * a color image, and a 2D detection array are received.
     */
     void synchronized_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
        const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);

    void color_image_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_image_info_sub_;
    bool color_image_info_received_ = false;
    std::string color_image_frame_id_;

    // Depth image sync policies uses ExactTime with the assumption
    // that the depth image and color image are from the same camera
    // and are synchronized.

    // Policy for Depth Image + Color Image + Detections
    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray>
        SyncPolicy_DI_CI_D;

    message_filters::Subscriber<sensor_msgs::msg::Image> color_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detections_sub_;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy_DI_CI_D>> sync_di_ci_d_;

};

} // namespace object_detection_example

#endif // OBJECT_DETECTION_EXAMPLE_ROS_HPP