#ifndef IMAGE_REPUBLISHER_NODE_HPP
#define IMAGE_REPUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "object_detection_example/types.hpp"
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <mutex>

class ImageRepublisherNode : public rclcpp::Node {
public:
    explicit ImageRepublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ImageRepublisherNode() = default;

private:
    object_detection_example::LetterboxProperties props_;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void detections_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
    std::mutex detections_mutex_;
    vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_detections_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

#endif // IMAGE_REPUBLISHER_NODE_HPP
