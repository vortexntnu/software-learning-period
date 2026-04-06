#ifndef IMAGE_REPUBLISHER_NODE_HPP
#define IMAGE_REPUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

#include "object_detection_example/types.hpp"

class ImageRepublisherNode : public rclcpp::Node {
public:
    explicit ImageRepublisherNode(const rclcpp::NodeOptions& options);

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void detections_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;

    // Protects latest_detections_ from concurrent access between image_callback
    // and detections_callback, which can fire on different executor threads.
    std::mutex detections_mutex_;
    std::shared_ptr<vision_msgs::msg::Detection2DArray> latest_detections_;

    // Pre-computed letterbox scale + padding (calculated once in the constructor).
    object_detection_example::LetterboxProperties props_;
};

#endif // IMAGE_REPUBLISHER_NODE_HPP
