#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <mutex>
#include <string>

namespace object_detection_example {

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using DetsMsg  = vision_msgs::msg::Detection2DArray;

    void image_callback(const ImageMsg::SharedPtr msg);
    void detection_callback(const DetsMsg::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depth_image_callback(const ImageMsg::SharedPtr msg);
    void process_and_publish();

    rclcpp::Publisher<ImageMsg>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<ImageMsg>::SharedPtr image_sub_;
    rclcpp::Subscription<ImageMsg>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<DetsMsg>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    std::mutex data_mutex_;
    ImageMsg::ConstSharedPtr last_image_;
    ImageMsg::ConstSharedPtr last_depth_image_;
    DetsMsg::ConstSharedPtr last_detections_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
    uint32_t image_width_ = 0;
    uint32_t image_height_ = 0;

};

} // namespace object_detection_example
