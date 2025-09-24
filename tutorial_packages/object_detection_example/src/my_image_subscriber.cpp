


#include "object_detection_example/my_image_subscriber.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "object_detection_example/types.hpp"
#include "object_detection_example/utils.hpp"
#include <mutex>

using object_detection_example::BoundingBox;
using object_detection_example::ImageDimensions;
using object_detection_example::LetterboxProperties;

namespace object_detection_example {

MySubscriber::MySubscriber() : Node("my_image_subscriber") {

    RCLCPP_INFO(this->get_logger(), "Starting MySubscriber node");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_with_boxes", rclcpp::QoS(10));
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed_node/left/image_rect_color", qos,
        std::bind(&MySubscriber::image_callback, this, std::placeholders::_1));
    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed_node/depth/depth_registered", qos,
        std::bind(&MySubscriber::depth_image_callback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/zed_node/depth/camera_info", qos,
        std::bind(&MySubscriber::camera_info_callback, this, std::placeholders::_1));

    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections_output", qos,
        std::bind(&MySubscriber::detection_callback, this, std::placeholders::_1));

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_in_boxes", rclcpp::QoS(10));
    }



void MySubscriber::image_callback(const ImageMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_image_ = msg;
    image_width_ = msg->width;
    image_height_ = msg->height;
    process_and_publish();
}

void MySubscriber::detection_callback(const DetsMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_detections_ = msg;
    process_and_publish();
}

void MySubscriber::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_camera_info_ = msg;
    process_and_publish();
}

void MySubscriber::depth_image_callback(const ImageMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_depth_image_ = msg;
    process_and_publish();
    
}


void MySubscriber::process_and_publish() {
    if (!last_image_ || !last_detections_ || !last_depth_image_ || !last_camera_info_ || image_width_ == 0 || image_height_ == 0) {
        RCLCPP_WARN(this->get_logger(), "Waiting for all data to be available...");
        return;}

    // Convert ROS images to OpenCV
    cv_bridge::CvImagePtr cv_ptr, depth_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
        depth_ptr = cv_bridge::toCvCopy(last_depth_image_, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Use utils.cpp functions for letterbox transform
    const ImageDimensions orig_dims{static_cast<int>(image_width_), static_cast<int>(image_height_)};
    const ImageDimensions letterbox_dims{640, 640};
    const LetterboxProperties letterbox_props = calculate_letterbox_properties(orig_dims, letterbox_dims);

    // Camera intrinsics from CameraInfo
    const auto& K = last_camera_info_->k;
    double fx = K[0];
    double fy = K[4];
    double cx = K[2];
    double cy = K[5];

    // Transform all detections using utils.cpp
    std::vector<BoundingBox> transformed_boxes;
    for (const auto& det : last_detections_->detections) {
        BoundingBox box{
            det.bbox.center.position.x,
            det.bbox.center.position.y,
            det.bbox.size_x,
            det.bbox.size_y
        };

        transformed_boxes.push_back(transform_bounding_box(box, letterbox_props));
    }

    // Point cloud extraction from bounding boxes
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& box : transformed_boxes) {
        int left   = std::max(0, static_cast<int>(box.center_x - box.size_x / 2.0));
        int right  = std::min(depth_ptr->image.cols - 1, static_cast<int>(box.center_x + box.size_x / 2.0));
        int top    = std::max(0, static_cast<int>(box.center_y - box.size_y / 2.0));
        int bottom = std::min(depth_ptr->image.rows - 1, static_cast<int>(box.center_y + box.size_y / 2.0));
        for (int v = top; v <= bottom; ++v) {
            for (int u = left; u <= right; ++u) {
                float depth = depth_ptr->image.at<float>(v, u);
                pcl::PointXYZ point;
                object_detection_example::project_pixel_to_point(u, v, depth, fx, fy, cx, cy, point);
                if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                    cloud->points.push_back(point);
                }
            }
        }
    }

    draw_bounding_boxes(cv_ptr->image, transformed_boxes);

    RCLCPP_INFO(this->get_logger(), "Publishing image with %zu boxes and %zu 3D points", transformed_boxes.size(), cloud->points.size());
    image_pub_->publish(*cv_ptr->toImageMsg());

    // Publish the point cloud as sensor_msgs::msg::PointCloud2
    if (!cloud->points.empty()) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header = last_image_->header;
        pointcloud_pub_->publish(cloud_msg);
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
}

} // namespace object_detection_example

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<object_detection_example::MySubscriber>());
    rclcpp::shutdown();
    return 0;
}