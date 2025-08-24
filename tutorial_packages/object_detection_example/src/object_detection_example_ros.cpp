#include "object_detection_example/object_detection_example_ros.hpp"

namespace object_detection_example {

using namespace std::placeholders;

ObjectDetectionExampleNode::ObjectDetectionExampleNode(const rclcpp::NodeOptions& options)
: Node("object_detection_example_node", options) {
    setup_sync();
}

void ObjectDetectionExampleNode::setup_sync() {
    // It is good practice to declare topic names in the config file.
    std::string color_image_sub_topic =
        this->declare_parameter<std::string>("color_image_sub_topic");
    std::string depth_image_sub_topic =
        this->declare_parameter<std::string>("depth_image_sub_topic");
    std::string detections_sub_topic =
        this->declare_parameter<std::string>("detections_sub_topic");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    color_image_sub_.subscribe(this, color_image_sub_topic,
                                   qos.get_rmw_qos_profile());
    depth_image_sub_.subscribe(this, depth_image_sub_topic,
                                qos.get_rmw_qos_profile());
    detections_sub_.subscribe(this, detections_sub_topic,
                                qos.get_rmw_qos_profile());
    sync_di_ci_d_ = std::make_shared<message_filters::Synchronizer<SyncPolicy_DI_CI_D>>(
        SyncPolicy_DI_CI_D(10), depth_image_sub_, color_image_sub_, detections_sub_);
    sync_di_ci_d_->registerCallback(
        std::bind(&ObjectDetectionExampleNode::synchronized_callback, this, _1, _2, _3));
}

void ObjectDetectionExampleNode::synchronized_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
    const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    if (!color_image_info_received_) {
        return; // We need the camera info to process the images, so we return if we haven't received it yet.
    }

}

void ObjectDetectionExampleNode::color_image_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg) {
    if (!color_image_info_received_) {

        // Store the relevent fields of the camera_info_msg in persistent member variables here.
        // Remember to declare the other member variables used in this function in the header file.

        color_image_frame_id_ = camera_info_msg->header.frame_id;
        color_image_info_received_ = true;
        color_image_info_sub_.reset(); // This shuts down the subscriber so we don't keep receiving camera info messages.
    }
}

} // namespace object_detection_example

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<object_detection_example::ObjectDetectionExampleNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}