#include "object_detection_example/image_republisher_node.hpp"
#include "object_detection_example/utils.hpp"
#include "object_detection_example/types.hpp"
#include <vision_msgs/msg/detection2_d_array.hpp>

ImageRepublisherNode::ImageRepublisherNode(const rclcpp::NodeOptions& options)
: Node("image_republisher_node", options)
{
    std::string sub_topic        = this->declare_parameter<std::string>("color_image_sub_topic", "/zed_node/left/image_rect_color");
    std::string pub_topic        = this->declare_parameter<std::string>("color_image_pub_topic", "/image_with_boxes");
    std::string detections_topic = this->declare_parameter<std::string>("detections_sub_topic", "/detections_output");

    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // Task 1.1
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        sub_topic, qos,
        std::bind(&ImageRepublisherNode::image_callback, this, std::placeholders::_1)
    );
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(pub_topic, qos);

    // Task 1.2
    detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        detections_topic, qos,
        std::bind(&ImageRepublisherNode::detections_callback, this, std::placeholders::_1)
    );

    // Task 1.3
    object_detection_example::ImageDimensions original;
    original.height = this->declare_parameter<int>("original_image_height", 720);
    original.width  = this->declare_parameter<int>("original_image_width", 1280);

    object_detection_example::ImageDimensions letterboxed;
    letterboxed.height = this->declare_parameter<int>("letterboxed_image_height", 640);
    letterboxed.width  = this->declare_parameter<int>("letterboxed_image_width", 640);

    this->props_ = object_detection_example::calculate_letterbox_properties(original, letterboxed);
}

void ImageRepublisherNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    // Task 1.1: sensor_msgs/Image -> cv::Mat
    cv_bridge::CvImagePtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat& annotated = cv_image_ptr->image;

    // Task 1.2: draw bounding boxes onto the image
    std::vector<object_detection_example::BoundingBox> boxes;
    {
        std::lock_guard<std::mutex> lock(this->detections_mutex_);
        if (this->latest_detections_) {
            for (const auto& det : this->latest_detections_->detections) {
                object_detection_example::BoundingBox b;
                b.center_x = det.bbox.center.position.x;
                b.center_y = det.bbox.center.position.y;
                b.size_x   = det.bbox.size_x;
                b.size_y   = det.bbox.size_y;

                // Task 1.3: transform from letterbox coordinates to original image coordinates
                boxes.push_back(object_detection_example::transform_bounding_box(b, props_));
            }
        }
    }

    if (!boxes.empty()) {
        object_detection_example::draw_bounding_boxes(annotated, boxes);
    }

    // Task 1.1: cv::Mat -> sensor_msgs/Image and publish
    cv_image_ptr->header = msg->header;
    pub_->publish(*cv_image_ptr->toImageMsg());
}

void ImageRepublisherNode::detections_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
{
    // Task 1.2: store latest detections for use in image_callback
    std::lock_guard<std::mutex> lock(this->detections_mutex_);
    latest_detections_ = std::make_shared<vision_msgs::msg::Detection2DArray>(*msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageRepublisherNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
