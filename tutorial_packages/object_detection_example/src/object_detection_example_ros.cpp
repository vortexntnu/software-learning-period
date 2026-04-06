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
    std::string camera_info_topic =
        this->declare_parameter<std::string>("camera_info_sub_topic");
    std::string pointcloud_pub_topic =
        this->declare_parameter<std::string>("pointcloud_pub_topic");

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

    // Camera info is only needed once to get the intrinsics, then the subscriber shuts itself down.
    color_image_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&ObjectDetectionExampleNode::color_image_info_callback, this, _1));

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_pub_topic, 10);

    // Task 2: pre-compute letterbox properties once so the callback doesn't repeat it every frame.
    ImageDimensions original;
    original.height = this->declare_parameter<int>("original_image_height", 720);
    original.width  = this->declare_parameter<int>("original_image_width", 1280);
    ImageDimensions letterboxed;
    letterboxed.height = this->declare_parameter<int>("letterboxed_image_height", 640);
    letterboxed.width  = this->declare_parameter<int>("letterboxed_image_width", 640);
    letterbox_props_ = calculate_letterbox_properties(original, letterboxed);
}

void ObjectDetectionExampleNode::synchronized_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
    const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    if (!color_image_info_received_) {
        return;
    }

    // Task 2: convert depth image to cv::Mat so we can read individual pixel values.
    // The ZED2 depth image is float32 — each pixel is distance in metres along the z-axis.
    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_image);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat& depth_mat = depth_ptr->image;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (const auto& det : detections->detections) {
        // Convert the ROS detection into our BoundingBox struct, then undo the
        // letterbox transform (Task 1.3) to get pixel coordinates in the original image.
        BoundingBox b;
        b.center_x = det.bbox.center.position.x;
        b.center_y = det.bbox.center.position.y;
        b.size_x   = det.bbox.size_x;
        b.size_y   = det.bbox.size_y;
        BoundingBox box = transform_bounding_box(b, letterbox_props_);

        // Clamp the bounding box to stay within the image bounds.
        int u_min = std::max(0, (int)(box.center_x - box.size_x / 2.0));
        int u_max = std::min(depth_mat.cols - 1, (int)(box.center_x + box.size_x / 2.0));
        int v_min = std::max(0, (int)(box.center_y - box.size_y / 2.0));
        int v_max = std::min(depth_mat.rows - 1, (int)(box.center_y + box.size_y / 2.0));

        // Task 2: for every pixel inside the bounding box, read its depth value and
        // project it into 3D space using the camera intrinsics.
        for (int v = v_min; v <= v_max; v++) {
            for (int u = u_min; u <= u_max; u++) {
                float depth = depth_mat.at<float>(v, u);

                pcl::PointXYZ point;
                project_pixel_to_point(
                    u, v, depth,
                    camera_intrinsics_.fx, camera_intrinsics_.fy,
                    camera_intrinsics_.cx, camera_intrinsics_.cy,
                    point);

                // project_pixel_to_point sets NaN for invalid depth — skip those.
                if (!std::isnan(point.x)) {
                    cloud.push_back(point);
                }
            }
        }
    }

    // Task 2: convert the PCL cloud to a ROS PointCloud2 message and publish.
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(cloud, output_msg);
    output_msg.header = color_image->header;
    pointcloud_pub_->publish(output_msg);
}

void ObjectDetectionExampleNode::color_image_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg) {
    if (!color_image_info_received_) {

        // Store the relevant fields of the camera_info_msg in persistent member variables here.
        // Remember to declare the other member variables used in this function in the header file.

        color_image_frame_id_ = camera_info_msg->header.frame_id;

        // Task 2: the K matrix is the 3×3 camera intrinsics matrix stored row-major.
        // K = [ fx  0  cx ]
        //     [  0 fy  cy ]
        //     [  0  0   1 ]
        camera_intrinsics_.fx = camera_info_msg->k[0];
        camera_intrinsics_.fy = camera_info_msg->k[4];
        camera_intrinsics_.cx = camera_info_msg->k[2];
        camera_intrinsics_.cy = camera_info_msg->k[5];

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