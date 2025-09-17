#include "object_detection_example/utils.hpp"

namespace object_detection_example {

LetterboxProperties calculate_letterbox_properties(const ImageDimensions& original_dims, const ImageDimensions& letterboxed_dims) {
    LetterboxProperties props;

    int org_image_width = original_dims.width;
    int org_image_height = original_dims.height;

    props.scale_factor = std::min(
        static_cast<double>(letterboxed_dims.width) / org_image_width,
        static_cast<double>(letterboxed_dims.height) / org_image_height);

    double resized_width = org_image_width * props.scale_factor;
    double resized_height = org_image_height * props.scale_factor;

    props.pad_x = (letterboxed_dims.width - resized_width) / 2.0;
    props.pad_y = (letterboxed_dims.height - resized_height) / 2.0;

    return props;
}

BoundingBox transform_bounding_box(const BoundingBox& bbox, const LetterboxProperties& props) {
    BoundingBox transformed_bbox = bbox;

    transformed_bbox.center_x = (bbox.center_x - props.pad_x) / props.scale_factor;
    transformed_bbox.center_y = (bbox.center_y - props.pad_y) / props.scale_factor;
    transformed_bbox.size_x /= props.scale_factor;
    transformed_bbox.size_y /= props.scale_factor;

    return transformed_bbox;
}

void draw_bounding_boxes(cv::Mat& image, const std::vector<BoundingBox>& boxes) {
    for (const auto& box : boxes) {
        cv::Point top_left(box.center_x - box.size_x / 2.0, box.center_y - box.size_y / 2.0);
        cv::Point bottom_right(box.center_x + box.size_x / 2.0, box.center_y + box.size_y / 2.0);

        cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
    }
}

void project_pixel_to_point(int u, int v, float depth, double fx, double fy, double cx, double cy, pcl::PointXYZ& point) {
    if (depth <= 0 || depth == std::numeric_limits<float>::infinity() || std::isnan(depth)) {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    point.x = (u - cx) * depth / fx;
    point.y = (v - cy) * depth / fy;
    point.z = depth;
}

} // namespace object_detection_example