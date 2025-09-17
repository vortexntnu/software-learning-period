#ifndef UTILS_HPP
#define UTILS_HPP

#include "types.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace object_detection_example {

LetterboxProperties calculate_letterbox_properties(const ImageDimensions& original_dims, const ImageDimensions& letterboxed_dims);
BoundingBox transform_bounding_box(const BoundingBox& bbox, const LetterboxProperties& props);
void draw_bounding_boxes(cv::Mat& image, const std::vector<BoundingBox>& boxes);
void project_pixel_to_point(int u, int v, float depth, double fx, double fy, double cx, double cy, pcl::PointXYZ& point);

} // namespace object_detection_example

#endif // UTILS_HPP