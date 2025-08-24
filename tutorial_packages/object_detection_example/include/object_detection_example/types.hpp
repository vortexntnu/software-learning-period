#ifndef TYPES_HPP
#define TYPES_HPP

namespace object_detection_example {

struct BoundingBox {
    double center_x;
    double center_y;
    double size_x;
    double size_y;
};

struct LetterboxProperties {
    double scale_factor;
    double pad_x;
    double pad_y;
};

struct CameraIntrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
};

struct ImageDimensions {
    int width;
    int height;
};

struct ImageProperties {
    CameraIntrinsics intr;
    ImageDimensions dim;
};

} // namespace object_detection_example

#endif // TYPES_HPP