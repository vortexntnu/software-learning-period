# Perception Learning Period: YOLO Segmentation and OpenCV

In this learning period, you will use a segmentation model that you have already trained and connect it to ROS 2. You will then use OpenCV to find the outline of the segmented object, approximate the outline with line segments, draw it on the original image, and publish the result as a new ROS 2 image topic.

The finished pipeline will look like this:

```text
rosbag image topic
        |
        v
yolo_segmentation_node
        |
        v
segmentation mask
        |
        v
your OpenCV contour node <--- original image
        |
        v
image with a line-segment outline
```

This guide starts after the dataset has been labelled and the model has been trained. The autonomy/state-machine learning period is not part of this exercise.

## Prerequisites

This guide assumes that you:

- use Ubuntu 22.04 and ROS 2 Humble;
- have already completed the basic software learning period;
- have a trained YOLO segmentation model as a `.pt` file;
- have a rosbag containing camera images;
- already have Foxglove and `foxglove_bridge` installed.

The example workspace is `~/ros2_ws`. If your workspace has a different name, change the commands accordingly.

---

## Task 1: Set up the segmentation pipeline

### Task 1.1: build the repository

Install the ROS dependencies used in this learning period:

```bash
sudo apt update
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-vision-msgs
```

Install the Python packages needed by the YOLO node:

```bash
python3 -m pip install ultralytics torch numpy
```

Build and source the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Remember to source both ROS 2 and the workspace in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Task 1.2: Inspect the rosbag

First, inspect the contents of the bag without playing it:

```bash
ros2 bag info <PATH_TO_BAG>
```

Find the topic containing the camera images. It should have the message type:

```text
sensor_msgs/msg/Image
```

Write down the topic name. This guide refers to it as `<IMAGE_TOPIC>`.

Play the bag in a loop:

```bash
ros2 bag play <PATH_TO_BAG> --loop
```

In another terminal, check that the image topic appears:

```bash
ros2 topic list
```

Questions:

1. What is the image topic called?
2. What can you see in the recording?
3. Which object should your model segment?

### Task 1.3: Open the rosbag in Foxglove

Set up Foxglove now so that you can see the result after every later task. Keep the rosbag playing and open a new terminal for `foxglove_bridge`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Foxglove connects to ROS 2 through the WebSocket created by `foxglove_bridge`.

Open Foxglove and:

1. Select **Open connection**.
2. Select **Foxglove WebSocket**.
3. Enter `ws://localhost:8765`.
4. Add an **Image** panel.
5. Select `<IMAGE_TOPIC>`.

You should now see the camera recording from the rosbag. Leave Foxglove open during the rest of the learning period. As you create new output topics, add another Image panel for each one.

If the image topic does not appear, check that both the bag and bridge are still running:

```bash
ros2 topic list
ros2 node list
```

If Foxglove is running on another computer, `localhost` refers to that other computer, not the ROS computer. In that case, connect to `ws://<ROS_COMPUTER_IP>:8765` and ensure the network and firewall allow the connection.

### Task 1.4: Add your trained model

Create a model directory inside the segmentation package and copy your trained model into it:

```bash
mkdir -p ~/ros2_ws/src/vortex-deep-learning-pipelines/ros/yolo_segmentation/model
cp <PATH_TO_YOUR_MODEL>/best.pt \
  ~/ros2_ws/src/vortex-deep-learning-pipelines/ros/yolo_segmentation/model/best.pt
```

Open:

```text
/perception-learning-period/segmentation_learning_period/config/segmentation_learning_period_params.yaml
```

Change at least these parameters:

```yaml
yolo_segmentation_node:
  ros__parameters:
    input_topic: "<IMAGE_TOPIC>"
    output_mask_topic: "/perception/segmentation_mask"
    output_debug_topic: "/perception/segmentation_debug"

    pub_bbox: false
    pub_mask: true
    pub_debug: true

    model_path: "best.pt"
    device: "cuda"  # Use "cpu" if CUDA is unavailable
    imgsz: 640
    confidence_threshold: 0.3
    iou: 0.7
    compile: false
    verbose: false
```

The node also subscribes to the configured camera-info topic. If your bag has a `sensor_msgs/msg/CameraInfo` topic, set `input_camera_info_topic` to it. The segmentation mask itself still works without camera info, but the node will warn that it cannot publish scaled camera information.

Build again so the model and configuration are installed:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select yolo_segmentation
source install/setup.bash
```

### Task 1.5: Run the YOLO segmentation node

Keep the rosbag playing. In another terminal, run:

```bash
ros2 launch yolo_segmentation yolo_segmentation.launch.py
```

Return to Foxglove and add Image panels for `/perception/segmentation_mask` and `/perception/segmentation_debug`. You can now compare the model output with the original image while the bag plays.

The mask topic uses `sensor_msgs/msg/Image` with `mono8` encoding:

- `0` means background;
- `255` means that at least one object was segmented.

The current YOLO node combines all detected instances and classes into one binary mask.

Questions:

1. Can you see the object in the segmentation mask?
2. What colour is the detected object in the mask? What colour is the background?
3. Does the mask follow the object while the recording plays?

---

## Task 2: Create an OpenCV contour package

You will now create a C++ node that subscribes to both the original image and the segmentation mask.

Create the package:

```bash
cd ~/ros2_ws/src
ros2 pkg create contour_overlay \
  --build-type ament_cmake \
  --dependencies rclcpp sensor_msgs cv_bridge message_filters
```

Your package should eventually look like this:

```text
contour_overlay/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── contour_overlay.launch.py
└── src/
    └── contour_overlay_node.cpp
```

### Task 2.1: Subscribe to two image topics

Create `src/contour_overlay_node.cpp`.

The node needs two inputs:

- the original image on `<IMAGE_TOPIC>`;
- the binary mask on `/perception/segmentation_mask`.

The mask is created from a particular input frame and receives the same timestamp as that frame. Use `message_filters` with an approximate-time synchronizer so that the correct mask is paired with the correct original image.

Useful headers and types:

```cpp
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
```

Inside your node, create two filtered subscribers and a synchronizer:

```cpp
using Image = sensor_msgs::msg::Image;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

message_filters::Subscriber<Image> image_subscriber_;
message_filters::Subscriber<Image> mask_subscriber_;
std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
```

Register a callback that accepts both messages:

```cpp
void image_callback(
  const Image::ConstSharedPtr & image_msg,
  const Image::ConstSharedPtr & mask_msg);
```

Use a best-effort sensor-data QoS profile for the subscriptions. This matches the YOLO node and is commonly used for camera streams. When using `message_filters::Subscriber`, pass the underlying ROS QoS profile through `rmw_qos_profile_t`.

Hint:

```cpp
const auto qos = rclcpp::SensorDataQoS();

image_subscriber_.subscribe(this, image_topic, qos.get_rmw_qos_profile());
mask_subscriber_.subscribe(this, mask_topic, qos.get_rmw_qos_profile());
```

Create a publisher for the completed overlay:

```cpp
overlay_publisher_ = create_publisher<Image>(
  "/perception/contour_overlay", rclcpp::SensorDataQoS());
```

### Task 2.2: Convert ROS images to OpenCV images

Use `cv_bridge` in the synchronized callback:

```cpp
const cv::Mat original = cv_bridge::toCvShare(image_msg, "bgr8")->image;
const cv::Mat mask = cv_bridge::toCvShare(mask_msg, "mono8")->image;
```

Do not draw directly into `original`. Make a copy:

```cpp
cv::Mat overlay = original.clone();
```

The YOLO node may publish a mask at a different resolution from the original image because the model resizes and letterboxes its input. Before extracting and drawing contours, resize the mask to the original image size:

```cpp
cv::Mat resized_mask;
cv::resize(mask, resized_mask, original.size(), 0.0, 0.0, cv::INTER_NEAREST);
```

Use `cv::INTER_NEAREST` for a binary mask. Other interpolation methods create intermediate grey values that do not represent either class.

> Note: resizing is sufficient for this exercise and for the current node output. For geometry that must be pixel-accurate, the model's complete scale-and-padding transform should be inverted instead of treating the letterboxed mask as a normally resized image.

### Task 2.3: Clean the mask

Even though the input should be binary, apply a threshold so the contour operation receives a clean image:

```cpp
cv::Mat binary_mask;
cv::threshold(resized_mask, binary_mask, 127, 255, cv::THRESH_BINARY);
```

Optional challenge: use the following OpenCV operations to remove isolated noise or close small holes:

```cpp
cv::morphologyEx()
cv::MORPH_OPEN
cv::MORPH_CLOSE
cv::getStructuringElement()
```

Questions:

1. Does the cleaned mask contain less noise than the original mask?
2. What visible change occurs when you use a larger kernel?

---

## Task 3: Convert the mask to line segments

### Task 3.1: Find the contours

Use OpenCV to extract the outside boundary of every segmented region:

```cpp
std::vector<std::vector<cv::Point>> contours;

cv::findContours(
  binary_mask,
  contours,
  cv::RETR_EXTERNAL,
  cv::CHAIN_APPROX_SIMPLE);
```

`cv::RETR_EXTERNAL` keeps only outer contours. `cv::CHAIN_APPROX_SIMPLE` removes many redundant points along straight sections of the boundary.

Small false detections can create unwanted contours. Filter them using `cv::contourArea()`:

```cpp
const double minimum_area = 200.0;

for (const auto & contour : contours) {
  if (cv::contourArea(contour) < minimum_area) {
    continue;
  }

  // Approximate and draw this contour here.
}
```

Make `minimum_area` a ROS parameter so it can be adjusted without recompiling the node.

### Task 3.2: Approximate contours with straight lines

A contour can contain hundreds of points. Use `cv::approxPolyDP()` to replace it with a smaller polygon:

```cpp
std::vector<cv::Point> polygon;
const double epsilon = 0.01 * cv::arcLength(contour, true);

cv::approxPolyDP(contour, polygon, epsilon, true);
```

The points in `polygon` define the line segments:

```text
(P0, P1), (P1, P2), ..., (Pn-1, P0)
```

Make the epsilon factor a ROS parameter. Try values such as `0.001`, `0.01`, `0.03`, and `0.1`.

Questions:

1. Does a larger epsilon make the outline simpler or more detailed?
2. Which tested value gives the best-looking outline in Foxglove?

### Task 3.3: Draw and publish the polygon

Draw each accepted polygon on the copied original image:

```cpp
cv::polylines(
  overlay,
  polygon,
  true,
  cv::Scalar(0, 255, 0),
  3,
  cv::LINE_AA);
```

Convert the result back to a ROS image and preserve the original header:

```cpp
auto output_msg = cv_bridge::CvImage(
  image_msg->header, "bgr8", overlay).toImageMsg();

overlay_publisher_->publish(*output_msg);
```

Your completed callback should perform these steps in order:

1. Convert the two ROS images using `cv_bridge`.
2. Resize the mask to the original image size.
3. Threshold and optionally clean the mask.
4. Find external contours.
5. Reject contours below the minimum area.
6. Approximate accepted contours with polygons.
7. Draw the polygons on a copy of the original frame.
8. Publish the result with the original header.

Add exception handling for `cv_bridge::Exception` so one malformed image does not terminate the node.

---

## Task 4: Build and run your OpenCV node

### Task 4.1: Update `CMakeLists.txt`

Add OpenCV and the executable to `contour_overlay/CMakeLists.txt`:

```cmake
find_package(OpenCV REQUIRED)

add_executable(contour_overlay_node src/contour_overlay_node.cpp)

ament_target_dependencies(contour_overlay_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)

target_link_libraries(contour_overlay_node ${OpenCV_LIBRARIES})

target_include_directories(contour_overlay_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS
  contour_overlay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

Ensure that the file still ends with:

```cmake
ament_package()
```

### Task 4.2: Check `package.xml`

The package-creation command should add the ROS dependencies. Add OpenCV if it is not already listed:

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>message_filters</depend>
```

OpenCV is found directly by CMake with `find_package(OpenCV REQUIRED)`; `cv_bridge` supplies the corresponding ROS dependency.

### Task 4.3: Build and run

Build only your package:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select contour_overlay
source install/setup.bash
```

Run it while both the bag and YOLO node are running:

```bash
ros2 run contour_overlay contour_overlay_node --ros-args \
  -p image_topic:=<IMAGE_TOPIC> \
  -p mask_topic:=/perception/segmentation_mask \
  -p output_topic:=/perception/contour_overlay \
  -p minimum_area:=200.0 \
  -p epsilon_factor:=0.01
```

If you hard-coded the topics while first testing the node, replace them with declared ROS parameters before continuing.

Check the result in Foxglove. Add an Image panel for `/perception/contour_overlay` and compare it with the original image and segmentation mask. The green outline should follow the edge of the segmented object while the bag plays.

---

## Task 5: Create one launch file

Create `contour_overlay/launch/contour_overlay.launch.py` that starts:

1. `yolo_segmentation_node` from the `yolo_segmentation` package;
2. your `contour_overlay_node`;
3. `foxglove_bridge` on port `8765`.

The rosbag should still be started separately. Recorded data is input to the pipeline, while the launch file starts the software that processes it.

Your final startup procedure should be:

```bash
# Terminal 1
ros2 bag play <PATH_TO_BAG> --loop

# Terminal 2
ros2 launch contour_overlay contour_overlay.launch.py
```

Open Foxglove and connect to:

```text
ws://localhost:8765
```

---

## Final deliverable

And now you should have a working segmentation model!
