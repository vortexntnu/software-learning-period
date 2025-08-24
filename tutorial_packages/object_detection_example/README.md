# Object Detection Example

This package is meant to familiarize the user with various relevant concepts and datatypes for the perception/object detection workflow.
Things covered in this example:
/bull 
Bounding Boxes
Camera Matrices
Stereo cameras
OpenCV
PointCloud library


# Prerequisites 
The rosbag that goes along with this example contains the relevant ros2 topic needed to complete this tutorial. Verify the contents of the rosbag by displaying the color image. 

The vision-msgs package required for the Detection2DArray (bounding boxes) is not part of the base ros2 installation and therefore has to be installed manually.

        sudo apt install ros-<ros2_distro>-vision-msgs


Once everything is set up we are ready to start the tutorial.

# Task 1.1 Repuslish the color image
Ros2 using the ros2 [Image](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html) definition for its messages. To process these images we first convert them to the OpenCV image object Cv::Mat. This is done using cv_bridge. If we need to publish the processed image we have to use cv_bridge to convert it from the OpenCV format back the ros2 format before publishing. First create a ros2 node that subscribes to the color image, converts it to the OpenCv format and then converts it back to the ros2 format for republishing on a different topic. Once you are able to verify that this works correctly you can move on to the next task.

# Task 1.2 Visualizing the bounding boxes

For this example the bounding boxes was generated from the predicitons of a yolov8 object detection model. The bounding boxes are represented my the [Detection2DArray](https://docs.ros.org/en/kinetic/api/vision_msgs/html/msg/Detection2DArray.html) message. By examninig the message definition we see that the detection2darray uses the [BoundingBox2D](https://docs.ros.org/en/kinetic/api/vision_msgs/html/msg/BoundingBox2D.html) to represent bounding boxes. The relevant fields of this message are the pixel coordinates(represented as float to allow sub-pixel presicion)
center.x
center.y
size_x
size_y

This gives us the necessary information to draw and visualize the bounding boxes on the color image. We have provided the function "draw_bounding_boxes" in "src/bounding_box_utils.cpp". Check the types.hpp file under the include directory for relevant helper structs. It is good practice to convert ros types top non-ros types before processing. Try to draw the bounding boxes on the color image and publish the image as a ros2 image message. You might notice that the boxes do not appear to be in the right position. This is expected and is what leads us to the next task.

# Task 1.3 Transforming the Bounding Boxes

The reason the initial bounding boxes might look incorrect is due to how the image is processed before being passed through the yolo object detection model. The yolo model expects images of size 640x640. Is it not necessarily the fact that the images from the camera will have these dimensions. Therefore some resizing has to be done. To ensure stable predictions from the object detection model is it also important that the objects are not stretched or distorted during this resizing process. Therefore we use a techinique called letterbox padding that resizes the images, while perseerving the aspect ratio.

## insert side-by-side image of letterboxed vs non-letterboxed color image here


the rosbag using in this example does not have access to this letterboxed image, only the original. Therefore we should transform the bounding boxes. Since we know the model operates on 640x640 images we only need to get the original image dimensions. These dimensions are included in the [Image](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html) message, but it is also common to retrieve them from the corresponding [camera info](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) (scroll down to the bottom for a compact message definition). By subscriping to this topic and storing the relevant information in persistant member variables, and by using the prior knowledgde of the letterbox image dimensions we have all we need to transform the bounding boxes. You are free to use the functions provided in 'src/bounding_box_utils.cpp" for this task. 


# Task 2

The rosbag also includes a depth image produced by a zed2 stereo camera. In this example pixels in the depth image is represented by float32 and each pixel value is the distance along the z-dimension(camera direction) for that pixel.

**Stereo Camera**

![Stereo camera](./img/stereo_camera.png)

Stereo cameras work by capturing two images of the same scene from slightly different viewpoints, similar to how human eyes perceive depth. By comparing these images, the system can calculate the distance to objects, enabling depth perception and 3D reconstruction of the environment. How pixels are matched between the images and how the depth image is calculated is not a topic of this tutorial.

The depth image produced by the stereo camera is overlapped with the left color image (the one present in this bag) so that fusing data from the two images is a straightforward process. The images are also syncronized which is an additional benefit. 

**Camera Matrix** 

Since depth images can have varying resolutions and fov we need some information to be able to map depth pixels to 3d points. For this we require a model of the camera, which is represented by a camera matrix. It is also worth noting that the frame convention for optical frames in ros2 is x-right, y-down and z-forward.

![Camera frame](./img/zed_frame_axis.png)

The "optical" frame is the one relevant for this example. The x-forward, y-left and z-up is also commonly used to define the physical frame of the camera (not optical), although maritime sensor conventions are set with z-down, so its important to be aware of this to avoid future confusion.

If you are new to the concept of camera matrices I recommend checkout out this informative video https://www.youtube.com/watch?v=Hz8kz5aeQ44

The camera matrix for the color image is received on the camera_info topic. Since the color image and the depth image are overlapping, is is in this case valid to use the camera matrix for the color image on the depth image.

We have provided a util function to convert depth pixels to 3d point using the camera matrix. When working with 3d data of large quantities is it normal to use the Poincloud library, which provided datastructures and algorithms for working with pointclouds.

Try to use the bounding boxes and the depth image to extract the 3d points for all pixels withing the bounding box and publish this as a pointcloud. As with images, ros2 also uses its own ros2 [pointcloud message](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html). To convert between these formats we use the pcl_conversions package.