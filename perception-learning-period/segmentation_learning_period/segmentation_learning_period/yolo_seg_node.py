"""ROS2 node for YOLO segmentation: subscribes to images, runs segmentation, and publishes results."""

import os
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from ultralytics.engine.results import Results
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from .yolo_seg import YoloSegmentation


class YoloSegmentationNode(Node):
    """ROS2 node for running YOLO segmentation and publishing results."""

    def __init__(self) -> None:
        super().__init__("yolo_segmentation_node")

        self._load_parameters()
        self._segmentation = self._load_model()

        self.bridge = CvBridge()
        self._original_camera_info: Optional[CameraInfo] = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image, self.input_topic, self.on_image, qos_profile
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.input_camera_info_topic,
            self.camera_info_callback,
            qos_profile,
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, self.output_camera_info_topic, qos_profile
        )

        if self.pub_debug:
            self._debug_publisher = self.create_publisher(
                Image, self.output_debug_topic, qos_profile
            )
        if self.pub_bbox:
            self._bbox_publisher = self.create_publisher(
                Detection2DArray, self.output_bbox_topic, qos_profile
            )
        if self.pub_mask:
            self._mask_publisher = self.create_publisher(
                Image, self.output_mask_topic, qos_profile
            )

        self.get_logger().info(f"Node initialized. Subscribing to '{self.input_topic}'")

    def _load_parameters(self) -> None:
        params = {
            "model_path": Parameter.Type.STRING,
            "device": Parameter.Type.STRING,
            "confidence_threshold": Parameter.Type.DOUBLE,
            "iou": Parameter.Type.DOUBLE,
            "imgsz": Parameter.Type.INTEGER,
            "compile": Parameter.Type.BOOL,
            "verbose": Parameter.Type.BOOL,
            "input_topic": Parameter.Type.STRING,
            "output_bbox_topic": Parameter.Type.STRING,
            "output_mask_topic": Parameter.Type.STRING,
            "output_debug_topic": Parameter.Type.STRING,
            "pub_bbox": Parameter.Type.BOOL,
            "pub_mask": Parameter.Type.BOOL,
            "pub_debug": Parameter.Type.BOOL,
            "input_camera_info_topic": Parameter.Type.STRING,
            "output_camera_info_topic": Parameter.Type.STRING,
        }
        for name, ptype in params.items():
            self.declare_parameter(name, ptype)
            setattr(self, name, self.get_parameter(name).value)

    def _load_model(self) -> YoloSegmentation:
        mp = os.path.expanduser(self.model_path)
        if not os.path.isabs(mp):
            share = get_package_share_directory("yolo_segmentation")
            mp = os.path.join(share, "model", mp)
        if not os.path.isfile(mp):
            self.get_logger().error(f"Model not found: {mp}")
            raise FileNotFoundError(mp)
        self.get_logger().info(f"Loading YOLO model from: {mp}")
        return YoloSegmentation(
            model_path=mp,
            device=self.device,
            confidence_threshold=self.confidence_threshold,
            iou=self.iou,
            imgsz=self.imgsz,
            compile=self.compile,
            verbose=self.verbose,
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self._original_camera_info = msg

    def on_image(self, msg: Image) -> None:
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results: list[Results] = self._segmentation.predict(cv_image)
        result: Results = results[0]

        if self.pub_bbox:
            self.publish_bboxes_and_confidences(result, msg.header)
        if self.pub_mask:
            self.publish_masks(result, msg.header)
        if self.pub_debug:
            self.publish_debug_image(result, msg.header)

    def publish_bboxes_and_confidences(self, result: Results, header: Header) -> None:
        det_array = Detection2DArray()
        det_array.header = header
        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        clss = result.boxes.cls.cpu().numpy()
        for (x1, y1, x2, y2), conf, cls_id in zip(boxes, confs, clss):
            det = Detection2D()
            det.header = header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(cls_id))
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center.position.x = float((x1 + x2) / 2.0)
            bbox.center.position.y = float((y1 + y2) / 2.0)
            bbox.center.theta = 0.0
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)
            det.bbox = bbox
            det_array.detections.append(det)

        self._bbox_publisher.publish(det_array)

    def publish_masks(self, result: Results, header: Header) -> None:
        if result.masks is None:
            return

        masks: np.ndarray = result.masks.data.cpu().numpy()
        if masks.shape[0] == 0:
            return

        binary_masks = masks > 0.0
        combined = np.any(binary_masks, axis=0).astype("uint8") * 255

        mask_msg: Image = self.bridge.cv2_to_imgmsg(combined, encoding="mono8")
        mask_msg.header = header
        self._mask_publisher.publish(mask_msg)

        self.publish_camera_info(mask_msg, header)

    def publish_camera_info(self, mask_msg: Image, header: Header) -> None:
        """Publish scaled camera info matching the mask resolution.

        This correctly handles YOLO's letterbox padding by:
        1. Computing the scale factor used by YOLO (based on imgsz)
        2. Computing the dimensions after scaling but before padding
        3. Computing the padding offset (letterbox grey bars)
        4. Applying scale to focal lengths and scale+offset to principal points
        """
        if self._original_camera_info is None:
            self.get_logger().warn(
                "Original camera_info not yet received. Skipping camera_info publishing.",
                throttle_duration_sec=5.0,
            )
            return

        mask_width = mask_msg.width
        mask_height = mask_msg.height
        orig_width = self._original_camera_info.width
        orig_height = self._original_camera_info.height

        if mask_width == 0 or mask_height == 0:
            self.get_logger().warn(
                f"Invalid mask resolution: {mask_width}x{mask_height}. Skipping camera_info publishing."
            )
            return

        scale = min(
            float(self.imgsz) / float(orig_width),
            float(self.imgsz) / float(orig_height),
        )

        scaled_width = orig_width * scale
        scaled_height = orig_height * scale

        pad_width = round((mask_width - scaled_width) / 2)
        pad_height = round((mask_height - scaled_height) / 2)

        scaled_camera_info = CameraInfo()
        scaled_camera_info.header = header
        scaled_camera_info.width = mask_width
        scaled_camera_info.height = mask_height

        scaled_camera_info.k = list(self._original_camera_info.k)
        scaled_camera_info.k[0] = self._original_camera_info.k[0] * scale
        scaled_camera_info.k[2] = (self._original_camera_info.k[2] * scale) + pad_width
        scaled_camera_info.k[4] = self._original_camera_info.k[4] * scale
        scaled_camera_info.k[5] = (self._original_camera_info.k[5] * scale) + pad_height

        scaled_camera_info.d = list(self._original_camera_info.d)
        scaled_camera_info.distortion_model = (
            self._original_camera_info.distortion_model
        )
        scaled_camera_info.r = list(self._original_camera_info.r)

        scaled_camera_info.p = list(self._original_camera_info.p)
        if len(scaled_camera_info.p) >= 12:
            scaled_camera_info.p[0] = self._original_camera_info.p[0] * scale
            scaled_camera_info.p[2] = (
                self._original_camera_info.p[2] * scale
            ) + pad_width
            scaled_camera_info.p[5] = self._original_camera_info.p[5] * scale
            scaled_camera_info.p[6] = (
                self._original_camera_info.p[6] * scale
            ) + pad_height

        self._camera_info_publisher.publish(scaled_camera_info)

    def publish_debug_image(self, result: Results, header: Header) -> None:
        debug_img: np.ndarray = self._segmentation.visualize(result)
        debug_msg: Image = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
        debug_msg.header = header
        self._debug_publisher.publish(debug_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
