# yolo_segmentation
Real-time instance segmentation using YOLOv8/v11. This node subscribes to an image topic, runs segmentation inference, and publishes bounding boxes, segmentation masks, and optional debug visualization.

## Features
- Instance segmentation with configurable confidence thresholds
- Publishes both bounding boxes and binary segmentation masks
- Optional debug visualization output
- Configurable publishers (enable/disable bbox, mask, or debug output independently)
- Support for CPU and GPU inference
- Parameterized through YAML configuration file

## Run
```bash
ros2 launch yolo_segmentation yolo_segmentation.launch.py
```

## Topics
### Subscribed
- `/gripper_camera/image_raw` (configurable) -> `sensor_msgs/Image`
  - Input camera image for segmentation

### Published
- `/pipeline/camera/bboxes` (configurable) -> `vision_msgs/Detection2DArray`
  - Bounding box detections with class IDs and confidence scores
  - Only published if `pub_bbox: True`

- `/pipeline/camera/segmentation_mask` (configurable) -> `sensor_msgs/Image` (mono8)
  - Binary segmentation mask (255 for detected instances, 0 for background)
  - Merged mask of all detected instances
  - Only published if `pub_mask: True`

- `/pipeline/camera/segmentation_debug` (configurable) -> `sensor_msgs/Image`
  - Debug visualization with segmentation overlays and bounding boxes
  - Only published if `pub_debug: True`

## Parameters
All parameters are configured in `config/yolo_segmentation_params.yaml`:

### Node Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/gripper_camera/image_raw` | Topic to subscribe to for input images |
| `output_bbox_topic` | string | `/pipeline/camera/bboxes` | Topic to publish bounding box detections |
| `output_mask_topic` | string | `/pipeline/camera/segmentation_mask` | Topic to publish segmentation masks |
| `output_debug_topic` | string | `/pipeline/camera/segmentation_debug` | Topic to publish debug visualization |
| `pub_bbox` | bool | `true` | Enable/disable bounding box publishing |
| `pub_mask` | bool | `true` | Enable/disable segmentation mask publishing |
| `pub_debug` | bool | `false` | Enable/disable debug visualization publishing |

### Model Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | `/ros2_ws/yolo_segmentation_training/results/custom_yolov89/weights/last.pt` | Path to trained YOLO model weights (.pt file) |
| `device` | string | `cpu` | Inference device: `cpu`, `cuda`, or `cuda:0` for GPU |
| `imgsz` | int | `640` | Input image size for model inference |
| `confidence_threshold` | float | `0.3` | Minimum confidence threshold (0.0-1.0) |
| `max_detections` | int | `1` | Maximum number of detections per image |
| `compile` | bool | `false` | Enable PyTorch model compilation for faster inference |

## Model Training
For training custom segmentation models, see the `yolo_segmentation_training` package.
