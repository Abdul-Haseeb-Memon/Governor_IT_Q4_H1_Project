---
sidebar_position: 4
title: 'Complete Perception Pipeline'
---

# Complete Perception Pipeline

This guide covers implementing a complete perception pipeline that integrates multiple vision components for humanoid robot perception. You'll learn how to create a unified system that processes visual input and provides comprehensive environmental understanding.

## Overview

A complete perception pipeline involves:
1. Multi-sensor data fusion (cameras, depth sensors, etc.)
2. Object detection and tracking
3. Scene understanding and semantic segmentation
4. 3D reconstruction and mapping
5. Integration with robot control systems

## Multi-Sensor Fusion Node

A node that combines data from multiple sensors:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray, Detection3DArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import json
from typing import Dict, Any, List
import threading
import queue

class MultiSensorFusionNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Initialize bridge
        self.bridge = CvBridge()

        # Publishers
        self.fused_perception_pub = self.create_publisher(String, 'fused_perception', 10)
        self.detection_3d_pub = self.create_publisher(Detection3DArray, 'detection_3d', 10)

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'camera/depth/points',
            self.pointcloud_callback,
            10
        )

        # Internal state
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_matrix = None
        self.latest_pointcloud = None
        self.fusion_lock = threading.Lock()

        # Queues for synchronized processing
        self.rgb_queue = queue.Queue(maxsize=2)
        self.depth_queue = queue.Queue(maxsize=2)

        self.get_logger().info('Multi-Sensor Fusion Node initialized')

    def rgb_callback(self, msg: Image):
        """
        Receive RGB image and queue for processing.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            try:
                self.rgb_queue.put_nowait(cv_image)
            except queue.Full:
                try:
                    self.rgb_queue.get_nowait()
                    self.rgb_queue.put_nowait(cv_image)
                except queue.Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg: Image):
        """
        Receive depth image and queue for processing.
        """
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            try:
                self.depth_queue.put_nowait(cv_depth)
            except queue.Full:
                try:
                    self.depth_queue.get_nowait()
                    self.depth_queue.put_nowait(cv_depth)
                except queue.Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """
        Receive camera calibration information.
        """
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def pointcloud_callback(self, msg: PointCloud2):
        """
        Receive point cloud data.
        """
        self.latest_pointcloud = msg

    def synchronize_and_process(self):
        """
        Synchronize RGB and depth data and process together.
        """
        # Get synchronized pair
        try:
            rgb_image = self.rgb_queue.get_nowait()
            depth_image = self.depth_queue.get_nowait()

            # Process the synchronized data
            perception_result = self.process_synchronized_data(rgb_image, depth_image)

            # Publish results
            self.publish_perception_result(perception_result)

        except queue.Empty:
            # No synchronized data available
            pass

    def process_synchronized_data(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> Dict[str, Any]:
        """
        Process synchronized RGB and depth data.
        """
        with self.fusion_lock:
            # Perform object detection on RGB
            detections_2d = self.detect_objects_2d(rgb_image)

            # Convert 2D detections to 3D using depth
            detections_3d = self.convert_2d_to_3d(detections_2d, depth_image)

            # Create comprehensive perception result
            perception_result = {
                'timestamp': self.get_clock().now().nanoseconds,
                'detections_2d': detections_2d,
                'detections_3d': detections_3d,
                'scene_analysis': self.analyze_scene(rgb_image, depth_image),
                'environment_map': self.create_environment_map(detections_3d)
            }

            return perception_result

    def detect_objects_2d(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Perform 2D object detection (placeholder implementation).
        """
        # In a real system, this would call your object detector
        # For this example, we'll return empty list
        return []

    def convert_2d_to_3d(self, detections_2d: List[Dict[str, Any]], depth_image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Convert 2D detections to 3D positions using depth information.
        """
        if self.camera_matrix is None:
            return []

        detections_3d = []

        for detection in detections_2d:
            bbox = detection['bbox']

            # Get depth at center of bounding box
            center_x = int((bbox['x1'] + bbox['x2']) / 2)
            center_y = int((bbox['y1'] + bbox['y2']) / 2)

            # Ensure coordinates are within image bounds
            center_x = max(0, min(center_x, depth_image.shape[1] - 1))
            center_y = max(0, min(center_y, depth_image.shape[0] - 1))

            depth_value = depth_image[center_y, center_x]

            if depth_value > 0:  # Valid depth
                # Convert pixel coordinates to 3D using camera matrix
                x = (center_x - self.camera_matrix[0, 2]) * depth_value / self.camera_matrix[0, 0]
                y = (center_y - self.camera_matrix[1, 2]) * depth_value / self.camera_matrix[1, 1]
                z = depth_value

                detection_3d = {
                    'class': detection['class'],
                    'confidence': detection['confidence'],
                    'position_3d': {'x': x, 'y': y, 'z': z},
                    'bbox_2d': bbox
                }

                detections_3d.append(detection_3d)

        return detections_3d

    def analyze_scene(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> Dict[str, Any]:
        """
        Analyze the scene for higher-level understanding.
        """
        scene_analysis = {
            'room_type': self.classify_room_type(rgb_image),
            'free_space': self.estimate_free_space(depth_image),
            'obstacles': self.detect_obstacles(depth_image),
            'surface_planes': self.detect_surface_planes(rgb_image, depth_image)
        }

        return scene_analysis

    def classify_room_type(self, image: np.ndarray) -> str:
        """
        Classify room type based on visual features (placeholder).
        """
        # In a real implementation, this would use image classification
        return "unknown"

    def estimate_free_space(self, depth_image: np.ndarray) -> Dict[str, float]:
        """
        Estimate free navigable space.
        """
        # Simple approach: count valid depth pixels
        valid_depths = depth_image[depth_image > 0]
        free_ratio = len(valid_depths) / (depth_image.shape[0] * depth_image.shape[1])

        return {
            'free_ratio': float(free_ratio),
            'avg_depth': float(np.mean(valid_depths)) if len(valid_depths) > 0 else 0.0
        }

    def detect_obstacles(self, depth_image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect obstacles in the environment.
        """
        # Simple threshold-based obstacle detection
        obstacle_mask = (depth_image > 0) & (depth_image < 0.5)  # Objects closer than 0.5m

        obstacles = []
        if np.any(obstacle_mask):
            # Find connected components as obstacles
            import cv2
            obstacle_contours, _ = cv2.findContours(
                obstacle_mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in obstacle_contours:
                if cv2.contourArea(contour) > 100:  # Filter small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    obstacles.append({
                        'bbox': {'x': x, 'y': y, 'width': w, 'height': h},
                        'area': cv2.contourArea(contour)
                    })

        return obstacles

    def detect_surface_planes(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect surface planes (tables, floors, walls) using depth information.
        """
        # Placeholder implementation
        # In a real system, this would use plane detection algorithms
        return []

    def create_environment_map(self, detections_3d: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Create a simple environment map from 3D detections.
        """
        environment_map = {
            'objects': [],
            'spatial_relationships': [],
            'navigation_grid': self.create_navigation_grid(detections_3d)
        }

        for detection in detections_3d:
            environment_map['objects'].append({
                'class': detection['class'],
                'position': detection['position_3d'],
                'confidence': detection['confidence']
            })

        return environment_map

    def create_navigation_grid(self, detections_3d: List[Dict[str, Any]]) -> List[List[int]]:
        """
        Create a simple navigation grid based on object positions.
        """
        # Create a 2D grid representing the environment
        grid_size = 20  # 20x20 grid
        grid_resolution = 0.1  # 10cm per cell
        grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

        # Mark cells with detected objects as occupied
        for detection in detections_3d:
            pos = detection['position_3d']
            # Convert 3D position to grid coordinates (centered around robot)
            grid_x = int(pos['x'] / grid_resolution) + grid_size // 2
            grid_y = int(pos['y'] / grid_resolution) + grid_size // 2

            # Check bounds
            if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                grid[grid_x][grid_y] = 1  # Mark as occupied

        return grid

    def publish_perception_result(self, result: Dict[str, Any]):
        """
        Publish the perception result.
        """
        # Publish as JSON string
        result_msg = String()
        result_msg.data = json.dumps(result, default=str)  # Handle numpy types
        self.fused_perception_pub.publish(result_msg)

        # Also publish 3D detections in standard format
        detection_3d_array = Detection3DArray()
        detection_3d_array.header.stamp = self.get_clock().now().to_msg()
        detection_3d_array.header.frame_id = 'camera_link'

        for detection in result['detections_3d']:
            # Convert to standard message format
            pass  # Implementation would depend on exact message definition

        self.detection_3d_pub.publish(detection_3d_array)


def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorFusionNode()

    # Timer for processing synchronized data
    timer = node.create_timer(0.1, node.synchronize_and_process)  # 10 Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Semantic Segmentation Integration

Integrate semantic segmentation for scene understanding:

```python
import torch
import torchvision.transforms as transforms
from PIL import Image as PILImage
import numpy as np

class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__('semantic_segmentation')

        # Publishers and subscribers
        self.segmentation_pub = self.create_publisher(Image, 'segmentation_mask', 10)
        self.image_sub = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Initialize segmentation model (using torchvision's DeepLabV3 as example)
        self.segmentation_model = torch.hub.load('pytorch/vision:v0.10.0', 'deeplabv3_resnet101', pretrained=True)
        self.segmentation_model.eval()

        # COCO dataset class names (80 classes)
        self.coco_classes = [
            '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
            'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
            'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
            'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table',
            'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
            'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Image transformation
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def image_callback(self, msg: Image):
        """
        Process incoming image for semantic segmentation.
        """
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convert to PIL and apply transformations
            pil_image = PILImage.fromarray(rgb_image)
            input_tensor = self.transform(pil_image)
            input_batch = input_tensor.unsqueeze(0)  # Create batch dimension

            # Run segmentation
            with torch.no_grad():
                output = self.segmentation_model(input_batch)['out'][0]
                segmentation_map = output.argmax(0).byte().cpu().numpy()

            # Convert segmentation map to color image for visualization
            colored_segmentation = self.colorize_segmentation(segmentation_map)

            # Convert back to ROS image
            segmentation_msg = self.bridge.cv2_to_imgmsg(colored_segmentation, encoding='rgb8')
            segmentation_msg.header = msg.header  # Preserve timestamp and frame_id
            self.segmentation_pub.publish(segmentation_msg)

        except Exception as e:
            self.get_logger().error(f'Error in semantic segmentation: {e}')

    def colorize_segmentation(self, segmentation_map: np.ndarray) -> np.ndarray:
        """
        Convert segmentation map to color image for visualization.
        """
        # Create a color map
        color_map = np.zeros((256, 3), dtype=np.uint8)

        # Generate random colors for each class (or use a predefined color map)
        np.random.seed(42)  # For reproducible colors
        for i in range(len(self.coco_classes)):
            color_map[i] = np.random.randint(0, 255, 3)

        # Apply color map
        colored_image = color_map[segmentation_map]
        return colored_image
```

## Scene Understanding and Context Node

A node that provides high-level scene understanding:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
from typing import Dict, Any, List
from dataclasses import dataclass
import numpy as np

@dataclass
class SceneObject:
    class_name: str
    position_3d: Dict[str, float]
    confidence: float
    bbox_2d: Dict[str, float]
    semantic_info: Dict[str, Any] = None

@dataclass
class SpatialRelationship:
    object1: str
    object2: str
    relationship: str  # 'left_of', 'right_of', 'above', 'below', 'near', 'on_top_of', etc.
    distance: float

class SceneUnderstandingNode(Node):
    def __init__(self):
        super().__init__('scene_understanding')

        # Publishers
        self.scene_description_pub = self.create_publisher(String, 'scene_description', 10)
        self.spatial_relationships_pub = self.create_publisher(String, 'spatial_relationships', 10)

        # Subscribers
        self.perception_sub = self.create_subscription(
            String,
            'fused_perception',
            self.perception_callback,
            10
        )

        # Internal state
        self.scene_objects: List[SceneObject] = []
        self.spatial_relationships: List[SpatialRelationship] = []
        self.scene_history: List[Dict[str, Any]] = []

        self.get_logger().info('Scene Understanding Node initialized')

    def perception_callback(self, msg: String):
        """
        Process perception results and generate scene understanding.
        """
        try:
            perception_data = json.loads(msg.data)

            # Update scene objects
            self.update_scene_objects(perception_data)

            # Analyze spatial relationships
            self.analyze_spatial_relationships()

            # Generate scene description
            scene_description = self.generate_scene_description()

            # Publish results
            self.publish_scene_description(scene_description)
            self.publish_spatial_relationships()

            # Update scene history
            self.scene_history.append({
                'timestamp': perception_data['timestamp'],
                'scene_description': scene_description,
                'object_count': len(self.scene_objects)
            })

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in perception data')

    def update_scene_objects(self, perception_data: Dict[str, Any]):
        """
        Update the list of known scene objects.
        """
        self.scene_objects.clear()

        for detection in perception_data.get('detections_3d', []):
            obj = SceneObject(
                class_name=detection['class'],
                position_3d=detection['position_3d'],
                confidence=detection['confidence'],
                bbox_2d=detection['bbox_2d']
            )
            self.scene_objects.append(obj)

    def analyze_spatial_relationships(self):
        """
        Analyze spatial relationships between objects.
        """
        self.spatial_relationships.clear()

        for i, obj1 in enumerate(self.scene_objects):
            for j, obj2 in enumerate(self.scene_objects):
                if i != j:  # Don't compare object to itself
                    relationship = self.compute_relationship(obj1, obj2)
                    if relationship:
                        self.spatial_relationships.append(relationship)

    def compute_relationship(self, obj1: SceneObject, obj2: SceneObject) -> SpatialRelationship:
        """
        Compute spatial relationship between two objects.
        """
        pos1 = obj1.position_3d
        pos2 = obj2.position_3d

        # Calculate distance
        distance = np.sqrt(
            (pos1['x'] - pos2['x'])**2 +
            (pos1['y'] - pos2['y'])**2 +
            (pos1['z'] - pos2['z'])**2
        )

        # Determine relationship based on positions
        dx = pos1['x'] - pos2['x']
        dy = pos1['y'] - pos2['y']
        dz = pos1['z'] - pos2['z']

        relationship = "near"  # Default relationship

        if abs(dx) > abs(dy) and abs(dx) > abs(dz):
            if dx > 0:
                relationship = "right_of"
            else:
                relationship = "left_of"
        elif abs(dy) > abs(dz):
            if dy > 0:
                relationship = "above"
            else:
                relationship = "below"
        else:
            if dz > 0:
                relationship = "in_front_of"
            else:
                relationship = "behind"

        return SpatialRelationship(
            object1=obj1.class_name,
            object2=obj2.class_name,
            relationship=relationship,
            distance=float(distance)
        )

    def generate_scene_description(self) -> Dict[str, Any]:
        """
        Generate a high-level description of the scene.
        """
        # Count objects by class
        object_counts = {}
        for obj in self.scene_objects:
            class_name = obj.class_name
            object_counts[class_name] = object_counts.get(class_name, 0) + 1

        # Identify the most prominent objects
        prominent_objects = sorted(
            self.scene_objects,
            key=lambda obj: obj.confidence,
            reverse=True
        )[:5]  # Top 5 most confident detections

        # Identify spatial patterns
        near_relationships = [
            rel for rel in self.spatial_relationships
            if rel.distance < 1.0  # Objects within 1 meter
        ]

        scene_description = {
            'timestamp': self.get_clock().now().nanoseconds,
            'object_counts': object_counts,
            'prominent_objects': [
                {
                    'class': obj.class_name,
                    'confidence': obj.confidence,
                    'position': obj.position_3d
                }
                for obj in prominent_objects
            ],
            'spatial_patterns': {
                'close_pairs': len(near_relationships),
                'relationships': [
                    {
                        'object1': rel.object1,
                        'object2': rel.object2,
                        'relationship': rel.relationship,
                        'distance': rel.distance
                    }
                    for rel in near_relationships
                ]
            },
            'scene_summary': self.create_scene_summary(object_counts)
        }

        return scene_description

    def create_scene_summary(self, object_counts: Dict[str, int]) -> str:
        """
        Create a textual summary of the scene.
        """
        if not object_counts:
            return "Empty scene - no objects detected."

        # Create summary based on object counts
        summary_parts = []

        for obj_class, count in object_counts.items():
            if count == 1:
                summary_parts.append(f"a {obj_class}")
            else:
                summary_parts.append(f"{count} {obj_class}s")

        if len(summary_parts) == 1:
            scene_summary = f"Scene contains {summary_parts[0]}."
        elif len(summary_parts) == 2:
            scene_summary = f"Scene contains {summary_parts[0]} and {summary_parts[1]}."
        else:
            last_part = summary_parts[-1]
            first_parts = ", ".join(summary_parts[:-1])
            scene_summary = f"Scene contains {first_parts}, and {last_part}."

        return scene_summary

    def publish_scene_description(self, description: Dict[str, Any]):
        """
        Publish scene description.
        """
        msg = String()
        msg.data = json.dumps(description)
        self.scene_description_pub.publish(msg)

    def publish_spatial_relationships(self):
        """
        Publish spatial relationships.
        """
        relationships_data = [
            {
                'object1': rel.object1,
                'object2': rel.object2,
                'relationship': rel.relationship,
                'distance': rel.distance
            }
            for rel in self.spatial_relationships
        ]

        msg = String()
        msg.data = json.dumps(relationships_data)
        self.spatial_relationships_pub.publish(msg)

    def get_object_by_class(self, class_name: str) -> List[SceneObject]:
        """
        Get all objects of a specific class.
        """
        return [obj for obj in self.scene_objects if obj.class_name == class_name]

    def get_nearest_object(self, target_class: str, reference_pos: Dict[str, float]) -> SceneObject:
        """
        Get the nearest object of a specific class to a reference position.
        """
        target_objects = self.get_object_by_class(target_class)

        if not target_objects:
            return None

        # Find nearest object
        nearest_obj = min(
            target_objects,
            key=lambda obj: np.sqrt(
                (obj.position_3d['x'] - reference_pos['x'])**2 +
                (obj.position_3d['y'] - reference_pos['y'])**2 +
                (obj.position_3d['z'] - reference_pos['z'])**2
            )
        )

        return nearest_obj


def main(args=None):
    rclpy.init(args=args)
    node = SceneUnderstandingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Perception Pipeline Integration Node

A node that orchestrates the entire perception pipeline:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import json
from typing import Dict, Any, List
import time
import threading

class PerceptionPipelineNode(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # Publishers
        self.perception_status_pub = self.create_publisher(String, 'perception_pipeline_status', 10)
        self.integrated_perception_pub = self.create_publisher(String, 'integrated_perception', 10)

        # Subscribers
        self.vision_input_sub = self.create_subscription(
            String,
            'vision_pipeline_input',
            self.vision_input_callback,
            10
        )

        self.scene_description_sub = self.create_subscription(
            String,
            'scene_description',
            self.scene_description_callback,
            10
        )

        # Internal state
        self.pipeline_components = {
            'object_detection': {'status': 'idle', 'last_update': 0},
            'semantic_segmentation': {'status': 'idle', 'last_update': 0},
            'spatial_analysis': {'status': 'idle', 'last_update': 0},
            'scene_understanding': {'status': 'idle', 'last_update': 0}
        }

        self.integrated_results = {}
        self.pipeline_lock = threading.Lock()

        # Timer for pipeline status updates
        self.status_timer = self.create_timer(1.0, self.publish_pipeline_status)

        self.get_logger().info('Perception Pipeline Node initialized')

    def vision_input_callback(self, msg: String):
        """
        Handle vision pipeline input requests.
        """
        try:
            request = json.loads(msg.data)
            pipeline_type = request.get('pipeline_type', 'default')

            self.get_logger().info(f'Received vision pipeline request: {pipeline_type}')

            # Process the request in a separate thread
            thread = threading.Thread(
                target=self.process_pipeline_request,
                args=(request,)
            )
            thread.daemon = True
            thread.start()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in vision pipeline request')

    def scene_description_callback(self, msg: String):
        """
        Receive scene descriptions from scene understanding node.
        """
        try:
            scene_data = json.loads(msg.data)
            self.integrated_results['scene_description'] = scene_data
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in scene description')

    def process_pipeline_request(self, request: Dict[str, Any]):
        """
        Process a vision pipeline request.
        """
        with self.pipeline_lock:
            pipeline_type = request.get('pipeline_type', 'default')
            timestamp = request.get('timestamp', time.time())

            self.get_logger().info(f'Processing {pipeline_type} pipeline request')

            # Update component status
            self.update_component_status('object_detection', 'processing')
            self.update_component_status('semantic_segmentation', 'processing')
            self.update_component_status('spatial_analysis', 'processing')
            self.update_component_status('scene_understanding', 'processing')

            # Simulate pipeline processing
            # In a real implementation, this would coordinate with actual pipeline components
            time.sleep(0.5)  # Simulate processing time

            # Generate integrated perception result
            integrated_result = self.generate_integrated_result(request, timestamp)

            # Publish integrated result
            result_msg = String()
            result_msg.data = json.dumps(integrated_result)
            self.integrated_perception_pub.publish(result_msg)

            # Update component status to completed
            self.update_component_status('object_detection', 'completed')
            self.update_component_status('semantic_segmentation', 'completed')
            self.update_component_status('spatial_analysis', 'completed')
            self.update_component_status('scene_understanding', 'completed')

            self.get_logger().info(f'Completed {pipeline_type} pipeline request')

    def generate_integrated_result(self, request: Dict[str, Any], timestamp: float) -> Dict[str, Any]:
        """
        Generate integrated perception result from all pipeline components.
        """
        integrated_result = {
            'request_id': request.get('request_id', 'unknown'),
            'pipeline_type': request.get('pipeline_type', 'default'),
            'timestamp': timestamp,
            'components': {
                'object_detection': self.integrated_results.get('object_detection', {}),
                'semantic_segmentation': self.integrated_results.get('semantic_segmentation', {}),
                'spatial_analysis': self.integrated_results.get('spatial_analysis', {}),
                'scene_description': self.integrated_results.get('scene_description', {})
            },
            'integrated_analysis': self.perform_integrated_analysis(),
            'action_recommendations': self.generate_action_recommendations(),
            'confidence_scores': self.calculate_confidence_scores()
        }

        return integrated_result

    def perform_integrated_analysis(self) -> Dict[str, Any]:
        """
        Perform analysis that integrates information from all components.
        """
        analysis = {
            'environment_type': self.classify_environment(),
            'navigation_feasibility': self.assess_navigation_feasibility(),
            'interaction_opportunities': self.identify_interaction_opportunities(),
            'safety_assessment': self.assess_safety()
        }

        return analysis

    def classify_environment(self) -> str:
        """
        Classify the current environment based on perception results.
        """
        # Placeholder implementation
        # In a real system, this would analyze scene description and object layout
        return "indoor"

    def assess_navigation_feasibility(self) -> Dict[str, float]:
        """
        Assess how feasible navigation is in the current environment.
        """
        # Placeholder implementation
        return {
            'clear_path_ratio': 0.7,
            'obstacle_density': 0.3,
            'navigation_confidence': 0.8
        }

    def identify_interaction_opportunities(self) -> List[Dict[str, Any]]:
        """
        Identify objects and opportunities for robot interaction.
        """
        # Placeholder implementation
        return [
            {
                'object_class': 'cup',
                'position': {'x': 1.0, 'y': 0.5, 'z': 0.8},
                'interaction_type': 'grasp',
                'confidence': 0.9
            }
        ]

    def assess_safety(self) -> Dict[str, float]:
        """
        Assess safety of the current environment.
        """
        # Placeholder implementation
        return {
            'obstacle_proximity_risk': 0.2,
            'navigation_safety': 0.9,
            'overall_safety_score': 0.8
        }

    def generate_action_recommendations(self) -> List[Dict[str, Any]]:
        """
        Generate action recommendations based on perception results.
        """
        recommendations = [
            {
                'action_type': 'NAVIGATE',
                'target': 'kitchen',
                'priority': 1,
                'confidence': 0.8,
                'reasoning': 'Detected cup in kitchen area'
            },
            {
                'action_type': 'APPROACH',
                'target': 'person',
                'priority': 2,
                'confidence': 0.9,
                'reasoning': 'Person detected in front of robot'
            }
        ]

        return recommendations

    def calculate_confidence_scores(self) -> Dict[str, float]:
        """
        Calculate overall confidence scores for the perception results.
        """
        return {
            'object_detection_confidence': 0.85,
            'spatial_understanding_confidence': 0.80,
            'scene_classification_confidence': 0.75,
            'overall_perception_confidence': 0.82
        }

    def update_component_status(self, component: str, status: str):
        """
        Update the status of a pipeline component.
        """
        if component in self.pipeline_components:
            self.pipeline_components[component]['status'] = status
            self.pipeline_components[component]['last_update'] = time.time()

    def publish_pipeline_status(self):
        """
        Publish the current status of the perception pipeline.
        """
        status_msg = String()
        status_msg.data = json.dumps({
            'timestamp': self.get_clock().now().nanoseconds,
            'components': self.pipeline_components,
            'active_requests': 0  # In a real system, track active requests
        })
        self.perception_status_pub.publish(status_msg)

    def get_pipeline_status(self) -> Dict[str, Any]:
        """
        Get the current status of the perception pipeline.
        """
        return {
            'components': self.pipeline_components,
            'overall_status': self.calculate_overall_status(),
            'last_update': time.time()
        }

    def calculate_overall_status(self) -> str:
        """
        Calculate the overall status of the pipeline.
        """
        statuses = [comp['status'] for comp in self.pipeline_components.values()]

        if 'processing' in statuses:
            return 'processing'
        elif 'error' in statuses:
            return 'error'
        else:
            return 'ready'


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPipelineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

- **Modularity**: Design pipeline components to be modular and replaceable
- **Real-time Performance**: Optimize for real-time processing requirements
- **Robustness**: Handle sensor failures and missing data gracefully
- **Calibration**: Ensure proper calibration of all sensors
- **Validation**: Validate perception results before using them for actions
- **Resource Management**: Manage computational resources efficiently
- **Data Fusion**: Effectively combine data from multiple sensors

## Next Steps

With the complete perception pipeline implemented, you now have the full vision component of the VLA system. The next chapter covers the capstone project that integrates all VLA components. Continue with the [Capstone: The Autonomous Humanoid](../capstone-vla/index.md) chapter.