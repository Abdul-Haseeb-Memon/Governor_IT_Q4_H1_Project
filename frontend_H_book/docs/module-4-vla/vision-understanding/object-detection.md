---
sidebar_position: 2
title: 'Object Detection and Identification'
---

# Object Detection and Identification

This guide covers implementing object detection and identification systems for humanoid robots. You'll learn how to use computer vision techniques to detect, identify, and classify objects in the robot's environment.

## Overview

Object detection is a critical component of robot perception, enabling robots to understand their environment and interact with objects. This involves:
- Detecting objects in camera feeds
- Classifying objects into known categories
- Localizing objects in 3D space
- Tracking objects over time

## Computer Vision Setup

First, let's set up the computer vision pipeline with required dependencies:

```bash
pip install opencv-python
pip install torch torchvision  # For PyTorch models
pip install ultralytics      # For YOLO models
pip install numpy
pip install pillow
```

## Basic Object Detection with YOLO

YOLO (You Only Look Once) is a popular real-time object detection system:

```python
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from typing import List, Dict, Any, Tuple
import time

class ObjectDetector:
    def __init__(self, model_path: str = "yolov8n.pt"):
        """
        Initialize object detector with YOLO model.
        """
        self.model = YOLO(model_path)
        self.class_names = self.model.names  # Dictionary mapping class IDs to names

    def detect_objects(self, image: np.ndarray, confidence_threshold: float = 0.5) -> List[Dict[str, Any]]:
        """
        Detect objects in an image.

        Args:
            image: Input image as numpy array (BGR format from OpenCV)
            confidence_threshold: Minimum confidence for detections

        Returns:
            List of detected objects with bounding boxes and class information
        """
        # Run inference
        results = self.model(image, conf=confidence_threshold)

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Extract bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = self.class_names[class_id]

                    detection = {
                        'class_id': class_id,
                        'class_name': class_name,
                        'confidence': confidence,
                        'bbox': {
                            'x1': int(x1),
                            'y1': int(y1),
                            'x2': int(x2),
                            'y2': int(y2),
                            'center_x': int((x1 + x2) / 2),
                            'center_y': int((y1 + y2) / 2)
                        },
                        'area': (x2 - x1) * (y2 - y1)
                    }
                    detections.append(detection)

        return detections

    def draw_detections(self, image: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """
        Draw bounding boxes and labels on image.
        """
        output_image = image.copy()

        for detection in detections:
            bbox = detection['bbox']
            class_name = detection['class_name']
            confidence = detection['confidence']

            # Draw bounding box
            cv2.rectangle(output_image, (bbox['x1'], bbox['y1']), (bbox['x2'], bbox['y2']), (0, 255, 0), 2)

            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(output_image, label, (bbox['x1'], bbox['y1'] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return output_image

# Example usage
if __name__ == "__main__":
    detector = ObjectDetector()

    # Load an image
    image = cv2.imread("test_image.jpg")
    if image is not None:
        detections = detector.detect_objects(image)
        print(f"Detected {len(detections)} objects")

        for detection in detections:
            print(f"Class: {detection['class_name']}, Confidence: {detection['confidence']:.2f}")

        # Draw detections on image
        output_image = detector.draw_detections(image, detections)
        cv2.imwrite("output_with_detections.jpg", output_image)
```

## Real-time Object Detection

For real-time processing from a camera feed:

```python
class RealTimeObjectDetector(ObjectDetector):
    def __init__(self, model_path: str = "yolov8n.pt", camera_index: int = 0):
        super().__init__(model_path)
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(camera_index)

        # Check if camera opened successfully
        if not self.cap.isOpened():
            raise ValueError(f"Cannot open camera {camera_index}")

    def run_real_time_detection(self, confidence_threshold: float = 0.5):
        """
        Run real-time object detection from camera feed.
        """
        print("Starting real-time object detection. Press 'q' to quit.")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Detect objects
            start_time = time.time()
            detections = self.detect_objects(frame, confidence_threshold)
            inference_time = time.time() - start_time

            # Draw detections
            output_frame = self.draw_detections(frame, detections)

            # Display inference time
            fps = 1.0 / inference_time if inference_time > 0 else 0
            cv2.putText(output_frame, f"FPS: {fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Display the frame
            cv2.imshow("Object Detection", output_frame)

            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release resources
        self.cap.release()
        cv2.destroyAllWindows()

    def get_detection_summary(self, detections: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Get a summary of detections for higher-level processing.
        """
        summary = {
            'total_objects': len(detections),
            'classes_detected': {},
            'largest_object': None,
            'closest_object': None
        }

        if not detections:
            return summary

        # Count objects by class
        for detection in detections:
            class_name = detection['class_name']
            summary['classes_detected'][class_name] = summary['classes_detected'].get(class_name, 0) + 1

        # Find largest object (by area)
        largest = max(detections, key=lambda x: x['area'])
        summary['largest_object'] = largest

        # Find closest object (by center_y - assuming camera is level)
        closest = min(detections, key=lambda x: x['bbox']['center_y'])
        summary['closest_object'] = closest

        return summary
```

## Custom Object Detection

For detecting specific objects relevant to robotics tasks:

```python
class CustomObjectDetector(ObjectDetector):
    def __init__(self, model_path: str = "yolov8n.pt", custom_classes: List[str] = None):
        super().__init__(model_path)

        # Define custom classes relevant to robotics
        self.robotics_classes = custom_classes or [
            'cup', 'bottle', 'book', 'chair', 'table', 'person',
            'robot', 'phone', 'laptop', 'box', 'ball'
        ]

    def detect_robotics_objects(self, image: np.ndarray, confidence_threshold: float = 0.5) -> List[Dict[str, Any]]:
        """
        Detect only objects relevant to robotics tasks.
        """
        all_detections = self.detect_objects(image, confidence_threshold)

        # Filter for robotics-relevant classes
        robotics_detections = [
            detection for detection in all_detections
            if detection['class_name'].lower() in [c.lower() for c in self.robotics_classes]
        ]

        return robotics_detections

    def find_object_by_name(self, image: np.ndarray, target_object: str, confidence_threshold: float = 0.5) -> List[Dict[str, Any]]:
        """
        Find specific objects by name in the image.
        """
        all_detections = self.detect_objects(image, confidence_threshold)

        target_detections = [
            detection for detection in all_detections
            if target_object.lower() in detection['class_name'].lower()
        ]

        return target_detections

    def find_interactable_objects(self, image: np.ndarray, confidence_threshold: float = 0.5) -> List[Dict[str, Any]]:
        """
        Find objects that can be interacted with (manipulated).
        """
        all_detections = self.detect_objects(image, confidence_threshold)

        # Define objects that are typically manipulable
        manipulable_classes = [
            'cup', 'bottle', 'book', 'phone', 'laptop', 'box', 'ball', 'toy',
            'container', 'tool', 'utensil', 'remote', 'keys', 'wallet'
        ]

        interactable_detections = [
            detection for detection in all_detections
            if detection['class_name'].lower() in manipulable_classes
        ]

        return interactable_detections
```

## 3D Object Localization

To get 3D positions of objects from 2D detections:

```python
class Object3DLocalizer:
    def __init__(self, camera_matrix: np.ndarray, distortion_coeffs: np.ndarray = None):
        """
        Initialize 3D localizer with camera parameters.

        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            distortion_coeffs: Distortion coefficients (optional)
        """
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs if distortion_coeffs is not None else np.zeros((4, 1))

    def estimate_3d_position(self, bbox_2d: Dict[str, float], object_real_height: float) -> Tuple[float, float, float]:
        """
        Estimate 3D position of object using monocular vision and known object height.

        Args:
            bbox_2d: 2D bounding box with center coordinates and dimensions
            object_real_height: Real height of the object in meters

        Returns:
            (x, y, z) coordinates in camera frame
        """
        # Calculate object height in pixels
        object_height_px = bbox_2d['y2'] - bbox_2d['y1']

        # Get camera focal length (average of fx and fy)
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        focal_length = (fx + fy) / 2.0

        # Calculate distance using similar triangles
        # (real_height / distance) = (pixel_height / focal_length)
        distance = (object_real_height * focal_length) / object_height_px

        # Calculate 3D coordinates
        # For simplicity, assuming object is at same height as camera
        # In practice, you'd need to account for camera height and object height
        center_x = bbox_2d['center_x']
        center_y = bbox_2d['center_y']

        # Image center
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Calculate angles
        angle_x = np.arctan2((center_x - cx), fx)
        angle_y = np.arctan2((center_y - cy), fy)

        # Calculate 3D position
        x = distance * np.tan(angle_x)
        y = distance * np.tan(angle_y)
        z = distance  # Distance along optical axis

        return x, y, z

    def localize_objects_3d(self, detections: List[Dict[str, Any]],
                           object_heights: Dict[str, float]) -> List[Dict[str, Any]]:
        """
        Add 3D positions to object detections.
        """
        localized_detections = []

        for detection in detections:
            class_name = detection['class_name'].lower()

            if class_name in object_heights:
                x, y, z = self.estimate_3d_position(
                    detection['bbox'],
                    object_heights[class_name]
                )

                detection['position_3d'] = {'x': x, 'y': y, 'z': z}
            else:
                # Use default height if not known
                x, y, z = self.estimate_3d_position(
                    detection['bbox'],
                    0.1  # Default 10cm height
                )
                detection['position_3d'] = {'x': x, 'y': y, 'z': z}

            localized_detections.append(detection)

        return localized_detections
```

## Object Tracking

For tracking objects across frames:

```python
import cv2

class ObjectTracker:
    def __init__(self):
        self.trackers = {}  # Dictionary of active trackers
        self.next_object_id = 0
        self.max_disappeared = 30  # Max frames to keep track without detection

    def update_trackers(self, detections: List[Dict[str, Any]], frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Update object trackers with new detections.
        """
        # Update existing trackers
        for obj_id in list(self.trackers.keys()):
            success, box = self.trackers[obj_id].update(frame)

            if success:
                # Update tracker position
                x, y, w, h = map(int, box)
                self.trackers[obj_id].position = (x, y, w, h)
            else:
                # Remove tracker if tracking failed
                del self.trackers[obj_id]

        # Match detections to existing trackers or create new ones
        updated_detections = []
        used_detections = set()

        for detection in detections:
            bbox = detection['bbox']
            det_center = ((bbox['x1'] + bbox['x2']) // 2, (bbox['y1'] + bbox['y2']) // 2)

            # Find best matching tracker
            best_match = None
            best_distance = float('inf')

            for obj_id, tracker in self.trackers.items():
                if hasattr(tracker, 'position'):
                    tx, ty, tw, th = tracker.position
                    track_center = (tx + tw // 2, ty + th // 2)
                    distance = np.sqrt((det_center[0] - track_center[0])**2 + (det_center[1] - track_center[1])**2)

                    if distance < best_distance and distance < 50:  # Threshold for matching
                        best_distance = distance
                        best_match = obj_id

            if best_match is not None:
                # Update existing tracker with this detection
                detection['object_id'] = best_match
                updated_detections.append(detection)
                used_detections.add(detections.index(detection))
            else:
                # Create new tracker for this detection
                new_tracker = cv2.legacy.TrackerKCF_create()
                x, y, w, h = bbox['x1'], bbox['y1'], bbox['x2'] - bbox['x1'], bbox['y2'] - bbox['y1']
                new_tracker.init(frame, (x, y, w, h))
                new_tracker.position = (x, y, w, h)

                new_obj_id = self.next_object_id
                self.trackers[new_obj_id] = new_tracker
                self.next_object_id += 1

                detection['object_id'] = new_obj_id
                updated_detections.append(detection)
                used_detections.add(detections.index(detection))

        return updated_detections
```

## Best Practices

- **Model Selection**: Choose appropriate models based on your hardware constraints and accuracy needs
- **Confidence Thresholds**: Adjust confidence thresholds based on application requirements
- **Preprocessing**: Preprocess images to improve detection quality (resize, normalize, etc.)
- **Post-processing**: Apply non-maximum suppression and other techniques to refine detections
- **Calibration**: Calibrate cameras for accurate 3D localization
- **Performance**: Optimize for real-time performance if needed

## Next Steps

After implementing object detection and identification, the next step is to link these vision outputs to ROS actions. Continue with the [Linking Vision Outputs to ROS Actions](./ros-linking.md) guide.