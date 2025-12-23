"""
Vision Processing Example for Robot Perception

This file demonstrates complete vision processing pipeline
for humanoid robot perception and action systems.
"""

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String
import json
from typing import List, Dict, Any, Tuple
import threading
import queue

class RobotVisionProcessor:
    """
    Complete vision processing pipeline for robot perception.
    """
    def __init__(self, model_path: str = "yolov8n.pt"):
        # Initialize computer vision components
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.bridge = CvBridge()

        # Camera parameters (to be set from calibration)
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Processing queues
        self.image_queue = queue.Queue(maxsize=5)
        self.processing_lock = threading.Lock()

        # Publishers and subscribers will be set externally
        self.image_sub = None
        self.camera_info_sub = None
        self.detection_pub = None
        self.visualization_pub = None

    def set_camera_parameters(self, camera_matrix: np.ndarray, distortion_coeffs: np.ndarray = None):
        """
        Set camera calibration parameters.
        """
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs if distortion_coeffs is not None else np.zeros((4, 1))

    def process_image(self, cv_image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Process image and return object detections.
        """
        with self.processing_lock:
            # Run object detection
            results = self.model(cv_image, conf=0.5)

            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
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

                        # Add 3D position if camera parameters are available
                        if self.camera_matrix is not None:
                            detection['position_3d'] = self.estimate_3d_position(
                                detection['bbox'],
                                object_height=0.1  # Default object height in meters
                            )

                        detections.append(detection)

            return detections

    def estimate_3d_position(self, bbox_2d: Dict[str, float], object_real_height: float) -> Dict[str, float]:
        """
        Estimate 3D position of object using monocular vision.
        """
        # Calculate object height in pixels
        object_height_px = bbox_2d['y2'] - bbox_2d['y1']

        # Get camera focal length
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        focal_length = (fx + fy) / 2.0

        # Calculate distance using similar triangles
        distance = (object_real_height * focal_length) / object_height_px

        # Calculate 3D coordinates
        center_x = bbox_2d['center_x']
        center_y = bbox_2d['center_y']
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        angle_x = np.arctan2((center_x - cx), fx)
        angle_y = np.arctan2((center_y - cy), fy)

        x = distance * np.tan(angle_x)
        y = distance * np.tan(angle_y)
        z = distance

        return {'x': float(x), 'y': float(y), 'z': float(z)}

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

            # Draw 3D position if available
            if 'position_3d' in detection:
                pos = detection['position_3d']
                pos_text = f"3D: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})"
                cv2.putText(output_image, pos_text, (bbox['x1'], bbox['y1'] - 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        return output_image

    def detections_to_ros(self, detections: List[Dict[str, Any]], header) -> Detection2DArray:
        """
        Convert detections to ROS Detection2DArray message.
        """
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_2d = Detection2D()
            detection_2d.header = header

            # Bounding box
            bbox = detection['bbox']
            detection_2d.bbox.center.x = (bbox['x1'] + bbox['x2']) / 2.0
            detection_2d.bbox.center.y = (bbox['y1'] + bbox['y2']) / 2.0
            detection_2d.bbox.size_x = bbox['x2'] - bbox['x1']
            detection_2d.bbox.size_y = bbox['y2'] - bbox['y1']

            # Results (classification)
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = detection['class_name']
            result.hypothesis.score = detection['confidence']
            detection_2d.results.append(result)

            detection_array.detections.append(detection_2d)

        return detection_array


class ROSVisionNode:
    """
    ROS node wrapper for vision processing.
    """
    def __init__(self, node_name: str = "robot_vision_processor"):
        rospy.init_node(node_name)

        # Initialize vision processor
        self.vision_processor = RobotVisionProcessor()

        # Publishers
        self.detection_pub = rospy.Publisher('detected_objects', Detection2DArray, queue_size=10)
        self.visualization_pub = rospy.Publisher('vision_visualization', Image, queue_size=10)
        self.object_info_pub = rospy.Publisher('object_info', String, queue_size=10)

        # Subscribers
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('camera/camera_info', CameraInfo, self.camera_info_callback)

        # Processing timer
        self.process_timer = rospy.Timer(rospy.Duration(0.1), self.process_timer_callback)  # 10 Hz

        # Internal state
        self.latest_image = None
        self.image_queue = queue.Queue(maxsize=2)
        self.processing_lock = threading.Lock()

        rospy.loginfo('Robot Vision Processor Node initialized')

    def image_callback(self, msg: Image):
        """
        Receive and queue camera images for processing.
        """
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.vision_processor.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Add to processing queue
            try:
                self.image_queue.put_nowait(cv_image)
            except queue.Full:
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put_nowait(cv_image)
                except queue.Empty:
                    pass

        except Exception as e:
            rospy.logerr(f'Error converting image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """
        Receive camera calibration information.
        """
        camera_matrix = np.array(msg.K).reshape(3, 3)
        distortion_coeffs = np.array(msg.D)
        self.vision_processor.set_camera_parameters(camera_matrix, distortion_coeffs)

    def process_timer_callback(self, event):
        """
        Timer callback to process queued images.
        """
        # Get latest image from queue
        latest_image = None
        while not self.image_queue.empty():
            try:
                latest_image = self.image_queue.get_nowait()
            except queue.Empty:
                break

        if latest_image is not None:
            # Process the image in a separate thread
            processing_thread = threading.Thread(
                target=self.process_image_thread,
                args=(latest_image, event.current_real)
            )
            processing_thread.daemon = True
            processing_thread.start()

    def process_image_thread(self, cv_image: np.ndarray, timestamp):
        """
        Process image in separate thread to avoid blocking.
        """
        with self.processing_lock:
            try:
                # Process image
                detections = self.vision_processor.process_image(cv_image)

                # Create header for messages
                header = rospy.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'camera_link'

                # Publish detections
                detection_msg = self.vision_processor.detections_to_ros(detections, header)
                self.detection_pub.publish(detection_msg)

                # Draw detections on image and publish visualization
                visualization_image = self.vision_processor.draw_detections(cv_image, detections)
                visualization_msg = self.vision_processor.bridge.cv2_to_imgmsg(visualization_image, encoding='bgr8')
                visualization_msg.header = header
                self.visualization_pub.publish(visualization_msg)

                # Publish object information
                info_msg = String()
                info_msg.data = json.dumps({
                    'timestamp': header.stamp.to_sec(),
                    'object_count': len(detections),
                    'objects': [
                        {
                            'class': d['class_name'],
                            'confidence': d['confidence'],
                            'bbox': d['bbox'],
                            'position_3d': d.get('position_3d', None)
                        } for d in detections
                    ]
                })
                self.object_info_pub.publish(info_msg)

                rospy.loginfo_throttle(5.0, f'Processed image with {len(detections)} objects detected')

            except Exception as e:
                rospy.logerr(f'Error processing image: {e}')

    def spin(self):
        """
        Start the ROS node spinning.
        """
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo('Shutting down vision processor node')


def create_vision_action_mapper():
    """
    Example of how to map vision outputs to robot actions.
    """
    class VisionActionMapper:
        def __init__(self):
            self.action_mapping = {
                'person': self.approach_person,
                'cup': self.grasp_cup,
                'bottle': self.grasp_bottle,
                'chair': self.navigate_around,
                'table': self.evaluate_surface
            }

        def map_detection_to_action(self, detection: Dict[str, Any]):
            """
            Map a detection to an appropriate action.
            """
            class_name = detection['class_name']
            confidence = detection['confidence']

            if confidence > 0.7 and class_name in self.action_mapping:
                return self.action_mapping[class_name](detection)
            else:
                return None

        def approach_person(self, detection: Dict[str, Any]):
            """
            Action: Approach detected person.
            """
            position = detection.get('position_3d', {'x': 0, 'y': 0, 'z': 1})
            return {
                'action_type': 'NAVIGATE_TO_POSITION',
                'target_position': position,
                'action_description': f'Approach person at {position}'
            }

        def grasp_cup(self, detection: Dict[str, Any]):
            """
            Action: Grasp detected cup.
            """
            position = detection.get('position_3d', {'x': 0, 'y': 0, 'z': 0.8})
            return {
                'action_type': 'GRASP_OBJECT',
                'target_position': position,
                'object_class': 'cup',
                'action_description': f'Grasp cup at {position}'
            }

        def grasp_bottle(self, detection: Dict[str, Any]):
            """
            Action: Grasp detected bottle.
            """
            position = detection.get('position_3d', {'x': 0, 'y': 0, 'z': 0.8})
            return {
                'action_type': 'GRASP_OBJECT',
                'target_position': position,
                'object_class': 'bottle',
                'action_description': f'Grasp bottle at {position}'
            }

        def navigate_around(self, detection: Dict[str, Any]):
            """
            Action: Navigate around detected chair.
            """
            position = detection.get('position_3d', {'x': 0, 'y': 0, 'z': 0})
            return {
                'action_type': 'NAVIGATE_AROUND',
                'obstacle_position': position,
                'action_description': f'Navigate around chair at {position}'
            }

        def evaluate_surface(self, detection: Dict[str, Any]):
            """
            Action: Evaluate detected table surface.
            """
            position = detection.get('position_3d', {'x': 0, 'y': 0, 'z': 0})
            return {
                'action_type': 'EVALUATE_SURFACE',
                'surface_position': position,
                'action_description': f'Evaluate table surface at {position}'
            }

    return VisionActionMapper()


def main():
    """
    Example usage of the vision processing system.
    """
    # Example 1: Basic vision processing
    print("=== Robot Vision Processing Example ===")

    # Initialize vision processor
    vision_processor = RobotVisionProcessor()

    # Example: Process a sample image (in practice, this would come from camera)
    # For demonstration, we'll create a blank image
    sample_image = np.zeros((480, 640, 3), dtype=np.uint8)
    sample_image[:] = [128, 128, 128]  # Gray background

    print("Processing sample image...")
    detections = vision_processor.process_image(sample_image)
    print(f"Found {len(detections)} objects in sample image (should be 0)")

    # Example 2: Create action mapper
    print("\n=== Vision-Action Mapping Example ===")
    action_mapper = create_vision_action_mapper()

    # Example detection (simulated)
    sample_detection = {
        'class_name': 'person',
        'confidence': 0.9,
        'bbox': {'x1': 100, 'y1': 100, 'x2': 200, 'y2': 300, 'center_x': 150, 'center_y': 200},
        'position_3d': {'x': 1.0, 'y': 0.5, 'z': 0.0}
    }

    action = action_mapper.map_detection_to_action(sample_detection)
    if action:
        print(f"Detected: person with confidence 0.9")
        print(f"Mapped action: {action['action_description']}")

    # Example 3: ROS integration (would run in actual ROS node)
    print("\n=== ROS Integration Example ===")
    print("Vision processing node would subscribe to:")
    print("- /camera/image_raw (sensor_msgs/Image)")
    print("- /camera/camera_info (sensor_msgs/CameraInfo)")
    print("And publish to:")
    print("- /detected_objects (vision_msgs/Detection2DArray)")
    print("- /vision_visualization (sensor_msgs/Image)")
    print("- /object_info (std_msgs/String)")

    print("\nTo run the ROS node:")
    print("rosrun your_package vision_node.py")


if __name__ == '__main__':
    main()