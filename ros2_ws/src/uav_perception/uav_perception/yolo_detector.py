#!/usr/bin/env python3
"""
YOLO Landing Marker Detector Node.

Subscribes to camera images, runs YOLOv8 inference,
publishes bounding box detections for landing markers.

Model: YOLOv8n fine-tuned on landing pad dataset.
       Falls back to COCO pre-trained if custom model not found.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from flight_core.msg import YoloDetection, YoloDetections

import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class', 'armor')
        self.declare_parameter('inference_size', 640)
        self.declare_parameter('max_fps', 10.0)  # Limit inference rate

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_class = self.get_parameter('target_class').value
        self.inference_size = self.get_parameter('inference_size').value
        max_fps = self.get_parameter('max_fps').value

        self.bridge = CvBridge()
        self.model = None
        self.camera_matrix = None

        # Rate limiting
        self.min_interval = 1.0 / max_fps
        self.last_inference_time = 0.0

        if not YOLO_AVAILABLE:
            self.get_logger().error(
                'ultralytics not installed! Run: pip install ultralytics')
            return

        # Load model
        if model_path:
            self.get_logger().info(f'Loading custom YOLO model: {model_path}')
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn(
                'No custom model specified, loading YOLOv8n (COCO pre-trained). '
                'Set model_path param for landing pad detection.')
            self.model = YOLO('yolov8n.pt')

        self.get_logger().info(
            f'YOLO detector ready (conf={self.conf_threshold}, '
            f'target_class={self.target_class}, size={self.inference_size})')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera_down/image_raw', self.image_callback, 10)

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera_down/camera_info', self.info_callback, 10)

        # Publishers
        self.detection_pub = self.create_publisher(
            YoloDetections, '/perception/yolo_detections', 10)

        self.debug_pub = self.create_publisher(
            Image, '/perception/yolo_debug_image', 10)

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.get_logger().info('Camera info received for YOLO node.')

    def image_callback(self, msg):
        if self.model is None:
            return

        # Rate limiting
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_inference_time) < self.min_interval:
            return
        self.last_inference_time = current_time

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Run YOLO inference
        results = self.model(
            cv_image,
            imgsz=self.inference_size,
            conf=self.conf_threshold,
            verbose=False,
        )

        # Build detection output
        det_msg = YoloDetections()
        det_msg.header = msg.header

        debug_image = cv_image.copy()

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                # Filter by target class (if using custom model)
                # For COCO pre-trained, accept all detections
                # User can filter by class_name later
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                w = int(x2 - x1)
                h = int(y2 - y1)

                det = YoloDetection()
                det.class_name = class_name
                det.confidence = confidence
                det.x_center = cx
                det.y_center = cy
                det.width = w
                det.height = h
                det_msg.detections.append(det)

                # Debug visualization
                color = (0, 255, 0) if class_name == self.target_class else (0, 165, 255)
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
                label = f'{class_name} {confidence:.2f}'
                cv2.putText(debug_image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publish
        self.detection_pub.publish(det_msg)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
