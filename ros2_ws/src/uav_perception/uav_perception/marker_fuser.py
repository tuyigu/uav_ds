#!/usr/bin/env python3
"""
Marker Fuser Node — Height-aware YOLO ↔ ArUco fusion.

Strategy:
  - Height > switch_altitude (default 5m): Use YOLO bbox center → estimate map position
  - Height ≤ switch_altitude:              Use ArUco pose (cm-level precision)

Output: /perception/landing_target (LandingTarget msg)
  Downstream consumers (SearchMarkerAction, PrecisionLandAction) don't need
  to know whether the source is YOLO or ArUco.
"""
import math
import rclpy
from rclpy.node import Node
from flight_core.msg import (
    ArucoMarkers,
    YoloDetections,
    LandingTarget,
    UavState,
)
from geometry_msgs.msg import PoseStamped


class MarkerFuserNode(Node):
    def __init__(self):
        super().__init__('marker_fuser')

        # Parameters
        self.declare_parameter('switch_altitude', 5.0)
        self.declare_parameter('target_class', 'armor')
        self.declare_parameter('min_yolo_confidence', 0.4)

        self.switch_alt = self.get_parameter('switch_altitude').value
        self.target_class = self.get_parameter('target_class').value
        self.min_yolo_conf = self.get_parameter('min_yolo_confidence').value

        # State
        self.current_height = 0.0  # AGL (above ground level)
        self.current_x = 0.0
        self.current_y = 0.0
        self.camera_fov_h = math.radians(62.2)  # Horizontal FOV (typical downward camera)
        self.camera_fov_v = math.radians(48.8)  # Vertical FOV
        self.image_width = 640
        self.image_height = 480

        # Last known targets (for smoothing / fallback)
        self.last_aruco_target = None
        self.last_yolo_target = None

        # Subscribers
        self.create_subscription(
            UavState, '/flight/uav_state', self._on_uav_state, 10)
        self.create_subscription(
            ArucoMarkers, '/perception/aruco_markers', self._on_aruco, 10)
        self.create_subscription(
            YoloDetections, '/perception/yolo_detections', self._on_yolo, 10)

        # Publisher
        self.target_pub = self.create_publisher(
            LandingTarget, '/perception/landing_target', 10)

        self.get_logger().info(
            f'Marker Fuser ready (switch_alt={self.switch_alt}m, '
            f'target_class={self.target_class})')

    def _on_uav_state(self, msg: UavState):
        """Update current drone position."""
        self.current_x = msg.x
        self.current_y = msg.y
        # Height AGL: we use absolute Z (NED → positive up convention)
        self.current_height = abs(msg.z)

    def _on_aruco(self, msg: ArucoMarkers):
        """Process ArUco detections (precise, low altitude)."""
        if not msg.markers:
            return

        # Use first detected marker (could filter by ID later)
        marker = msg.markers[0]

        target = LandingTarget()
        target.header = msg.header
        target.source = 'aruco'
        target.confidence = 1.0  # ArUco is binary: detected or not
        target.is_precise = True

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = marker.pose
        target.target_pose = pose

        self.last_aruco_target = target

        # Always publish ArUco if available AND we're low enough
        # (even if YOLO is also publishing)
        if self.current_height <= self.switch_alt:
            self.target_pub.publish(target)
            self.get_logger().debug(
                f'ArUco target at ({marker.pose.position.x:.2f}, '
                f'{marker.pose.position.y:.2f}), h={self.current_height:.1f}m')

    def _on_yolo(self, msg: YoloDetections):
        """Process YOLO detections (coarse, high altitude)."""
        if not msg.detections:
            return

        # Find best landing pad detection
        best = None
        for det in msg.detections:
            if det.confidence < self.min_yolo_conf:
                continue
            # Accept target_class match OR any class if using COCO fallback
            if best is None or det.confidence > best.confidence:
                best = det

        if best is None:
            return

        # Estimate map position from pixel coordinates + drone height
        est_x, est_y = self._pixel_to_ground(
            best.x_center, best.y_center)

        target = LandingTarget()
        target.header = msg.header
        target.source = 'yolo'
        target.confidence = best.confidence
        target.is_precise = False

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose.position.x = est_x
        pose.pose.position.y = est_y
        pose.pose.position.z = 0.0  # Ground level assumption
        pose.pose.orientation.w = 1.0
        target.target_pose = pose

        self.last_yolo_target = target

        # Only publish YOLO if we're high (ArUco not reliable at long range)
        if self.current_height > self.switch_alt:
            self.target_pub.publish(target)
            self.get_logger().info(
                f'YOLO target est. ({est_x:.2f}, {est_y:.2f}), '
                f'conf={best.confidence:.2f}, h={self.current_height:.1f}m')

    def _pixel_to_ground(self, px: int, py: int) -> tuple:
        """
        Estimate ground position from pixel coordinates.

        Uses pinhole camera model + known altitude to project
        image center offset to ground plane.

        This gives meter-level accuracy (good enough for Phase 4 approach).

        Args:
            px: pixel x coordinate (center of bbox)
            py: pixel y coordinate (center of bbox)

        Returns:
            (est_x, est_y) in map frame
        """
        # Convert pixel offset from image center to angle
        dx_px = px - self.image_width / 2.0
        dy_px = py - self.image_height / 2.0

        # Angular offset
        angle_x = dx_px / self.image_width * self.camera_fov_h
        angle_y = dy_px / self.image_height * self.camera_fov_v

        # Ground offset at current height
        ground_dx = self.current_height * math.tan(angle_x)
        ground_dy = self.current_height * math.tan(angle_y)

        # Map coordinates (drone position + ground offset)
        est_x = self.current_x + ground_dx
        est_y = self.current_y + ground_dy

        return est_x, est_y


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFuserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
