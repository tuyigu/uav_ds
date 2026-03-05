#!/usr/bin/env python3
"""
Marker Fuser Node — Height-aware YOLO ↔ ArUco fusion.

Strategy:
  - Height > switch_altitude (default 5m): Use YOLO bbox center → estimate map position
  - Height ≤ switch_altitude:              Use ArUco pose (cm-level precision)
  - Hysteresis band (±1m) to prevent flickering near switch altitude

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
        self.declare_parameter('hysteresis', 1.0)        # ±1m band to prevent flickering
        self.declare_parameter('target_class', 'armor')
        self.declare_parameter('min_yolo_confidence', 0.4)
        self.declare_parameter('target_marker_id', -1)    # -1 = accept any marker

        self.switch_alt = self.get_parameter('switch_altitude').value
        self.hysteresis = self.get_parameter('hysteresis').value
        self.target_class = self.get_parameter('target_class').value
        self.min_yolo_conf = self.get_parameter('min_yolo_confidence').value
        self.target_marker_id = self.get_parameter('target_marker_id').value

        # State
        self.current_height = 0.0  # AGL (above ground level)
        self.current_x = 0.0
        self.current_y = 0.0
        self.camera_fov_h = math.radians(62.2)  # Horizontal FOV (typical downward camera)
        self.camera_fov_v = math.radians(48.8)  # Vertical FOV
        self.image_width = 640
        self.image_height = 480

        # Source tracking with hysteresis
        self.active_source = 'none'  # 'yolo', 'aruco', or 'none'

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
            f'hysteresis=±{self.hysteresis}m, target_class={self.target_class}, '
            f'target_marker_id={self.target_marker_id})')

    def _should_use_aruco(self) -> bool:
        """Determine if ArUco should be the active source, with hysteresis.
        
        Hysteresis prevents flickering near the switch altitude:
        - Switch from YOLO → ArUco when descending below (switch_alt - hysteresis)
        - Switch from ArUco → YOLO when ascending above (switch_alt + hysteresis)
        """
        if self.active_source == 'aruco':
            # Currently using ArUco — stay with ArUco unless significantly above switch alt
            return self.current_height <= (self.switch_alt + self.hysteresis)
        else:
            # Currently using YOLO or none — switch to ArUco only when clearly below switch alt
            return self.current_height <= (self.switch_alt - self.hysteresis)

    def _on_uav_state(self, msg: UavState):
        """Update current drone position."""
        self.current_x = msg.x
        self.current_y = msg.y
        # Height AGL: we use absolute Z (ENU convention, positive up)
        self.current_height = abs(msg.z)

    def _on_aruco(self, msg: ArucoMarkers):
        """Process ArUco detections (precise, low altitude)."""
        if not msg.markers:
            return

        # Filter by target marker ID if specified
        marker = None
        if self.target_marker_id >= 0:
            for m in msg.markers:
                if m.marker_id == self.target_marker_id:
                    marker = m
                    break
            if marker is None:
                return  # Target marker not found in this frame
        else:
            # Use first detected marker
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

        # Publish ArUco if conditions met (with hysteresis)
        if self._should_use_aruco():
            self.active_source = 'aruco'
            self.target_pub.publish(target)
            self.get_logger().debug(
                f'ArUco target (id={marker.marker_id}) at '
                f'({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f}), '
                f'h={self.current_height:.1f}m')

    def _on_yolo(self, msg: YoloDetections):
        """Process YOLO detections (coarse, high altitude)."""
        if not msg.detections:
            return

        # Find best landing pad detection, preferring target class
        best = None
        best_is_target = False
        for det in msg.detections:
            if det.confidence < self.min_yolo_conf:
                continue
            is_target = (det.class_name == self.target_class)
            # Prefer target class over other classes
            if best is None:
                best = det
                best_is_target = is_target
            elif is_target and not best_is_target:
                best = det
                best_is_target = is_target
            elif is_target == best_is_target and det.confidence > best.confidence:
                best = det
                best_is_target = is_target

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

        # Only publish YOLO if ArUco is not active (with hysteresis)
        if not self._should_use_aruco():
            self.active_source = 'yolo'
            self.target_pub.publish(target)
            self.get_logger().info(
                f'YOLO target est. ({est_x:.2f}, {est_y:.2f}), '
                f'conf={best.confidence:.2f}, class={best.class_name}, '
                f'h={self.current_height:.1f}m',
                throttle_duration_sec=2.0)

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
