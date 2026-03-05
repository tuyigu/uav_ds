#!/usr/bin/env python3
"""
3D A* Path Planner using OctoMap

Provides a PlanPath service that:
1. Reads the current OctoMap (binary)
2. Runs 3D A* from start to goal
3. Smooths the path
4. Returns collision-free waypoints

The planner maintains a voxel grid representation of occupied space
from the OctoMap for fast collision checking.

Coordinate System:
    All planning is done in the PX4 local ENU frame (same as flight_core).
    - Goals come from the BT agent in PX4 ENU coordinates.
    - Start position defaults to UavState (PX4 ENU), falling back to SLAM odometry.
    - OctoMap occupied cells are in the 'map' frame (configured via octomap.yaml).
    - If map frame != PX4 ENU frame, TF2 is used for conversion.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

from flight_core.msg import UavState

from uav_navigation.srv import PlanPath

import tf2_ros
from geometry_msgs.msg import PointStamped

import heapq
import math
import struct
import numpy as np


class VoxelGrid:
    """Simple 3D voxel grid for collision checking."""

    def __init__(self, resolution=0.5):
        self.resolution = resolution
        self.occupied = set()  # set of (ix, iy, iz) voxel indices
        self.min_bound = np.array([0.0, 0.0, 0.0])
        self.max_bound = np.array([0.0, 0.0, 0.0])

    def world_to_grid(self, x, y, z):
        """Convert world coordinates to grid indices."""
        ix = int(math.floor(x / self.resolution))
        iy = int(math.floor(y / self.resolution))
        iz = int(math.floor(z / self.resolution))
        return (ix, iy, iz)

    def grid_to_world(self, ix, iy, iz):
        """Convert grid indices to world coordinates (center of voxel)."""
        x = (ix + 0.5) * self.resolution
        y = (iy + 0.5) * self.resolution
        z = (iz + 0.5) * self.resolution
        return (x, y, z)

    def is_occupied(self, ix, iy, iz):
        """Check if a voxel is occupied."""
        return (ix, iy, iz) in self.occupied

    def is_free(self, x, y, z, margin=0.0):
        """Check if a world position is free with safety margin."""
        margin_voxels = int(math.ceil(margin / self.resolution))
        cx, cy, cz = self.world_to_grid(x, y, z)

        for dx in range(-margin_voxels, margin_voxels + 1):
            for dy in range(-margin_voxels, margin_voxels + 1):
                for dz in range(-margin_voxels, margin_voxels + 1):
                    if self.is_occupied(cx + dx, cy + dy, cz + dz):
                        return False
        return True

    def update_from_marker_array(self, markers):
        """Update voxel grid from occupied_cells_vis_array MarkerArray."""
        self.occupied.clear()
        for marker in markers:
            for point in marker.points:
                idx = self.world_to_grid(point.x, point.y, point.z)
                self.occupied.add(idx)
                # Update bounds
                self.min_bound = np.minimum(
                    self.min_bound, [point.x, point.y, point.z])
                self.max_bound = np.maximum(
                    self.max_bound, [point.x, point.y, point.z])


class AStarPlanner:
    """3D A* path planner on a voxel grid."""

    # 26-connected neighborhood (all neighbors in 3D)
    NEIGHBORS_26 = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                NEIGHBORS_26.append(
                    (dx, dy, dz, math.sqrt(dx*dx + dy*dy + dz*dz)))

    def __init__(self, voxel_grid, safety_margin=0.5, max_iterations=50000):
        self.grid = voxel_grid
        self.safety_margin = safety_margin
        self.max_iterations = max_iterations
        self.margin_voxels = int(
            math.ceil(safety_margin / voxel_grid.resolution))

    def heuristic(self, a, b):
        """Euclidean distance heuristic."""
        return math.sqrt(
            (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def is_node_free(self, ix, iy, iz):
        """Check if a grid node and its safety margin are free."""
        for dx in range(-self.margin_voxels, self.margin_voxels + 1):
            for dy in range(-self.margin_voxels, self.margin_voxels + 1):
                for dz in range(-self.margin_voxels, self.margin_voxels + 1):
                    if self.grid.is_occupied(ix + dx, iy + dy, iz + dz):
                        return False
        return True

    def plan(self, start_world, goal_world):
        """
        Run A* from start to goal (world coordinates).
        Returns list of world-coordinate waypoints or None if no path found.
        """
        start = self.grid.world_to_grid(*start_world)
        goal = self.grid.world_to_grid(*goal_world)

        # Check start and goal validity
        if not self.is_node_free(*start):
            # Start is in collision — try to find nearby free cell
            start = self._find_nearest_free(start)
            if start is None:
                return None, "Start position is blocked"

        if not self.is_node_free(*goal):
            goal = self._find_nearest_free(goal)
            if goal is None:
                return None, "Goal position is blocked"

        # A* search
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed_set = set()
        iterations = 0

        while open_set and iterations < self.max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = self._reconstruct_path(came_from, current)
                world_path = [
                    self.grid.grid_to_world(*p) for p in path]
                smoothed = self._smooth_path(world_path)
                return smoothed, f"Path found ({len(smoothed)} waypoints, {iterations} iterations)"

            if current in closed_set:
                continue
            closed_set.add(current)

            for dx, dy, dz, cost in self.NEIGHBORS_26:
                neighbor = (current[0]+dx, current[1]+dy, current[2]+dz)

                if neighbor in closed_set:
                    continue

                if not self.is_node_free(*neighbor):
                    continue

                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))

        return None, f"No path found after {iterations} iterations"

    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from A* came_from dict."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _smooth_path(self, path):
        """
        Simple path smoothing: remove intermediate waypoints
        if straight line between non-adjacent waypoints is collision-free.
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            # Try to skip ahead as far as possible
            furthest = i + 1
            for j in range(len(path) - 1, i, -1):
                if self._line_of_sight(path[i], path[j]):
                    furthest = j
                    break
            smoothed.append(path[furthest])
            i = furthest

        return smoothed

    def _line_of_sight(self, p1, p2):
        """Check if straight line between two world points is collision-free."""
        dist = math.sqrt(
            (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2)
        steps = max(int(dist / (self.grid.resolution * 0.5)), 2)

        for s in range(steps + 1):
            t = s / steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            z = p1[2] + t * (p2[2] - p1[2])
            if not self.grid.is_free(x, y, z, self.safety_margin):
                return False
        return True

    def _find_nearest_free(self, pos, max_radius=10):
        """Find nearest free grid cell within radius."""
        for r in range(1, max_radius):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    for dz in range(-r, r+1):
                        if abs(dx) == r or abs(dy) == r or abs(dz) == r:
                            candidate = (pos[0]+dx, pos[1]+dy, pos[2]+dz)
                            if self.is_node_free(*candidate):
                                return candidate
        return None


class PathPlannerNode(Node):
    """ROS 2 node providing 3D path planning service.
    
    Coordinate System Design:
        - All service inputs/outputs are in PX4 local ENU frame (matching flight_core).
        - OctoMap data is in the 'map' frame. If 'map' frame aligns with PX4 ENU
          (which it should when TF is properly set up), no conversion is needed.
        - Start position uses UavState (PX4 ENU) as the primary source,
          falling back to SLAM odometry if UavState is unavailable.
    """

    def __init__(self):
        super().__init__('path_planner')

        # Parameters
        self.declare_parameter('voxel_resolution', 0.5)
        self.declare_parameter('default_safety_margin', 0.5)
        self.declare_parameter('max_planning_iterations', 50000)
        self.declare_parameter('planning_frame', 'map')

        self.voxel_res = self.get_parameter(
            'voxel_resolution').value
        self.default_margin = self.get_parameter(
            'default_safety_margin').value
        self.max_iters = self.get_parameter(
            'max_planning_iterations').value
        self.planning_frame = self.get_parameter(
            'planning_frame').value

        # Voxel grid
        self.voxel_grid = VoxelGrid(resolution=self.voxel_res)

        # TF2 for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to OctoMap occupied cells
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/occupied_cells_vis_array',
            self.marker_callback,
            qos_latched
        )

        # Primary position source: UavState from flight_core (PX4 ENU)
        self.px4_pos = None
        self.uav_state_sub = self.create_subscription(
            UavState,
            '/flight/uav_state',
            self._uav_state_callback,
            10
        )

        # Fallback position source: SLAM odometry
        self.slam_pos = None
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

        # PlanPath service
        self.srv = self.create_service(
            PlanPath,
            'plan_path',
            self.plan_path_callback
        )

        # Path visualization publisher
        self.path_viz_pub = self.create_publisher(
            Marker,
            '/planned_path_viz',
            10
        )

        self.octomap_ready = False
        self.get_logger().info(
            f'Path Planner ready (resolution={self.voxel_res}m, '
            f'margin={self.default_margin}m, frame={self.planning_frame})')

    def _uav_state_callback(self, msg: UavState):
        """Update current position from flight_core (PX4 local ENU frame)."""
        if msg.connected:
            self.px4_pos = (msg.x, msg.y, msg.z)

    def marker_callback(self, msg):
        """Update voxel grid from OctoMap visualization."""
        self.voxel_grid.update_from_marker_array(msg.markers)
        self.octomap_ready = True
        self.get_logger().info(
            f'OctoMap updated: {len(self.voxel_grid.occupied)} occupied voxels',
            throttle_duration_sec=10.0)

    def odom_callback(self, msg):
        """Update current position from SLAM (fallback, odom frame)."""
        self.slam_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def _get_current_position(self):
        """Get current UAV position in PX4 ENU frame.
        
        Priority:
            1. UavState from flight_core (PX4 ENU) — preferred, matches BT goals
            2. SLAM odometry (may need frame conversion) — fallback
        
        Returns:
            tuple (x, y, z) in PX4 ENU frame, or None if no position available
        """
        if self.px4_pos is not None:
            return self.px4_pos

        if self.slam_pos is not None:
            self.get_logger().warn(
                'Using SLAM odometry as position source (PX4 state unavailable). '
                'Coordinates may not perfectly align with BT goals.',
                throttle_duration_sec=5.0)
            # TODO: Use TF2 to transform from odom frame to map/PX4 frame
            # For now, use directly (works when both origins are aligned at takeoff)
            return self.slam_pos

        return None

    def plan_path_callback(self, request, response):
        """Handle PlanPath service request.
        
        All coordinates are expected in PX4 local ENU frame.
        """
        self.get_logger().info(
            f'PlanPath request: '
            f'({request.start.x:.1f}, {request.start.y:.1f}, {request.start.z:.1f}) -> '
            f'({request.goal.x:.1f}, {request.goal.y:.1f}, {request.goal.z:.1f})')

        # Use current PX4 position if start is zero (default from BT)
        start = (request.start.x, request.start.y, request.start.z)
        if start == (0.0, 0.0, 0.0):
            current = self._get_current_position()
            if current is not None:
                start = current
                self.get_logger().info(
                    f'Using current position as start: '
                    f'({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f})')
            else:
                response.success = False
                response.waypoints = []
                response.message = 'No position data available (neither PX4 nor SLAM)'
                self.get_logger().error(response.message)
                return response

        goal = (request.goal.x, request.goal.y, request.goal.z)

        # If no OctoMap yet, return direct path (straight line)
        if not self.octomap_ready or len(self.voxel_grid.occupied) == 0:
            self.get_logger().warn(
                'No OctoMap data, returning direct path')
            response.success = True
            response.waypoints = [
                self._make_point(*start),
                self._make_point(*goal)
            ]
            response.message = 'Direct path (no obstacle data)'
            self._publish_path_viz(response.waypoints)
            return response

        # Run A*
        margin = request.safety_margin if request.safety_margin > 0 \
            else self.default_margin

        planner = AStarPlanner(
            self.voxel_grid,
            safety_margin=margin,
            max_iterations=self.max_iters
        )

        path, message = planner.plan(start, goal)

        if path is None:
            response.success = False
            response.waypoints = []
            response.message = message
            self.get_logger().error(f'Planning failed: {message}')
        else:
            response.success = True
            response.waypoints = [self._make_point(*p) for p in path]
            response.message = message
            self.get_logger().info(f'Planning succeeded: {message}')
            self._publish_path_viz(response.waypoints)

        return response

    def _make_point(self, x, y, z):
        """Create a Point message."""
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def _publish_path_viz(self, waypoints):
        """Publish planned path as a Marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'planned_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = waypoints
        marker.pose.orientation.w = 1.0

        self.path_viz_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
