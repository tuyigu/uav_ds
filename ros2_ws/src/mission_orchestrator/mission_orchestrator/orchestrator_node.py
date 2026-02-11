#!/usr/bin/env python3
"""
Mission Orchestrator Node.

Sits between web_agent and uav_bt_agent:
  - Receives missions from web_agent
  - Receives operator commands from web
  - Monitors drone health via UavState
  - Monitors BT execution feedback
  - Publishes intents to BT agent
  - Publishes enriched status to web
"""
import math
import time

import rclpy
from rclpy.node import Node

from uav_web_agent.msg import DeliveryMission, MissionStatus, MissionIntent, OperatorCommand
from flight_core.msg import UavState

from .state_models import MissionState, SelfHealth, SLATracker, UserPerception
from .intent_engine import (
    decide_intent,
    reinterpret_bt_failure,
    compute_user_perception,
)


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')

        # ── State Models ──────────────────────────────────────────
        self.mission = MissionState()
        self.health = SelfHealth()
        self.sla = SLATracker()
        self.user = UserPerception()

        # Home position in local ENU (meters) — used for RETURN_HOME
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 0.0)
        self.mission.home_x = self.get_parameter('home_x').value
        self.mission.home_y = self.get_parameter('home_y').value
        self.mission.home_z = self.get_parameter('home_z').value

        # Home GPS coordinates — used for GPS→ENU conversion origin
        self.declare_parameter('home_lat', 47.3977419)
        self.declare_parameter('home_lon', 8.5455939)

        # Flight altitude parameters (configurable)
        self.declare_parameter('cruise_alt', 30.0)
        self.declare_parameter('approach_alt', 15.0)
        self.declare_parameter('search_alt', 5.0)
        self.declare_parameter('takeoff_height', 5.0)
        self.mission.cruise_alt = self.get_parameter('cruise_alt').value
        self.mission.approach_alt = self.get_parameter('approach_alt').value
        self.mission.search_alt = self.get_parameter('search_alt').value
        self.takeoff_height = self.get_parameter('takeoff_height').value

        # Decision loop rate
        self.declare_parameter('loop_rate_hz', 2.0)
        loop_rate = self.get_parameter('loop_rate_hz').value

        # GeoFence parameters
        self.declare_parameter('geofence_radius', 100.0)
        self.declare_parameter('geofence_max_alt', 50.0)
        self.geofence_radius = self.get_parameter('geofence_radius').value
        self.geofence_max_alt = self.get_parameter('geofence_max_alt').value

        # Last published intent (avoid spamming same intent)
        self._last_intent = ""
        self._last_reason = ""

        # ── Subscribers ───────────────────────────────────────────

        # New mission from web_agent
        self.create_subscription(
            DeliveryMission, '/uav/mission/new', self._on_new_mission, 10)

        # Operator commands forwarded by web_agent
        self.create_subscription(
            OperatorCommand, '/uav/command', self._on_operator_command, 10)

        # Drone physical state from flight_core
        self.create_subscription(
            UavState, '/flight/uav_state', self._on_uav_state, 10)

        # BT execution feedback
        self.create_subscription(
            MissionStatus, '/bt/execution_status', self._on_bt_status, 10)

        # ── Publishers ────────────────────────────────────────────

        # Intent → BT Agent
        self.intent_pub = self.create_publisher(
            MissionIntent, '/orchestrator/intent', 10)

        # Enriched status → Web (via web_agent)
        self.status_pub = self.create_publisher(
            MissionStatus, '/web/mission_status', 10)

        # ── Decision Timer ────────────────────────────────────────
        self.create_timer(1.0 / loop_rate, self._decision_loop)

        self.get_logger().info(
            f'Mission Orchestrator started (loop={loop_rate}Hz, '
            f'home=({self.mission.home_x}, {self.mission.home_y}, {self.mission.home_z}))')

    # ── Callbacks ─────────────────────────────────────────────────

    def _on_new_mission(self, msg: DeliveryMission):
        """New mission received from web_agent."""
        if self.mission.is_active():
            self.get_logger().warn(
                f'Busy with {self.mission.mission_id}, rejecting {msg.mission_id}')
            # Publish rejection
            status = MissionStatus()
            status.mission_id = msg.mission_id
            status.uav_id = msg.uav_id
            status.status = 'REJECTED'
            status.reason = 'UAV busy with another mission'
            status.user_facing_msg = '无人机忙碌中，请稍候'
            self.status_pub.publish(status)
            return

        self.get_logger().info(f'Accepted mission: {msg.mission_id}')

        # ── GPS → Local ENU conversion ──────────────────────────
        # Web sends GPS (lat/lon). FlightCore uses local ENU (meters).
        # Convert using Equirectangular approximation (good for <10km range).
        home_lat = self.get_parameter('home_lat').value
        home_lon = self.get_parameter('home_lon').value

        pickup_x, pickup_y = self._gps_to_enu(
            msg.pickup_lat, msg.pickup_lon, home_lat, home_lon)
        dropoff_x, dropoff_y = self._gps_to_enu(
            msg.dropoff_lat, msg.dropoff_lon, home_lat, home_lon)

        self.get_logger().info(
            f'GPS→ENU: pickup ({msg.pickup_lat},{msg.pickup_lon}) → ({pickup_x:.1f},{pickup_y:.1f})m, '
            f'dropoff ({msg.dropoff_lat},{msg.dropoff_lon}) → ({dropoff_x:.1f},{dropoff_y:.1f})m')

        # Initialize mission state
        self.mission.mission_id = msg.mission_id
        self.mission.uav_id = msg.uav_id
        self.mission.pickup_x = pickup_x
        self.mission.pickup_y = pickup_y
        self.mission.pickup_z = msg.pickup_alt
        self.mission.dropoff_x = dropoff_x
        self.mission.dropoff_y = dropoff_y
        self.mission.dropoff_z = msg.dropoff_alt
        self.mission.priority = msg.priority
        self.mission.phase = "PICKUP"
        self.mission.status = "ACCEPTED"
        self.mission.has_cargo = False
        self.mission.retry_count = 0

        # Set initial target: go to merchant
        self.mission.target_x = self.mission.pickup_x
        self.mission.target_y = self.mission.pickup_y
        self.mission.target_z = self.mission.pickup_z

        # Initialize SLA
        self.sla.created_at = time.time()
        self.sla.mission_accepted_at = time.time()
        self.sla.deadline_ts = msg.deadline_ts

        # Reset user perception
        self.user = UserPerception()

        # Force first intent
        self._last_intent = ""
        self._last_reason = ""

        # Publish accepted status
        self._publish_status()

    def _on_operator_command(self, msg: OperatorCommand):
        """Operator command from Web UI."""
        self.get_logger().info(
            f'Operator command: {msg.command} (mission={msg.mission_id}, reason={msg.reason})')

        # Validate mission_id
        if msg.mission_id and msg.mission_id != self.mission.mission_id:
            self.get_logger().warn(
                f'Command for unknown mission {msg.mission_id}, ignoring')
            return

        self.mission.pending_command = msg.command
        self.mission.pending_command_reason = msg.reason

    def _on_uav_state(self, msg: UavState):
        """Drone physical state update from flight_core."""
        self.health.battery_level = msg.battery
        self.health.connected = msg.connected
        self.health.armed = msg.armed
        self.health.pos_x = msg.x
        self.health.pos_y = msg.y
        self.health.pos_z = msg.z
        self.health.flight_phase = msg.phase
        self.health.last_uav_state_time = time.time()

        # Check GeoFence
        self._check_geofence()

        # Infer SLAM health from flight phase
        if msg.phase == "FAIL":
            self.health.slam_ok = False
        elif msg.sub_phase == "SLAM_LOST":
            self.health.slam_ok = False
        else:
            self.health.slam_ok = True

    def _on_bt_status(self, msg: MissionStatus):
        """BT execution feedback."""
        self.get_logger().info(f'BT status: {msg.status} (reason={msg.reason})')
        self.health.bt_alive = True
        self.health.last_bt_heartbeat_time = time.time()

        if msg.status == "FAILED":
            # Reinterpret BT failure
            intent, reason = reinterpret_bt_failure(self.mission, self.health)
            self.get_logger().warn(
                f'BT FAILURE reinterpreted → intent={intent}, reason={reason}')
            self._publish_intent(intent, reason)
            return

        if msg.status == "SUCCESS":
            # BT completed current action successfully
            # In our fix, BT sends the successful intent in msg.reason
            self._on_bt_success(msg.reason)
            return

        # Update mission status from BT feedback
        if msg.status in ("APPROACHING", "WAITING_LOAD", "WAITING_UNLOAD"):
            self.mission.status = msg.status

    def _on_bt_success(self, intent: str):
        """BT completed current intent successfully. Advance phase if appropriate."""
        
        # Only certain intents cross mission leg boundaries
        if intent not in ("DELIVER", "REROUTE", "RETURN_HOME"):
            self.get_logger().info(f"Intent {intent} finished, but not advancing phase.")
            return

        current_phase = self.mission.phase

        if current_phase == "PICKUP":
            # Arrived at merchant → wait for loading confirmation
            self.mission.status = "WAITING_LOAD"
            self.get_logger().info('Arrived at pickup, waiting for load confirmation')

        elif current_phase == "DELIVERY":
            # Arrived at customer → wait for unloading confirmation
            self.mission.status = "WAITING_UNLOAD"
            self.get_logger().info('Arrived at dropoff, waiting for delivery confirmation')

        elif current_phase == "RETURN":
            # Arrived home → mission complete
            self.mission.status = "COMPLETED"
            self.get_logger().info(f'Mission {self.mission.mission_id} COMPLETED')
            self._publish_status()
            self.mission.reset()
            return

        self._publish_status()

    # ── Decision Loop ─────────────────────────────────────────────

    def _decision_loop(self):
        """Main decision loop running at fixed rate."""
        if not self.mission.is_active():
            return  # Nothing to do

        # Run intent engine
        intent, reason = decide_intent(
            self.mission, self.health, self.sla, self.user)

        # Consume pending command after processing
        if self.mission.pending_command:
            self.mission.pending_command = ""

        # Publish intent (only if changed, to avoid spamming BT)
        if intent != self._last_intent or reason != self._last_reason:
            self._publish_intent(intent, reason)

        # Handle terminal intents
        if intent == "ABORT":
            self.mission.status = "ABORTED"
            self._publish_status()
            self.mission.reset()
            return

        # Update user perception
        compute_user_perception(self.mission, self.health, self.sla, self.user)
        self._publish_status()

    # ── Publishers ────────────────────────────────────────────────

    def _publish_intent(self, intent: str, reason: str):
        """Publish intent to BT Agent."""
        msg = MissionIntent()
        msg.mission_id = self.mission.mission_id
        msg.intent = intent
        msg.phase = self.mission.phase
        msg.target_x = self.mission.target_x
        msg.target_y = self.mission.target_y
        msg.target_z = self.mission.target_z
        msg.target_marker_id = self.mission.target_marker_id
        msg.reason = reason

        self.intent_pub.publish(msg)
        self._last_intent = intent
        self._last_reason = reason

        self.get_logger().info(
            f'Published intent: {intent} (phase={self.mission.phase}, reason={reason})')

    def _publish_status(self):
        """Publish enriched status to Web."""
        msg = MissionStatus()
        msg.mission_id = self.mission.mission_id
        msg.uav_id = self.mission.uav_id
        msg.status = self.mission.status
        msg.reason = ""
        msg.progress = self.user.progress
        msg.user_facing_msg = self.user.user_facing_msg
        msg.eta_seconds = self.user.eta_seconds

        self.status_pub.publish(msg)

    def _check_geofence(self):
        """Check if UAV is outside safety boundaries."""
        if not self.mission.is_active():
            return

        # Distance from home (local ENU)
        dx = self.health.pos_x - self.mission.home_x
        dy = self.health.pos_y - self.mission.home_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist > self.geofence_radius:
            self.get_logger().error(f"GEOFENCE VIOLATION: Horizontal distance {dist:.1f}m > {self.geofence_radius}m")
            self.mission.pending_command = "RETURN_HOME"
            self.mission.pending_command_reason = "GEOFENCE_HORIZONTAL"

        if self.health.pos_z > self.geofence_max_alt:
            self.get_logger().error(f"GEOFENCE VIOLATION: Altitude {self.health.pos_z:.1f}m > {self.geofence_max_alt}m")
            self.mission.pending_command = "RETURN_HOME"
            self.mission.pending_command_reason = "GEOFENCE_ALTITUDE"

    # ── Coordinate Conversion ────────────────────────────────────

    @staticmethod
    def _gps_to_enu(lat: float, lon: float,
                     ref_lat: float, ref_lon: float) -> tuple:
        """
        Convert GPS (lat, lon) to local ENU (x_east, y_north) in meters.

        Uses Equirectangular approximation — accurate within ~1m for <10km range.
        """
        R = 6371000.0  # Earth radius in meters
        d_lat = math.radians(lat - ref_lat)
        d_lon = math.radians(lon - ref_lon)
        cos_ref = math.cos(math.radians(ref_lat))

        x_east = d_lon * cos_ref * R
        y_north = d_lat * R
        return x_east, y_north


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
