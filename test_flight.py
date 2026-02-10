#!/usr/bin/env python3
"""
Simple UAV test flight script to demonstrate SLAM and OctoMap.
Sends the UAV on a square pattern at two altitudes to build a 3D map.

Usage:
    python3 test_flight.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus


class SimpleFlightController(Node):
    def __init__(self):
        super().__init__('simple_flight_controller')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos_profile)
        
        self.vehicle_status = VehicleStatus()
        self.counter = 0
        self.armed = False
        self.offboard = False
        self.waypoint_index = 0
        self.waypoint_counter = 0  # counts iterations at current waypoint
        
        # Waypoints (NED: x-forward, y-right, z-down)
        self.waypoints = [
            (0.0, 0.0, -3.0),    # Takeoff to 3m
            (8.0, 0.0, -3.0),    # Forward 8m
            (8.0, 8.0, -3.0),    # Right 8m
            (0.0, 8.0, -3.0),    # Back 8m
            (0.0, 0.0, -3.0),    # Return to start
            (0.0, 0.0, -6.0),    # Climb to 6m
            (8.0, 0.0, -6.0),    # Forward at 6m
            (8.0, 8.0, -6.0),    # Right
            (0.0, 8.0, -6.0),    # Back
            (0.0, 0.0, -6.0),    # Return
            (0.0, 0.0, -3.0),    # Descend to 3m
        ]
        
        self.dwell_time = 400  # 8 seconds at 50Hz
        self.timer = self.create_timer(0.02, self.timer_cb)
        
        self.get_logger().info('Flight Controller ready - will fly 8m square at 3m and 6m')
    
    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg
    
    def send_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
    
    def send_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
    
    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
    
    def timer_cb(self):
        # Always send offboard heartbeat + first waypoint position
        self.send_offboard_mode()
        
        # Phase 1: Send setpoints for a bit before arming (PX4 requirement)
        if self.counter < 50:
            self.send_position(0.0, 0.0, -3.0)
            self.counter += 1
            return
        
        # Phase 2: Arm
        if not self.armed:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('Arming...')
            self.armed = True
            self.send_position(0.0, 0.0, -3.0)
            self.counter += 1
            return
        
        # Phase 3: Switch to offboard
        if not self.offboard:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info('Offboard mode...')
            self.offboard = True
            self.send_position(0.0, 0.0, -3.0)
            self.counter += 1
            return
        
        # Phase 4: Fly waypoints
        if self.waypoint_index < len(self.waypoints):
            wp = self.waypoints[self.waypoint_index]
            self.send_position(wp[0], wp[1], wp[2])
            self.waypoint_counter += 1
            
            if self.waypoint_counter >= self.dwell_time:
                self.waypoint_counter = 0
                self.waypoint_index += 1
                if self.waypoint_index < len(self.waypoints):
                    nwp = self.waypoints[self.waypoint_index]
                    self.get_logger().info(
                        f'[{self.waypoint_index+1}/{len(self.waypoints)}] '
                        f'Going to ({nwp[0]}, {nwp[1]}, {nwp[2]})')
                else:
                    self.get_logger().info('All waypoints complete! Landing...')
        else:
            # Land
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info('Landing...')
            self.timer.cancel()
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFlightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
