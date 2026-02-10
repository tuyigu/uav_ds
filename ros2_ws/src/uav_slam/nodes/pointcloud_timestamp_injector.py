#!/usr/bin/env python3
"""
Point Cloud Timestamp Injector Node

Gazebo's LiDAR sensor does not provide per-point timestamps in PointCloud2.
FAST-LIO requires a 'time' field for each point to perform motion undistortion.
This node subscribes to the raw point cloud, adds a synthetic 'time' field
(offset from scan start), and republishes.

For a spinning LiDAR like Velodyne 16, points are collected over one full
rotation (~100ms at 10Hz). We assign timestamps linearly based on point
index within the scan.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct


class PointCloudTimestampInjector(Node):
    def __init__(self):
        super().__init__('pointcloud_timestamp_injector')

        # Declare parameters
        self.declare_parameter('scan_period', 0.1)  # 10Hz LiDAR scan rate
        self.scan_period = self.get_parameter('scan_period').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar_points/points',
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/lidar_points/points_stamped',
            qos
        )

        self.get_logger().info(
            f'PointCloud Timestamp Injector started (scan_period={self.scan_period}s)')

    def callback(self, msg: PointCloud2):
        """Add 'time' field to each point in the cloud."""

        # Parse existing fields
        # We need to append a float32 'time' field
        existing_point_step = msg.point_step
        new_point_step = existing_point_step + 4  # add 4 bytes for float32 time

        num_points = msg.width * msg.height
        if num_points == 0:
            return

        # Create new fields list with 'time' appended
        new_fields = list(msg.fields)
        time_field = PointField()
        time_field.name = 'time'
        time_field.offset = existing_point_step
        time_field.datatype = PointField.FLOAT32
        time_field.count = 1
        new_fields.append(time_field)

        # Build new data buffer
        old_data = bytes(msg.data)
        new_data = bytearray(num_points * new_point_step)

        # Calculate time offset for each point
        # Velodyne 16 fires in order, so time offset is linear
        scan_duration_ms = self.scan_period * 1000.0  # in milliseconds

        for i in range(num_points):
            # Copy existing point data
            old_start = i * existing_point_step
            new_start = i * new_point_step
            new_data[new_start:new_start + existing_point_step] = \
                old_data[old_start:old_start + existing_point_step]

            # Add time offset (linear distribution over scan period, in seconds)
            time_offset = (float(i) / float(num_points)) * self.scan_period
            struct.pack_into('f', new_data, new_start + existing_point_step, time_offset)

        # Create new message
        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.fields = new_fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = new_point_step
        out_msg.row_step = new_point_step * msg.width
        out_msg.data = new_data
        out_msg.is_dense = msg.is_dense

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTimestampInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
