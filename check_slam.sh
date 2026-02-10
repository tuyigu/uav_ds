#!/bin/bash
# SLAM Diagnostics Script
# Checks the health of SLAM-related topics

echo "=== SLAM Diagnostics ==="
echo ""

echo "1. Checking IMU topic:"
ros2 topic hz /imu --window 10

echo ""
echo "2. Checking LiDAR point cloud:"
ros2 topic hz /lidar_points/points --window 10

echo ""
echo "3. Checking SLAM odometry:"
ros2 topic hz /Odometry --window 10

echo ""
echo "4. Checking SLAM point cloud output:"
ros2 topic hz /cloud_registered --window 10

echo ""
echo "5. Sample IMU data (first message):"
timeout 2 ros2 topic echo /imu --once

echo ""
echo "6. Check TF tree:"
ros2 run tf2_tools view_frames
echo "TF tree saved to frames.pdf"
