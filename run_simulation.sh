#!/usr/bin/env zsh

# Configuration
WORKSPACE_DIR="$HOME/Dev/Robotics/workspaces/uav_ds"
PX4_DIR="$HOME/Dev/Robotics/firmwares/PX4-Autopilot"

# 1. Source ROS 2 environment
source /opt/ros/humble/setup.zsh
cd $WORKSPACE_DIR/ros2_ws
colcon build --packages-select uav_simulation
source install/setup.zsh

# 2. Setup Gazebo Resource Path
# Includes: ~/.gz/models_flat (Fuel), uav_simulation, AND PX4 default paths
export GZ_SIM_RESOURCE_PATH="$HOME/.gz/models_flat:$WORKSPACE_DIR/ros2_ws/src/uav_simulation/models:$WORKSPACE_DIR/ros2_ws/src/uav_simulation/worlds:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

echo "Starting Simulation..."
echo "Model: x500_delivery"
echo "World: urban_delivery"

# 3. Launch PX4 SITL
cd $PX4_DIR
PX4_GZ_MODEL=x500_delivery PX4_GZ_WORLD=urban_delivery ./build/px4_sitl_default/bin/px4

# Note: In a separate terminal, run:
# ros2 launch uav_simulation simulation.launch.py
