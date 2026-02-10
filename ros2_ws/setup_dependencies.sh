#!/bin/bash
set -e

echo "Setting up external dependencies..."

mkdir -p src

# 1. BehaviorTree.CPP v4
if [ ! -d "src/BehaviorTree.CPP" ]; then
    echo "Cloning BehaviorTree.CPP..."
    git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git src/BehaviorTree.CPP
else
    echo "Updating BehaviorTree.CPP..."
    cd src/BehaviorTree.CPP && git pull && cd ../..
fi

# 2. FAST_LIO_ROS2
if [ ! -d "src/FAST_LIO_ROS2" ]; then
    echo "Cloning FAST_LIO_ROS2..."
    git clone https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO_ROS2
    # Note: Might need to checkout specific branch depending on ROS 2 version
else
    echo "Updating FAST_LIO_ROS2..."
    cd src/FAST_LIO_ROS2 && git pull && cd ../..
fi

# 3. Groot2
mkdir -p tools
if [ ! -f "tools/Groot2.AppImage" ]; then
    echo "Downloading Groot2..."
    wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage -O tools/Groot2.AppImage
    chmod +x tools/Groot2.AppImage
fi

echo "Dependencies ready. Run 'colcon build' to compile."
