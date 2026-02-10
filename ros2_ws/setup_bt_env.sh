#!/bin/bash
set -e

echo "Received request to setup BehaviorTree.CPP v4 and Groot2..."

# 1. Install Dependencies
sudo apt-get update
sudo apt-get install -y libzmq3-dev libdw-dev qtbase5-dev libqt5svg5-dev libqt5opengl5-dev

# 2. Clone BehaviorTree.CPP v4 into workspace
echo "Cloning BehaviorTree.CPP v4..."
cd src
if [ ! -d "BehaviorTree.CPP" ]; then
    git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
    cd BehaviorTree.CPP
    git checkout master # v4 is on master now, or check generic v4 tag
    # Verify it is v4 from CMakeLists or similar, but master provides v4.
    cd ..
else
    echo "BehaviorTree.CPP already exists. Pulling latest..."
    cd BehaviorTree.CPP
    git pull
    cd ..
fi

# 3. Clone Groot2 (Binary or Source?)
# Groot2 source is closed? No, Groot2 is commercial but free for non-commercial?
# Wait, Groot (v1) is open source. Groot2 is from BehaviorTree.CPP team.
# The user asked for Groot2. Groot2 Linux installer is available from website.
# But we can also use Groot (v1) compatible with v4?
# Actually, BT v4 works best with Groot2.
# Groot2 is an AppImage generally.

echo "Downloading Groot2 AppImage..."
mkdir -p ../tools
cd ../tools
if [ ! -f "Groot2.AppImage" ]; then
    wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage -O Groot2.AppImage
    chmod +x Groot2.AppImage
fi
cd ../src

# 4. Build Workspace (BT CPP will be built as a package)
echo "Building workspace..."
cd ..
colcon build --packages-select behaviortree_cpp --symlink-install

# 5. Build Agent
colcon build --packages-select uav_bt_agent --symlink-install

echo "Setup Complete! Run './tools/Groot2.AppImage' to visualize."
