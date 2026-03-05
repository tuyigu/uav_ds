#!/bin/bash
# 启动 Web 后端 (FastAPI)
# 请确保先在一个终端运行 ros2 launch uav_bringup uav_system.launch.py

source ros2_ws/install/setup.bash
cd web_backend

echo "Starting Web Backend on http://0.0.0.0:8000 ..."
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
