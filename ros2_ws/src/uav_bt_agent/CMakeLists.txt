cmake_minimum_required(VERSION 3.8)
project(uav_bt_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(flight_core REQUIRED)
find_package(uav_navigation REQUIRED)
find_package(uav_web_agent REQUIRED)

include_directories(include)

# BT Plugin Library (Shared Object)
add_library(bt_plugins SHARED
  src/plugins/action/takeoff_action.cpp
  src/plugins/action/land_action.cpp
  src/plugins/action/move_to_action.cpp
  src/plugins/action/plan_path_action.cpp
  src/plugins/action/follow_path_action.cpp
  src/plugins/action/search_marker_action.cpp
  src/plugins/action/precision_land_action.cpp
  # Condition nodes...
)
ament_target_dependencies(bt_plugins
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  flight_core
  uav_navigation
)

# Main Executable
add_executable(bt_agent_node src/main.cpp)
target_link_libraries(bt_agent_node bt_plugins)
ament_target_dependencies(bt_agent_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  flight_core
  uav_navigation
  uav_web_agent
)

# Install
install(TARGETS bt_plugins bt_agent_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
