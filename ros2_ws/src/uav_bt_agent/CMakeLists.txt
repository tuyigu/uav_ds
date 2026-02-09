cmake_minimum_required(VERSION 3.8)
project(uav_bt_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(flight_core REQUIRED)
find_package(uav_web_agent REQUIRED)

include_directories(
  include
)

add_executable(uav_bt_agent_node
  src/main.cpp
  src/plugins/action/takeoff_action.cpp
  src/plugins/action/move_to_action.cpp
  src/plugins/action/land_action.cpp
)

target_include_directories(uav_bt_agent_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(uav_bt_agent_node
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  flight_core
  uav_web_agent
)

install(TARGETS uav_bt_agent_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
