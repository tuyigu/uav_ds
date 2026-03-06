cmake_minimum_required(VERSION 3.14)
project(flight_core_v2)

# ── C++ standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -O2)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(flight_core REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Custom action/msg generation ──────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowPath.action"
  "action/VelocityServo.action"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ── Trajectory math library (no ROS deps, easy to unit-test) ─────────────────
add_library(trajectory_lib STATIC
  src/trajectory/spline_path.cpp
  src/trajectory/scurve_planner.cpp
  src/trajectory/trajectory.cpp
  src/trajectory/trajectory_sampler.cpp
)
target_include_directories(trajectory_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_lib
  Eigen3::Eigen
)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(flight_core_v2_node
  src/main.cpp
  src/flight_core_v2.cpp
  src/motion_manager.cpp
  src/px4_interface.cpp
)
target_include_directories(flight_core_v2_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(flight_core_v2_node
  trajectory_lib
  Eigen3::Eigen
)
ament_target_dependencies(flight_core_v2_node
  rclcpp
  rclcpp_action
  px4_msgs
  geometry_msgs
  nav_msgs
  flight_core
)

# Link generated action types to executable
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(flight_core_v2_node "${cpp_typesupport_target}")

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS flight_core_v2_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclcpp_action px4_msgs geometry_msgs nav_msgs flight_core Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
