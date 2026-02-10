cmake_minimum_required(VERSION 3.8)
project(flight_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/UavState.msg"
  "msg/ArucoMarker.msg"
  "msg/ArucoMarkers.msg"
)

set(ACTION_FILES
  "action/MoveTo.action"
  "action/Takeoff.action"
  "action/Land.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${ACTION_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

include_directories(
  include
)

add_library(${PROJECT_NAME}_core
  src/flight_core.cpp
  src/flight_fsm.cpp
  src/px4_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_action
  px4_msgs
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(flight_core_node src/main.cpp)
target_link_libraries(flight_core_node ${PROJECT_NAME}_core)
ament_target_dependencies(flight_core_node
  rclcpp
  rclcpp_action
  px4_msgs
)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  flight_core_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_action px4_msgs)

ament_package()

