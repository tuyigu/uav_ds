cmake_minimum_required(VERSION 3.8)
project(uav_web_agent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flight_core REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/DeliveryMission.msg"
  "msg/MissionStatus.msg"
  "msg/MissionIntent.msg"
  "msg/OperatorCommand.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces
)

include_directories(
  include
)

add_executable(uav_web_agent_node
  src/web_agent_node.cpp
  src/main.cpp
)

ament_target_dependencies(uav_web_agent_node
  rclcpp
  flight_core
  rclcpp_action
  px4_msgs
)

target_include_directories(uav_web_agent_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS uav_web_agent_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(uav_web_agent_node "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

ament_package()

