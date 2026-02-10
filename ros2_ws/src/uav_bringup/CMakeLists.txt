cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS nodes/tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME px4_tf_broadcaster)

ament_package()
