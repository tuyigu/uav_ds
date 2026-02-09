cmake_minimum_required(VERSION 3.8)
project(sentinel_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 声明所有消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DeliveryTask.msg"
        "msg/TaskFeedback.msg"  # <--- [必须添加这一行]
)

ament_package()