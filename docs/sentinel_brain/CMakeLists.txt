cmake_minimum_required(VERSION 3.8)
project(sentinel_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------
# 1. 查找依赖
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)       # 用于处理 Action Client
find_package(sentinel_interfaces REQUIRED)  # 用于任务和状态回传消息
find_package(flight_core REQUIRED)          # 用于起飞、降落、移动接口

# -------------------------------------------------------------------
# 2. 定义可执行文件
# -------------------------------------------------------------------
add_executable(mission_manager src/mission_manager.cpp)

# -------------------------------------------------------------------
# 3. 链接依赖
# -------------------------------------------------------------------
ament_target_dependencies(mission_manager
        rclcpp
        rclcpp_action
        sentinel_interfaces
        flight_core
)

# -------------------------------------------------------------------
# 4. 安装与打包
# -------------------------------------------------------------------
install(TARGETS mission_manager
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()