cmake_minimum_required(VERSION 3.8)
project(px4_tf_broadcaster)

# 显式指定 C++17，保证现代特性可用
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# Dependencies
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
# 删除了独立的 tf2，因为 tf2_ros 已包含
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# ======================
# Build node
# ======================
add_executable(px4_tf_broadcaster_node
        src/px4_tf_broadcaster.cpp
)

# 包含头文件路径（虽然 ament 自动处理，但在某些 IDE 如 CLion 中显式包含更友好）
target_include_directories(px4_tf_broadcaster_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_tf_broadcaster_node
        rclcpp
        px4_msgs
        tf2_ros
        nav_msgs
        geometry_msgs
)

# ======================
# Install
# ======================
install(TARGETS
        px4_tf_broadcaster_node
        DESTINATION lib/${PROJECT_NAME}
)
# 安装 Launch 文件夹到 share 目录
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)
ament_package()