cmake_minimum_required(VERSION 3.8)
project(sentinel_agent)

# -------------------------------------------------------------------
# 1. 查找依赖
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sentinel_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# 查找 Curl (Ubuntu 22.04 推荐写法)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)

# -------------------------------------------------------------------
# 2. 定义可执行文件
# -------------------------------------------------------------------
add_executable(agent_node src/AgentNode.cpp)

# 包含 Curl 路径
target_include_directories(agent_node PRIVATE
        ${CURL_INCLUDE_DIRS}
)

# -------------------------------------------------------------------
# 3. 链接依赖 (核心修复点)
# -------------------------------------------------------------------

# 链接第三方非 ROS 库
target_link_libraries(agent_node
        ${CURL_LIBRARIES}
        nlohmann_json::nlohmann_json
)

# 使用 ament 工具链接 ROS 2 依赖
# 它会自动找到 sentinel_interfaces__rosidl_typesupport_cpp
ament_target_dependencies(agent_node
        rclcpp
        px4_msgs
        sentinel_interfaces
)

# -------------------------------------------------------------------
# 4. 安装
# -------------------------------------------------------------------
install(TARGETS agent_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()