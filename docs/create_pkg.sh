#!/bin/bash

# --- ⚙️ 配置区域 (以后只改这里！) ---
PKG_NAME="ros2_robot_arm"      # 项目包名
NODE_NAME="arm_controller"     # 节点可执行文件名
CLASS_NAME="ArmController"     # C++ 类名
# --------------------------------

# 进入 src (假设你在工作空间根目录运行)
mkdir -p src
cd src

# 创建目录结构
mkdir -p ${PKG_NAME}/{config,launch,include/${PKG_NAME},src}

# --- 1. package.xml ---
cat > ${PKG_NAME}/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${PKG_NAME}</name>
  <version>0.1.0</version>
  <description>Auto-generated ROS 2 Package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>px4_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# --- 2. CMakeLists.txt ---
cat > ${PKG_NAME}/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(${PKG_NAME})

# 自动生成编译数据库
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

# 主节点
add_executable(${NODE_NAME} src/main_node.cpp)
target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:\${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${NODE_NAME} rclcpp px4_msgs)

# 安装规则
install(TARGETS ${NODE_NAME}
  DESTINATION lib/\${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/\${PROJECT_NAME})

ament_package()
EOF

# --- 3. 创建一个简单的 main_node.cpp 模板 ---
cat > ${PKG_NAME}/src/main_node.cpp << EOF
#include <rclcpp/rclcpp.hpp>

class ${CLASS_NAME} : public rclcpp::Node {
public:
    ${CLASS_NAME}() : Node("${NODE_NAME}") {
        RCLCPP_INFO(this->get_logger(), "🚀 ${CLASS_NAME} Started!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<${CLASS_NAME}>());
    rclcpp::shutdown();
    return 0;
}
EOF

echo "✅ 项目 ${PKG_NAME} 创建完毕！"