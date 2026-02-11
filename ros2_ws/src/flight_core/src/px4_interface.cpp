#include "flight_core/px4_interface.hpp"
#include <limits> // 用于获取 NaN

using namespace px4_msgs::msg;

namespace flight_core {

PX4Interface::PX4Interface(rclcpp::Node* node) : node_(node) {
    auto qos = rclcpp::QoS(10).best_effort().durability_volatile();

    offboard_pub_ = node_->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
    traj_pub_ = node_->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
    cmd_pub_ = node_->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

    pos_sub_ = node_->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            // 只有当数据有效时才更新，并转换为 ENU (ROS 标准)
            if (msg->xy_valid && msg->z_valid) {
                // NED -> ENU 转换
                // NED X (North) -> ENU Y
                // NED Y (East) -> ENU X
                // NED Z (Down) -> ENU Z (Up, flipped)
                current_state_.x = msg->y;
                current_state_.y = msg->x;
                current_state_.z = -msg->z;
                current_state_.connected = true;
            }
        });

    battery_sub_ = node_->create_subscription<BatteryStatus>(
        "/fmu/out/battery_status", qos,
        [this](const BatteryStatus::SharedPtr msg) {
            current_state_.battery = msg->remaining;
        });
}

// 心跳包：明确告诉 PX4 我们只控制位置
void PX4Interface::publish_heartbeat() {
    OffboardControlMode msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;

    msg.position = true;      // 启用位置控制
    msg.velocity = false;     // 禁用速度控制
    msg.acceleration = false; // 禁用加速度控制
    msg.attitude = false;     // 禁用姿态控制
    msg.body_rate = false;    // 禁用角速度控制

    offboard_pub_->publish(msg);
}

// 轨迹设定：使用 NaN 解锁 PX4 内部规划器
void PX4Interface::set_trajectory(const Target& target) {
    TrajectorySetpoint msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;

    // 1. 位置设定 (ENU -> NED 转换)
    // ENU X (East) -> NED Y
    // ENU Y (North) -> NED X
    // ENU Z (Up) -> NED Z (Down, flipped)
    msg.position = {
        static_cast<float>(target.y),
        static_cast<float>(target.x),
        static_cast<float>(-target.z)
    };

    // 2. 偏航角设定
    if (std::isnan(target.yaw)) {
        msg.yaw = std::numeric_limits<float>::quiet_NaN();
    } else {
        msg.yaw = static_cast<float>(target.yaw);
    }

    // 3. 将速度、加速度、Jerk 设为 NaN，避免与 PX4 内部控制器冲突
    const float nan_val = std::numeric_limits<float>::quiet_NaN();

    msg.velocity = {nan_val, nan_val, nan_val};
    msg.acceleration = {nan_val, nan_val, nan_val};
    msg.jerk = {nan_val, nan_val, nan_val};

    traj_pub_->publish(msg);
}

void PX4Interface::command_switch_offboard() {
    // 1.0 = enable, 6.0 = OFFBOARD mode
    send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
}

void PX4Interface::command_switch_land() {
    send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

void PX4Interface::command_arm() {
    send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void PX4Interface::command_disarm() {
    send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void PX4Interface::send_vehicle_command(uint16_t command, float param1, float param2) {
    VehicleCommand msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    cmd_pub_->publish(msg);
}

CurrentState PX4Interface::get_state() const { return current_state_; }

}  // namespace flight_core

