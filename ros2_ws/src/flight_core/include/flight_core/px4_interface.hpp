#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include "types.hpp"

namespace flight_core {

    class PX4Interface {
    public:
        explicit PX4Interface(rclcpp::Node* node);

        // --- 发送指令 ---

        // 发送 Offboard 心跳 (告诉 PX4 我们要控什么)
        void publish_heartbeat();

        // 发送轨迹设定点 (核心控制函数，输入预期为 ENU)
        void set_trajectory(const Target& target);

        // 模式切换命令
        void command_arm();
        void command_disarm();
        void command_switch_offboard();
        void command_switch_land();

        // --- 获取数据 ---
        CurrentState get_state() const;

    private:
        void send_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

        rclcpp::Node* node_;

        // Publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

        // Subscriptions
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
        rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;

        CurrentState current_state_;
    };

}  // namespace flight_core

