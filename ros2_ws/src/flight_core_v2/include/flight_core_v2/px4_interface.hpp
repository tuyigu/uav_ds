#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "flight_core_v2/types.hpp"
#include <mutex>
#include <limits>

namespace fc2 {

/**
 * @brief Thin wrapper around PX4 uXRCE-DDS topics.
 *
 * Responsibilities:
 *   1. Subscribe to PX4 state (position, velocity, battery).
 *   2. Publish TrajectorySetpoint with position+velocity feedforward.
 *   3. Publish OffboardControlMode (switch between pos+vel / vel-only modes).
 *   4. Send VehicleCommand for arm, disarm, mode changes.
 *
 * NED↔ENU conversion is handled entirely in this class.
 * All external interfaces use ENU (ROS convention).
 */
class PX4Interface {
public:
    explicit PX4Interface(rclcpp::Node* node);

    // ── Setpoint publishing ───────────────────────────────────────
    /// Publish position + velocity feedforward (normal trajectory mode)
    void publishSetpoint(const MotionSetpoint& sp);

    /// Publish Offboard heartbeat (must be called >2 Hz before enabling Offboard)
    /// @param velocity_only  true → OffboardControlMode with position=false, velocity=true
    void publishHeartbeat(bool velocity_only = false);

    // ── Mode / arm commands ───────────────────────────────────────
    void commandArm();
    void commandDisarm();
    void commandSwitchOffboard();
    void commandSwitchLand();

    // ── State read ───────────────────────────────────────────────
    DroneState getState() const;

private:
    void sendVehicleCommand(uint16_t command, float p1 = 0.f, float p2 = 0.f);

    rclcpp::Node* node_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr        bat_sub_;

    mutable std::mutex state_mutex_;
    DroneState state_{};
};

} // namespace fc2
