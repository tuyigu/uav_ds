#include "flight_core_v2/px4_interface.hpp"
#include <limits>

using namespace px4_msgs::msg;

namespace fc2 {

static constexpr float NAN_F = std::numeric_limits<float>::quiet_NaN();

PX4Interface::PX4Interface(rclcpp::Node* node) : node_(node)
{
    auto qos = rclcpp::QoS(10).best_effort().durability_volatile();

    offboard_pub_ = node_->create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);
    traj_pub_ = node_->create_publisher<TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos);
    cmd_pub_ = node_->create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", qos);

    // Position + velocity subscriber (ENU ← NED conversion inside)
    pos_sub_ = node_->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            if (msg->xy_valid && msg->z_valid) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                // NED → ENU
                state_.position.x() =  msg->y; // East
                state_.position.y() =  msg->x; // North
                state_.position.z() = -msg->z; // Up
                // Velocity NED → ENU
                state_.velocity.x() =  msg->vy;
                state_.velocity.y() =  msg->vx;
                state_.velocity.z() = -msg->vz;
                state_.connected = true;
            }
        });

    bat_sub_ = node_->create_subscription<BatteryStatus>(
        "/fmu/out/battery_status", qos,
        [this](const BatteryStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.battery = msg->remaining;
        });
}

// ─── Heartbeat ──────────────────────────────────────────────────────────────

void PX4Interface::publishHeartbeat(bool velocity_only)
{
    OffboardControlMode msg{};
    msg.timestamp  = node_->get_clock()->now().nanoseconds() / 1000;
    msg.position   = !velocity_only;  // position control when in trajectory mode
    msg.velocity   = true;            // always enable velocity (feedforward or direct)
    msg.acceleration = false;
    msg.attitude   = false;
    msg.body_rate  = false;
    offboard_pub_->publish(msg);
}

// ─── Trajectory setpoint (pos + vel feedforward) ────────────────────────────

void PX4Interface::publishSetpoint(const MotionSetpoint& sp)
{
    if (!sp.valid) return;

    TrajectorySetpoint msg{};
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;

    if (sp.velocity_only) {
        // Velocity-servo mode: NaN position, direct velocity
        msg.position = {NAN_F, NAN_F, NAN_F};
        // ENU → NED
        msg.velocity = {
            static_cast<float>( sp.velocity.y()),
            static_cast<float>( sp.velocity.x()),
            static_cast<float>(-sp.velocity.z())
        };
        msg.yaw      = std::isnan(sp.yaw) ? NAN_F : static_cast<float>(sp.yaw);
        msg.yawspeed = std::isnan(sp.yaw_rate) ? NAN_F : static_cast<float>(sp.yaw_rate);
    } else {
        // Normal trajectory mode: position + velocity feedforward
        // ENU → NED
        msg.position = {
            static_cast<float>( sp.position.y()),
            static_cast<float>( sp.position.x()),
            static_cast<float>(-sp.position.z())
        };
        msg.velocity = {
            static_cast<float>( sp.velocity.y()),
            static_cast<float>( sp.velocity.x()),
            static_cast<float>(-sp.velocity.z())
        };
        msg.acceleration = {NAN_F, NAN_F, NAN_F};
        msg.jerk         = {NAN_F, NAN_F, NAN_F};
        msg.yaw      = std::isnan(sp.yaw) ? NAN_F : static_cast<float>(sp.yaw);
        msg.yawspeed = NAN_F;
    }

    traj_pub_->publish(msg);
}

// ─── Commands ───────────────────────────────────────────────────────────────

void PX4Interface::commandArm()
{
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void PX4Interface::commandDisarm()
{
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void PX4Interface::commandSwitchOffboard()
{
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void PX4Interface::commandSwitchLand()
{
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

void PX4Interface::sendVehicleCommand(uint16_t command, float p1, float p2)
{
    VehicleCommand msg{};
    msg.timestamp        = node_->get_clock()->now().nanoseconds() / 1000;
    msg.command          = command;
    msg.param1           = p1;
    msg.param2           = p2;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    cmd_pub_->publish(msg);
}

DroneState PX4Interface::getState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

} // namespace fc2
