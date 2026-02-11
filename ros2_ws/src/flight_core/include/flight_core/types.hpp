#pragma once
#include <limits> // for quiet_NaN

namespace flight_core {

    // 定义飞行阶段
    enum class FlightPhase {
        IDLE,
        TAKING_OFF,
        HOLDING,
        MOVING,
        LANDING,
        LANDED
    };

    // 发送给 PX4 的目标 / FSM 内部计算的目标 (ROS ENU系)
    struct Target {
        double x;     // East
        double y;     // North
        double z;     // Up (高度为正)
        double yaw;   
    };

    // 从 PX4 获取并转换为 ENU 后的当前状态
    struct CurrentState {
        double x;     // East
        double y;     // North
        double z;     // Up
        double yaw;
        float battery; // 电量百分比 0.0 - 1.0
        bool connected;
        bool armed;
    };

} // namespace flight_core

