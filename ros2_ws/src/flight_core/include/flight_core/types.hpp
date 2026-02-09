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

    // 发送给 PX4 的目标 / FSM 内部计算的目标
    struct Target {
        double x;
        double y;
        double z;     // PX4 NED系，高度为负
        double yaw;   // 必须包含 yaw
    };

    // 从 PX4 获取的当前状态
    struct CurrentState {
        double x;
        double y;
        double z;
        double yaw;   // 必须包含 yaw
        bool connected;
        bool armed;
    };

} // namespace flight_core

