#pragma once
#include "types.hpp"
#include <cmath>
#include <limits>

namespace flight_core {

    class FlightFSM {
    public:
        FlightFSM() = default;

        // --- 外部控制接口 (Action / Service 调用) ---

        // 起飞
        bool trigger_takeoff(double height, double current_x, double current_y);

        // 降落
        bool trigger_land();

        // 移动 (Action 调用)：只设置“终点”，不直接改 PX4 目标
        bool trigger_move(double x, double y, double z, double yaw);

        // 悬停/急刹车：必须传入当前位置，以便“锁”在当前点
        bool trigger_hold(const CurrentState& current_pos);

        // --- 周期性更新 (Timer 调用) ---
        // 核心算法在这里：计算 track_target_ 如何一步步逼近 goal_target_
        void update_state(const CurrentState& current_pos);

        // --- Getters ---
        FlightPhase phase() const { return phase_; }

        // 获取的是“影子目标”(用于发给 PX4)，而不是“终点”
        Target track_target() const { return track_target_; }

        // (可选) 如果 Action Feedback 想显示离“终点”还有多远，可以加这个
        Target goal_target() const { return goal_target_; }

    private:
        FlightPhase phase_{FlightPhase::IDLE};

        // 双目标设计
        Target goal_target_;   // 用户想要的终点 (The Goal)
        Target track_target_;  // 实际发给 PX4 的每一步目标 (The Shadow/Carrot)
        
        // 平滑控制状态
        struct Velocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        } current_vel_;
    };

}  // namespace flight_core

