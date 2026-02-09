#include "flight_core/flight_fsm.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace flight_core {

// --- 物理限制参数 ---
// 稍微调大一点速度，让现象更明显
static constexpr double MAX_SPEED_XY = 1.0; // m/s
static constexpr double MAX_SPEED_Z  = 0.7; // m/s
static constexpr double MAX_YAW_RATE = 0.5; // rad/s
static constexpr double DT           = 0.05; // 50ms

// 辅助：计算两点平面距离
static double dist_sq_xy(const Target& a, const Target& b) {
    return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2);
}

// ---------------------------------------------------------
// 起飞
// ---------------------------------------------------------
bool FlightFSM::trigger_takeoff(double height, double current_x, double current_y) {
    if (phase_ != FlightPhase::IDLE) return false;

    // 起飞时，目标设为当前位置的正上方
    goal_target_.x = current_x;
    goal_target_.y = current_y;
    goal_target_.z = -std::abs(height);
    goal_target_.yaw = std::numeric_limits<double>::quiet_NaN();

    // 影子目标直接跟上
    track_target_ = goal_target_;
    
    // 重置速度状态
    current_vel_ = {0.0, 0.0, 0.0};

    phase_ = FlightPhase::TAKING_OFF;
    return true;
}

// ---------------------------------------------------------
// 降落
// ---------------------------------------------------------
bool FlightFSM::trigger_land() {
    if (phase_ == FlightPhase::IDLE || phase_ == FlightPhase::LANDED) return false;
    phase_ = FlightPhase::LANDING;
    return true;
}

// ---------------------------------------------------------
// 移动
// ---------------------------------------------------------
bool FlightFSM::trigger_move(double x, double y, double z, double yaw) {
    if (phase_ != FlightPhase::HOLDING && phase_ != FlightPhase::MOVING) {
        return false;
    }

    // 设置终点
    goal_target_.x = x;
    goal_target_.y = y;
    goal_target_.z = -std::abs(z);
    goal_target_.yaw = yaw;

    // 不重置 track_target_，保证轨迹连续
    phase_ = FlightPhase::MOVING;
    return true;
}

// ---------------------------------------------------------
// 悬停 / 刹车
// ---------------------------------------------------------
bool FlightFSM::trigger_hold(const CurrentState& current_pos) {
    if (phase_ == FlightPhase::MOVING || phase_ == FlightPhase::TAKING_OFF) {
        // 刹车时，把影子目标强行拉回当前物理位置
        track_target_.x = current_pos.x;
        track_target_.y = current_pos.y;
        track_target_.z = current_pos.z;
        track_target_.yaw = current_pos.yaw;

        track_target_.yaw = current_pos.yaw;

        goal_target_ = track_target_;
        phase_ = FlightPhase::HOLDING;
        current_vel_ = {0.0, 0.0, 0.0}; // 刹车清零
        return true;
    }
    return false;
}

// ---------------------------------------------------------
// 核心：状态更新与“虚拟兔子”跑动逻辑
// ---------------------------------------------------------
void FlightFSM::update_state(const CurrentState& cur) {

    // 1. 起飞逻辑 (保持简单，使用位置判定)
    if (phase_ == FlightPhase::TAKING_OFF) {
        if (std::abs(cur.z - goal_target_.z) < 0.2) {
            phase_ = FlightPhase::HOLDING;
            track_target_ = goal_target_;
        }
        return;
    }

    // 2. 移动逻辑 (影子轨迹生成)
    if (phase_ == FlightPhase::MOVING) {

        // 计算：影子目标 (Rabbit) -> 终点 (Goal) 的向量
        // 使用 track_target_，让“虚拟兔子”自己在前面跑
        double dx = goal_target_.x - track_target_.x;
        double dy = goal_target_.y - track_target_.y;
        double dz = goal_target_.z - track_target_.z;

        double dist_rem_xy = std::sqrt(dx*dx + dy*dy);
        double dist_rem_z  = std::abs(dz);

        // 判定物理位置是否已接近终点
        double dist_phys_to_goal = std::sqrt(
            std::pow(goal_target_.x - cur.x, 2) +
            std::pow(goal_target_.y - cur.y, 2) +
            std::pow(goal_target_.z - cur.z, 2)
        );

        if (dist_phys_to_goal < 0.2) {
            // 物理上也到了，切回悬停
            phase_ = FlightPhase::HOLDING;
            track_target_ = goal_target_; // 锁死
            return;
        }

        // B. 平滑速度控制 (XY)
        // 1. 计算期望速度 (P控制: 越远越快，但也受最大速度限制)
        double kP = 1.0; 
        double des_vel_x = dx * kP;
        double des_vel_y = dy * kP;

        // 饱和限制
        double des_speed_xy = std::sqrt(des_vel_x*des_vel_x + des_vel_y*des_vel_y);
        if (des_speed_xy > MAX_SPEED_XY) {
            des_vel_x = (des_vel_x / des_speed_xy) * MAX_SPEED_XY;
            des_vel_y = (des_vel_y / des_speed_xy) * MAX_SPEED_XY;
        }

        // 2. 加速度限制 (Ramp)
        double max_accel_xy = 0.5; // m/s^2
        double max_delta_v = max_accel_xy * DT;

        double diff_vx = des_vel_x - current_vel_.x;
        double diff_vy = des_vel_y - current_vel_.y;
        double diff_v_norm = std::sqrt(diff_vx*diff_vx + diff_vy*diff_vy);

        if (diff_v_norm > max_delta_v) {
            current_vel_.x += (diff_vx / diff_v_norm) * max_delta_v;
            current_vel_.y += (diff_vy / diff_v_norm) * max_delta_v;
        } else {
            current_vel_.x = des_vel_x;
            current_vel_.y = des_vel_y;
        }

        // 3. 位置积分
        track_target_.x += current_vel_.x * DT;
        track_target_.y += current_vel_.y * DT;

        // C. 平滑速度控制 (Z)
        double des_vel_z = dz * kP;
        double des_speed_z = std::abs(des_vel_z);
        if (des_speed_z > MAX_SPEED_Z) {
            des_vel_z = (des_vel_z / des_speed_z) * MAX_SPEED_Z;
        }
        
        double max_accel_z = 0.3; 
        double max_delta_vz = max_accel_z * DT;
        
        double diff_vz = des_vel_z - current_vel_.z;
        if (std::abs(diff_vz) > max_delta_vz) {
            current_vel_.z += std::copysign(max_delta_vz, diff_vz);
        } else {
            current_vel_.z = des_vel_z;
        }
        
        track_target_.z += current_vel_.z * DT;

        // [安全牵引绳] Leash Logic
        double rabbit_dist_to_plane = std::sqrt(
            std::pow(track_target_.x - cur.x, 2) +
            std::pow(track_target_.y - cur.y, 2)
        );

        if (rabbit_dist_to_plane > 2.0) {
            // 此处留空，必要时可以添加“拉回兔子”的逻辑
        }

        // D. Yaw 处理
        if (!std::isnan(goal_target_.yaw)) {
             double err_yaw = goal_target_.yaw - track_target_.yaw;
             double step_yaw = MAX_YAW_RATE * DT;
             if (std::abs(err_yaw) > step_yaw) {
                 track_target_.yaw += std::copysign(step_yaw, err_yaw);
             } else {
                 track_target_.yaw = goal_target_.yaw;
             }
        } else {
            track_target_.yaw = std::numeric_limits<double>::quiet_NaN();
        }
    }
}

} // namespace flight_core

