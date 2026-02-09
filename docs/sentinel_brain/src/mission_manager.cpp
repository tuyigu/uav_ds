#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <vector>
#include <chrono>

// 1. 引入接口
#include "sentinel_interfaces/msg/delivery_task.hpp"
#include "sentinel_interfaces/msg/task_feedback.hpp"
#include "flight_core/srv/takeoff.hpp"
#include "flight_core/srv/land.hpp"
#include "flight_core/action/move_to.hpp"

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node {
public:
    using MoveTo = flight_core::action::MoveTo;
    using GoalHandleMoveTo = rclcpp_action::ClientGoalHandle<MoveTo>;

    MissionManager() : Node("mission_manager") {
        // 订阅 Agent 任务
        task_sub_ = create_subscription<sentinel_interfaces::msg::DeliveryTask>(
            "/delivery_task", 10, std::bind(&MissionManager::on_task_received, this, std::placeholders::_1));

        // 发布状态反馈
        feedback_pub_ = create_publisher<sentinel_interfaces::msg::TaskFeedback>("/task_feedback", 10);

        // 控制层服务与 Action 路径
        takeoff_client_ = create_client<flight_core::srv::Takeoff>("/flight/takeoff");
        land_client_ = create_client<flight_core::srv::Land>("/flight/land");
        move_client_ = rclcpp_action::create_client<MoveTo>(this, "/flight/move_to");

        RCLCPP_INFO(get_logger(), "🧠 Brain Node Online. Fixed Takeoff sequence logic.");
    }

private:
    void on_task_received(const sentinel_interfaces::msg::DeliveryTask::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received Mission: %s. Executing sequence...", msg->task_id.c_str());
        // 开启独立线程执行序列，避免阻塞 ROS 执行器
        std::thread([this, msg]() { execute_mission(msg); }).detach();
    }

    void execute_mission(const sentinel_interfaces::msg::DeliveryTask::SharedPtr task) {
        auto tid = task->task_id;

        // --- STEP 1: 起飞 (固定高度 2.0m) ---
        publish_status(tid, 1, "Executing Takeoff (2.0m)...");
        if (!call_takeoff(2.0f)) {
            publish_status(tid, 6, "Takeoff Failed");
            return;
        }

        // ★ 核心修复：状态切换缓冲
        // 因为 FlightCore 在接收到 Takeoff 后会进入 TAKING_OFF 状态，
        // 只有物理到达高度后才会转为 HOLDING。我们需要等待这个物理过程。
        RCLCPP_INFO(get_logger(), "Takeoff command accepted. Waiting for UAV to reach altitude...");
        publish_status(tid, 1, "Waiting for Hover State...");

        // 简单策略：强制等待 5 秒（根据你的 MAX_SPEED_Z 计算，起飞到 2m 约需 3-4s）
        std::this_thread::sleep_for(5s);

        // --- STEP 2: 路径规划 ---
        float cruise_alt = 5.0f;
        std::vector<MoveTo::Goal> sequence = {
            make_goal(task->pickup_x, task->pickup_y, cruise_alt, task->pickup_yaw),
            make_goal(task->pickup_x, task->pickup_y, task->pickup_z, task->pickup_yaw),
            make_goal(task->pickup_x, task->pickup_y, cruise_alt, task->pickup_yaw),
            make_goal(task->dropoff_x, task->dropoff_y, cruise_alt, task->dropoff_yaw),
            make_goal(task->dropoff_x, task->dropoff_y, task->dropoff_z, task->dropoff_yaw)
        };

        // --- STEP 3: 执行 Action 序列 ---
        for (size_t i = 0; i < sequence.size(); ++i) {
            uint8_t stage = (i < 3) ? 1 : 3;
            publish_status(tid, stage, "Navigating to Waypoint " + std::to_string(i+1));

            RCLCPP_INFO(get_logger(), "Sending Action: Step %zu", i+1);

            if (!call_move(sequence[i])) {
                RCLCPP_ERROR(get_logger(), "Step %zu Failed! Logic Aborted.", i+1);
                publish_status(tid, 6, "Mission Aborted: Navigation Rejected");
                call_land();
                return;
            }
            RCLCPP_INFO(get_logger(), "Step %zu reached.", i+1);
        }

        // --- STEP 4: 结束任务 ---
        publish_status(tid, 5, "Mission Successful. Landing...");
        call_land();
    }

    // 同步 Service 调用
    bool call_takeoff(float alt) {
        if (!takeoff_client_->wait_for_service(5s)) return false;

        auto req = std::make_shared<flight_core::srv::Takeoff::Request>();
        req->height = alt;

        auto future = takeoff_client_->async_send_request(req);
        if (future.wait_for(15s) == std::future_status::ready) {
            return future.get()->success;
        }
        return false;
    }

    // 同步 Action 调用
    bool call_move(const MoveTo::Goal& goal) {
        if (!move_client_->wait_for_action_server(5s)) return false;

        auto send_goal_options = rclcpp_action::Client<MoveTo>::SendGoalOptions();
        auto future_goal_handle = move_client_->async_send_goal(goal, send_goal_options);

        if (future_goal_handle.wait_for(10s) != std::future_status::ready) return false;

        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) return false; // 这里就是之前 Reject 的地方

        auto future_result = move_client_->async_get_result(goal_handle);
        if (future_result.wait_for(5min) == std::future_status::ready) {
            auto wrapped_result = future_result.get();
            return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
        }
        return false;
    }

    void call_land() {
        if (land_client_->wait_for_service(2s)) {
            land_client_->async_send_request(std::make_shared<flight_core::srv::Land::Request>());
        }
    }

    void publish_status(std::string id, uint8_t status, std::string msg) {
        auto feedback = sentinel_interfaces::msg::TaskFeedback();
        feedback.task_id = id;
        feedback.status = status;
        feedback.message = msg;
        feedback_pub_->publish(feedback);
    }

    MoveTo::Goal make_goal(float x, float y, float z, float yaw) {
        MoveTo::Goal g; g.x = x; g.y = y; g.z = z; g.yaw = yaw; return g;
    }

    rclcpp::Subscription<sentinel_interfaces::msg::DeliveryTask>::SharedPtr task_sub_;
    rclcpp::Publisher<sentinel_interfaces::msg::TaskFeedback>::SharedPtr feedback_pub_;
    rclcpp::Client<flight_core::srv::Takeoff>::SharedPtr takeoff_client_;
    rclcpp::Client<flight_core::srv::Land>::SharedPtr land_client_;
    rclcpp_action::Client<MoveTo>::SharedPtr move_client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManager>());
    rclcpp::shutdown();
    return 0;
}