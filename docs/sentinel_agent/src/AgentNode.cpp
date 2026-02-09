#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sentinel_interfaces/msg/delivery_task.hpp>
#include <sentinel_interfaces/msg/task_feedback.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class AgentNode : public rclcpp::Node {
public:
    AgentNode() : Node("sentinel_agent"), running_(true) {

        // 1. 订阅 PX4 状态 (使用 SensorDataQoS 以匹配 PX4 默认发布)
        battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status_v1", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mtx_);
                battery_ = msg->remaining;
            });

        pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mtx_);
                // PX4 使用 NED (北东地)，这里存原始值，上报时再处理
                current_x_ = msg->x;
                current_y_ = msg->y;
                current_z_ = msg->z;
            });

        // 2. 订阅来自 Brain 的业务反馈
        feedback_sub_ = create_subscription<sentinel_interfaces::msg::TaskFeedback>(
            "/task_feedback", 10,
            [this](const sentinel_interfaces::msg::TaskFeedback::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mtx_);
                current_task_id_ = msg->task_id;
                current_status_ = msg->status;
                current_message_ = msg->message;
            });

        // 3. 发布任务给 Brain
        task_pub_ = create_publisher<sentinel_interfaces::msg::DeliveryTask>("/delivery_task", 10);

        // 4. 网络线程
        net_thread_ = std::thread(&AgentNode::network_loop, this);
        RCLCPP_INFO(get_logger(), "Sentinel Agent Online (Humble). Communicating with Web...");
    }

    ~AgentNode() override {
        running_ = false;
        if (net_thread_.joinable()) net_thread_.join();
    }

private:
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s) {
        size_t total = size * nmemb;
        s->append((char*)contents, total);
        return total;
    }

    void network_loop() {
        CURL *curl = curl_easy_init();
        if (!curl) return;

        while (running_) {
            // 构造状态快照
            json status_json;
            {
                std::lock_guard<std::mutex> lock(data_mtx_);
                status_json = {
                    {"drone_id", "sentinel_01"},
                    {"timestamp", std::time(nullptr)},
                    {"body", {
                        {"battery_pct", battery_},
                        {"position", {
                            {"x", current_x_},
                            {"y", current_y_},
                            {"z", -current_z_} // NED to ENU: Z轴取反得到海拔高度
                        }}
                    }},
                    {"mission", {
                        {"current_task_id", current_task_id_},
                        {"stage", current_status_},
                        {"message", current_message_}
                    }}
                };
            }

            std::string request_body = status_json.dump();
            std::string response_string;

            struct curl_slist *headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8000/status");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 800L);

            CURLcode res = curl_easy_perform(curl);
            curl_slist_free_all(headers); // 及时释放每一轮的 headers

            if (res == CURLE_OK) {
                try {
                    auto res_j = json::parse(response_string);
                    if (res_j.contains("new_task") && !res_j["new_task"].is_null()) {
                        auto t = res_j["new_task"];
                        std::string t_id = t.value("task_id", "");

                        if (!t_id.empty() && t_id != last_dispatched_task_id_) {
                            auto msg = sentinel_interfaces::msg::DeliveryTask();
                            msg.task_id = t_id;
                            msg.created_at = t.value("created_at", 0.0);

                            // 解析坐标 (增加默认值防止 null 导致崩掉)
                            auto p = t["pickup"];
                            msg.pickup_x = p.value("x", 0.0f);
                            msg.pickup_y = p.value("y", 0.0f);
                            msg.pickup_z = p.value("z", 0.0f);
                            msg.pickup_yaw = p.value("yaw", 0.0f);

                            auto d = t["dropoff"];
                            msg.dropoff_x = d.value("x", 0.0f);
                            msg.dropoff_y = d.value("y", 0.0f);
                            msg.dropoff_z = d.value("z", 0.0f);
                            msg.dropoff_yaw = d.value("yaw", 0.0f);

                            task_pub_->publish(msg);
                            last_dispatched_task_id_ = t_id;
                            RCLCPP_INFO(get_logger(), "🚀 New Task Dispatched: %s", t_id.c_str());
                        }
                    }
                } catch (const std::exception& e) {
                    RCLCPP_DEBUG(get_logger(), "JSON parse error: %s", e.what());
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        curl_easy_cleanup(curl);
    }

    // ROS 成员
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<sentinel_interfaces::msg::TaskFeedback>::SharedPtr feedback_sub_;
    rclcpp::Publisher<sentinel_interfaces::msg::DeliveryTask>::SharedPtr task_pub_;

    // 数据互斥锁
    std::mutex data_mtx_;
    float battery_ = 0.0f;
    float current_x_ = 0.0f, current_y_ = 0.0f, current_z_ = 0.0f;
    std::string current_task_id_ = "NONE";
    uint8_t current_status_ = 0;
    std::string current_message_ = "Ready";

    std::string last_dispatched_task_id_ = "";
    std::atomic<bool> running_;
    std::thread net_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentNode>());
    rclcpp::shutdown();
    return 0;
}