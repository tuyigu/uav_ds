#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "uav_bt_agent/plugins/action/takeoff_action.hpp"
#include "uav_bt_agent/plugins/action/land_action.hpp"
#include "uav_bt_agent/plugins/action/move_to_action.hpp"
#include "uav_bt_agent/plugins/action/plan_path_action.hpp"
#include "uav_bt_agent/plugins/action/follow_path_action.hpp"
#include "uav_bt_agent/plugins/action/search_marker_action.hpp"
#include "uav_bt_agent/plugins/action/precision_land_action.hpp"

#include "uav_web_agent/msg/mission_intent.hpp"
#include "uav_web_agent/msg/mission_status.hpp"
#include "flight_core/msg/uav_state.hpp"

using namespace uav_bt_agent;

class UAVBtAgent : public rclcpp::Node
{
public:
    UAVBtAgent() : Node("uav_bt_agent")
    {
        // 1. Initialize ROS interfaces
        // Subscribe to Orchestrator intents (not missions directly)
        intent_sub_ = this->create_subscription<uav_web_agent::msg::MissionIntent>(
            "/orchestrator/intent", 10,
            std::bind(&UAVBtAgent::intentCallback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<uav_web_agent::msg::MissionStatus>(
            "/bt/execution_status", 10);
            
        battery_sub_ = this->create_subscription<flight_core::msg::UavState>(
            "/flight/uav_state", 10,
            [this](const flight_core::msg::UavState::SharedPtr msg) {
                last_state_ = msg;
            });

    }

    void init()
    {
        // 2. Register Plugins
        // Note: We use registerBuilder below to inject ROS node dependency.

        // 3. Register Simple Actions/Conditions (Lambdas)
        
        // Arm / Offboard (assuming handled by Takeoff or we just do logging)
        factory_.registerSimpleAction("ArmAction", [&](BT::TreeNode&) {
            RCLCPP_INFO(this->get_logger(), "Arming UAV...");
            // Real implementation would call service, but flight_core handles it.
            return BT::NodeStatus::SUCCESS;
        });
        
        factory_.registerSimpleAction("SetOffboardAction", [&](BT::TreeNode&) {
            RCLCPP_INFO(this->get_logger(), "Switching to Offboard...");
            return BT::NodeStatus::SUCCESS;
        });

        factory_.registerSimpleAction("PublishStatusAction", [&](BT::TreeNode& node) {
            auto status = node.getInput<std::string>("status");
            auto reason = node.getInput<std::string>("reason");
            if (status) {
                publishStatus(status.value(), reason.value_or(""));
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }, {BT::InputPort<std::string>("status"), BT::InputPort<std::string>("reason")});
        
        factory_.registerSimpleAction("EchoAction", [&](BT::TreeNode& node) {
            auto msg = node.getInput<std::string>("message");
            if (msg) RCLCPP_INFO(this->get_logger(), "Echo: %s", msg.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }, {BT::InputPort<std::string>("message")});

        // Conditions
        factory_.registerSimpleCondition("IsBatteryOk", [&](BT::TreeNode& node) {
            auto threshold = node.getInput<float>("threshold");
            if (!last_state_ || !threshold) return BT::NodeStatus::SUCCESS;
            return (last_state_->battery > threshold.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }, {BT::InputPort<float>("threshold")});
        
        factory_.registerSimpleCondition("IsInGeofence", [&](BT::TreeNode&) {
            return BT::NodeStatus::SUCCESS; 
        });

        factory_.registerSimpleCondition("IsSlamHealthy", [&](BT::TreeNode&) {
            return BT::NodeStatus::SUCCESS; 
        });
        
        factory_.registerSimpleCondition("IsMarkerVisible", [&](BT::TreeNode&) {
            return BT::NodeStatus::SUCCESS; 
        }, {BT::InputPort<int>("marker_id")});
        
        factory_.registerSimpleCondition("IsPathClear", [&](BT::TreeNode&) {
            return BT::NodeStatus::SUCCESS; 
        }, {BT::InputPort<std::vector<geometry_msgs::msg::Point>>("path")});
        
        factory_.registerSimpleAction("ReturnToHomeAction", [&](BT::TreeNode&) {
            RCLCPP_WARN(this->get_logger(), "Returning to home...");
            return BT::NodeStatus::SUCCESS; 
        });
        
        factory_.registerSimpleAction("EmergencyLandAction", [&](BT::TreeNode&) {
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY LANDING!");
            return BT::NodeStatus::SUCCESS; 
        });

        // 4. Load Tree
        this->declare_parameter<std::string>("bt_xml_path", "");
        std::string xml_path;
        this->get_parameter("bt_xml_path", xml_path);
        
        if (xml_path.empty()) {
            // Fallback default
            xml_path = "install/uav_bt_agent/share/uav_bt_agent/config/delivery_tree.xml"; 
        }

        try {
            // Pass 'this' node to plugins via blackboard or register args
            auto node_ptr = shared_from_this();
            
            // Re-register with explicit builder for my custom plugins
            factory_.registerBuilder<TakeoffAction>("TakeoffAction", 
                [node_ptr](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<TakeoffAction>(name, config, node_ptr);
            });
            factory_.registerBuilder<LandAction>("LandAction", 
                [node_ptr](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<LandAction>(name, config, node_ptr);
            });
            factory_.registerBuilder<MoveToAction>("GoToAction", 
                [node_ptr](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<MoveToAction>(name, config, node_ptr);
            });
            factory_.registerBuilder<PlanPathAction>("PlanPathAction", 
                [node_ptr](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<PlanPathAction>(name, config, node_ptr);
            });
            factory_.registerBuilder<FollowPathAction>("FollowPathAction", 
                [node_ptr](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<FollowPathAction>(name, config, node_ptr);
            });
            factory_.registerBuilder<SearchMarkerAction>("SearchMarkerAction", 
                [node_ptr](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<SearchMarkerAction>(name, config, node_ptr);
            });
            factory_.registerBuilder<PrecisionLandAction>("PrecisionLandAction", 
                [node_ptr](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<PrecisionLandAction>(name, config, node_ptr);
            });

            tree_ = factory_.createTreeFromFile(xml_path);
            
            // Set blackboard defaults for 5-phase flight dynamics
            auto bb = tree_.rootBlackboard();
            bb->set("intent", std::string(""));  // No intent until Orchestrator sends one
            bb->set("phase", std::string("IDLE"));
            bb->set("should_hold", false);
            bb->set("should_abort", false);
            
            bb->set("cruise_alt", 30.0f);
            bb->set("approach_alt", 15.0f);
            bb->set("search_alt", 5.0f);
            bb->set("takeoff_height", 5.0f);
            
            bb->set("home_x", 0.0f);
            bb->set("home_y", 0.0f);
            
            bb->set("battery_min", 0.15f);
            
            // Groot2 Publisher
            groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_);
            RCLCPP_INFO(this->get_logger(), "Groot2 Publisher started on port 1667");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create tree: %s", e.what());
        }
    }

    void update()
    {
        // Inject real-time state into Blackboard
        if (last_state_) {
            auto bb = tree_.rootBlackboard();
            bb->set("current_x", last_state_->x);
            bb->set("current_y", last_state_->y);
            bb->set("current_z", last_state_->z);
            bb->set("battery", last_state_->battery);
        }

        if (is_running_) {
            BT::NodeStatus status = tree_.tickExactlyOnce();
            
            if (status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Tree finished: SUCCESS");
                publishStatus("SUCCESS", current_intent_);
                is_running_ = false;
            } else if (status == BT::NodeStatus::FAILURE) {
                RCLCPP_WARN(this->get_logger(), "Tree finished: FAILURE");
                publishStatus("FAILED", "BT execution failed");
                is_running_ = false;
            }
            // RUNNING → continue next tick
        }
    }

    void intentCallback(const uav_web_agent::msg::MissionIntent::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received intent: %s (phase=%s, reason=%s)",
                    msg->intent.c_str(), msg->phase.c_str(), msg->reason.c_str());
        
        auto blackboard = tree_.rootBlackboard();
        
        // Map intent to blackboard
        // Note: Cast to float because BT ports (MoveTo) expect float, but ROS msg is float64 (double)
        std::string intent_str = msg->intent;
        if (intent_str == "REROUTE") {
            intent_str = "DELIVER";
        }
        blackboard->set("intent", intent_str);
        blackboard->set("phase", msg->phase);
        blackboard->set("dest_x", (float)msg->target_x);
        blackboard->set("dest_y", (float)msg->target_y);
        blackboard->set("dest_z", (float)msg->target_z);
        blackboard->set("target_marker_id", msg->target_marker_id);
        
        current_mission_id_ = msg->mission_id;
        
        // Intent-based behavior
        if (msg->intent == "DELIVER" || msg->intent == "RETURN_HOME" || msg->intent == "REROUTE") {
            current_intent_ = msg->intent;
            if (!is_running_) {
                is_running_ = true;
                publishStatus("RUNNING", msg->intent);
            }
            // If already running, BT will pick up new blackboard values on next tick
        } else if (msg->intent == "HOLD") {
            // Set hold flag → BT should react and hover
            blackboard->set("should_hold", true);
            publishStatus("HOLDING", msg->reason);
        } else if (msg->intent == "ABORT") {
            blackboard->set("should_abort", true);
            publishStatus("ABORTING", msg->reason);
        } else if (msg->intent == "WAIT_CONFIRM") {
            blackboard->set("should_hold", true);
            publishStatus("WAITING", msg->reason);
        }
    }
    
    void publishStatus(const std::string& status, const std::string& reason)
    {
        uav_web_agent::msg::MissionStatus msg;
        msg.mission_id = current_mission_id_;
        msg.status = status;
        msg.reason = reason;
        msg.progress = 0.0f;
        msg.user_facing_msg = "";
        msg.eta_seconds = 0;
        status_pub_->publish(msg);
    }

private:
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
    
    bool is_running_ = false;
    std::string current_mission_id_;
    std::string current_intent_;
    
    rclcpp::Subscription<uav_web_agent::msg::MissionIntent>::SharedPtr intent_sub_;
    rclcpp::Publisher<uav_web_agent::msg::MissionStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<flight_core::msg::UavState>::SharedPtr battery_sub_;
    flight_core::msg::UavState::SharedPtr last_state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UAVBtAgent>();
    node->init();
    
    rclcpp::Rate rate(10);
    while(rclcpp::ok()) {
        node->update();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
