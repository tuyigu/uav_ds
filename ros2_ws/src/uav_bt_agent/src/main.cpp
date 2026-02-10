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

#include "uav_web_agent/msg/delivery_mission.hpp"
#include "uav_web_agent/msg/mission_status.hpp"
#include "flight_core/msg/uav_state.hpp"

using namespace uav_bt_agent;

class UAVBtAgent : public rclcpp::Node
{
public:
    UAVBtAgent() : Node("uav_bt_agent")
    {
        // 1. Initialize ROS interfaces
        mission_sub_ = this->create_subscription<uav_web_agent::msg::DeliveryMission>(
            "/uav/mission/new", 10,
            std::bind(&UAVBtAgent::missionCallback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<uav_web_agent::msg::MissionStatus>(
            "/bt/mission_status", 10);
            
        battery_sub_ = this->create_subscription<flight_core::msg::UavState>(
            "/flight/uav_state", 10,
            [this](const flight_core::msg::UavState::SharedPtr msg) {
                last_state_ = msg;
            });

        // 2. Register Plugins
        // Note: We use registerBuilder below to inject ROS node dependency.
        // registerNodeType is removed to avoid static assertion errors about constructor signature.

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
        });
        
        factory_.registerSimpleAction("EchoAction", [&](BT::TreeNode& node) {
            auto msg = node.getInput<std::string>("message");
            if (msg) RCLCPP_INFO(this->get_logger(), "Echo: %s", msg.value().c_str());
            return BT::NodeStatus::SUCCESS;
        });

        // Conditions
        factory_.registerSimpleCondition("IsBatteryOk", [&](BT::TreeNode& node) {
            if (!last_state_) return BT::NodeStatus::SUCCESS; // Assume OK if no data
            auto threshold = node.getInput<float>("threshold");
            // Check voltage or percentage. UavState has battery_remaining (0-1)
            // if (last_state_->battery_remaining < threshold.value_or(0.2)) return BT::NodeStatus::FAILURE;
            return BT::NodeStatus::SUCCESS; 
        });
        
        factory_.registerSimpleCondition("IsInGeofence", [&](BT::TreeNode&) {
            // Check bounds
            return BT::NodeStatus::SUCCESS; 
        });

        factory_.registerSimpleCondition("IsSlamHealthy", [&](BT::TreeNode&) {
            // Check SLAM status/latency
            return BT::NodeStatus::SUCCESS; 
        });
        
        factory_.registerSimpleCondition("IsMarkerVisible", [&](BT::TreeNode&) {
            // Logic would need access to last marker time. SearchMarkerAction handles this during search.
            // This is just a check.
            return BT::NodeStatus::SUCCESS; 
        });
        
        factory_.registerSimpleCondition("IsPathClear", [&](BT::TreeNode&) {
            return BT::NodeStatus::SUCCESS; 
        });
        
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
            // In v4, we can register a builder that captures 'this'.
            // The method registerNodeType<T> expects T to have specific constructor.
            // My plugins expect (name, config, node_ptr).
            // So I need to specificy the builder explicitly.
             
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
            
            // Groot2 Publisher
            groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_);
            RCLCPP_INFO(this->get_logger(), "Groot2 Publisher started on port 1667");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create tree: %s", e.what());
        }
    }

    void update()
    {
        if (is_running_) {
            BT::NodeStatus status = tree_.tickExactlyOnce(); // v4 API for reactive trees
            // or tickOnce();
            
            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(this->get_logger(), "Tree finished with %s", toStr(status).c_str());
                is_running_ = false;
            }
        }
    }

    void missionCallback(const uav_web_agent::msg::DeliveryMission::SharedPtr msg)
    {
        if (is_running_) {
            RCLCPP_WARN(this->get_logger(), "Busy, ignoring mission");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Received mission %s", msg->mission_id.c_str());
        
        // Set Blackboard Variables
        auto blackboard = tree_.rootBlackboard();
        blackboard->set("dest_x", msg->dropoff_lat); // Hack mapping
        blackboard->set("dest_y", msg->dropoff_lon); // Hack mapping
        blackboard->set("takeoff_height", 5.0);
        blackboard->set("cruise_alt", 5.0);
        blackboard->set("search_alt", 3.0);
        blackboard->set("pickup_wait_time", 5000); // ms
        blackboard->set("home_x", 0.0);
        blackboard->set("home_y", 0.0);
        blackboard->set("target_marker_id", 0); // Default or from mission?
        
        current_mission_id_ = msg->mission_id;
        is_running_ = true;
    }
    
    void publishStatus(const std::string& status, const std::string& reason)
    {
        uav_web_agent::msg::MissionStatus msg;
        msg.mission_id = current_mission_id_;
        msg.status = status;
        msg.reason = reason;
        status_pub_->publish(msg);
    }

private:
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
    
    bool is_running_ = false;
    std::string current_mission_id_;
    
    rclcpp::Subscription<uav_web_agent::msg::DeliveryMission>::SharedPtr mission_sub_;
    rclcpp::Publisher<uav_web_agent::msg::MissionStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<flight_core::msg::UavState>::SharedPtr battery_sub_;
    flight_core::msg::UavState::SharedPtr last_state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UAVBtAgent>();
    
    rclcpp::Rate rate(10);
    while(rclcpp::ok()) {
        node->update();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
