#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include "uav_bt_agent/plugins/action/takeoff_action.hpp"
#include "uav_bt_agent/plugins/action/move_to_action.hpp"
#include "uav_bt_agent/plugins/action/land_action.hpp"
#include "uav_web_agent/msg/delivery_mission.hpp"
#include "uav_web_agent/msg/mission_status.hpp"

using DeliveryMission = uav_web_agent::msg::DeliveryMission;
using MissionStatus = uav_web_agent::msg::MissionStatus;

class UAVBtAgent : public rclcpp::Node
{
public:
  UAVBtAgent() : Node("uav_bt_agent")
  {
    // Subscribe to mission
    mission_sub_ = this->create_subscription<DeliveryMission>(
      "/uav/mission/new", 10,
      std::bind(&UAVBtAgent::missionCallback, this, std::placeholders::_1));

    // Publish mission status
    status_pub_ = this->create_publisher<MissionStatus>(
      "/bt/mission_status", 10);
    
    // Initialize BT Factory
    factory_.registerBuilder<uav_bt_agent::TakeoffAction>(
        "TakeoffAction",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<uav_bt_agent::TakeoffAction>(name, config, shared_from_this());
        });
    
    factory_.registerBuilder<uav_bt_agent::MoveToAction>(
        "MoveToAction",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<uav_bt_agent::MoveToAction>(name, config, shared_from_this());
        });
        
    factory_.registerBuilder<uav_bt_agent::LandAction>(
        "LandAction",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<uav_bt_agent::LandAction>(name, config, shared_from_this());
        });
        
    // Pre-load tree (optional, or load on demand)
    // Default path assumes running from workspace root, but launch file should override this
    this->declare_parameter<std::string>("bt_xml_path", "install/uav_bt_agent/share/uav_bt_agent/config/simple_flight_tree.xml");
    this->get_parameter("bt_xml_path", tree_xml_path_);
  }

  void missionCallback(const DeliveryMission::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received mission: %s", msg->mission_id.c_str());
    
    if (is_running_) {
        RCLCPP_WARN(this->get_logger(), "Agent is busy, ignoring mission");
        return;
    }

    try {
        tree_ = factory_.createTreeFromFile(tree_xml_path_);
        
        // Set Blackboard values
        // Note: transforming Lat/Lon to local XYZ is skipped here as per task (using direct values or assuming simple conversion)
        // For this version, we map dropoff_lat/lon to x/y directly for testing
        // You might want to use 0,0 for pickup if needed.
        
        tree_.rootBlackboard()->set("takeoff_height", 5.0f);
        tree_.rootBlackboard()->set("target_x", (float)msg->dropoff_lat); // Hack for mapping
        tree_.rootBlackboard()->set("target_y", (float)msg->dropoff_lon); // Hack for mapping
        tree_.rootBlackboard()->set("target_z", (float)msg->dropoff_alt);
        tree_.rootBlackboard()->set("target_yaw", 0.0f);

        is_running_ = true;
        current_mission_id_ = msg->mission_id;
        current_uav_id_ = msg->uav_id;

        RCLCPP_INFO(this->get_logger(), "Starting Behavior Tree execution...");
        
        // Publish RUNNING status
        publishStatus("RUNNING", "Mission started");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create tree: %s", e.what());
        publishStatus("FAILED", e.what());
    }
  }

  void update()
  {
    if (is_running_) {
        BT::NodeStatus status = tree_.tickRoot();
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Mission finished with SUCCESS");
            publishStatus("COMPLETED", "Tree finished successfully");
            is_running_ = false;
        }
        else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_INFO(this->get_logger(), "Mission finished with FAILURE");
            publishStatus("FAILED", "Tree finished with failure");
            is_running_ = false;
        }
    }
  }

  void publishStatus(const std::string& status_str, const std::string& reason)
  {
    MissionStatus status_msg;
    status_msg.mission_id = current_mission_id_;
    status_msg.uav_id = current_uav_id_;
    status_msg.status = status_str;
    status_msg.reason = reason;
    status_pub_->publish(status_msg);
  }

private:
  rclcpp::Subscription<DeliveryMission>::SharedPtr mission_sub_;
  rclcpp::Publisher<MissionStatus>::SharedPtr status_pub_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::string tree_xml_path_;
  bool is_running_ = false;
  std::string current_mission_id_;
  std::string current_uav_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UAVBtAgent>();

  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    node->update();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

