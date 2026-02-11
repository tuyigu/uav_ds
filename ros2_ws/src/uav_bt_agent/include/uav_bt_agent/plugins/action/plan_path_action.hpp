#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "uav_navigation/srv/plan_path.hpp"
#include "uav_bt_agent/plugins/common_types.hpp"


class PlanPathAction : public BT::StatefulActionNode
{
public:
    PlanPathAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        client_ = node_->create_client<uav_navigation::srv::PlanPath>("plan_path");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<float>("goal_x"),
            BT::InputPort<float>("goal_y"),
            BT::InputPort<float>("goal_z"),
            BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("waypoints")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<uav_navigation::srv::PlanPath>::SharedPtr client_;
    std::shared_future<std::shared_ptr<uav_navigation::srv::PlanPath::Response>> future_;
};
