#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "flight_core/action/move_to.hpp"
#include "uav_bt_agent/plugins/common_types.hpp"

class FollowPathAction : public BT::StatefulActionNode
{
public:
    using MoveTo = flight_core::action::MoveTo;
    using GoalHandleMoveTo = rclcpp_action::ClientGoalHandle<MoveTo>;

    FollowPathAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        client_ = rclcpp_action::create_client<MoveTo>(node_, "flight/move_to");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<geometry_msgs::msg::Point>>("waypoints")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<MoveTo>::SharedPtr client_;
    
    std::vector<geometry_msgs::msg::Point> waypoints_;
    size_t current_idx_;
    bool action_active_;
    
    std::shared_future<GoalHandleMoveTo::SharedPtr> goal_handle_future_;
    std::shared_future<GoalHandleMoveTo::WrappedResult> result_future_;
};
