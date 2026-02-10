#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace BT {
    template <> inline
    std::vector<geometry_msgs::msg::Point> convertFromString(StringView /*str*/)
    {
        // We generally don't parse vectors from XML string manually in this project
        // We rely on port remapping (Blackboard)
        return {}; 
    }
}
