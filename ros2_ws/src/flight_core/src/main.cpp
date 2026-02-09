#include <rclcpp/rclcpp.hpp>
#include "flight_core/flight_core.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<flight_core::FlightCore>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

