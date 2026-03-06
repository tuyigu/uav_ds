#include "flight_core_v2/flight_core_v2.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fc2::FlightCoreV2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
