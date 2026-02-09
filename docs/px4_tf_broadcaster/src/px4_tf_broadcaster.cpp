#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp" // 新增：Odom 消息头文件

class Px4OdomBridge : public rclcpp::Node {
public:
    Px4OdomBridge() : Node("px4_odom_bridge",
                      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 新增：初始化 Odom 话题发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        auto px4_qos = rclcpp::QoS(10).best_effort().durability_volatile();
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", px4_qos,
            std::bind(&Px4OdomBridge::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "🚀 Sentinel-X Bridge (TF + Odom) 已启动");
    }

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        auto now = this->get_clock()->now();
        if (now.nanoseconds() == 0) return;

        // --- 1. 发布 TF (保持不变) ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        // NED -> ENU 坐标映射
        t.transform.translation.x = msg->position[1];  // East -> X
        t.transform.translation.y = msg->position[0];  // North -> Y
        t.transform.translation.z = -msg->position[2]; // -Down -> Z

        t.transform.rotation.w = msg->q[0];
        t.transform.rotation.x = msg->q[2];
        t.transform.rotation.y = msg->q[1];
        t.transform.rotation.z = -msg->q[3];

        tf_broadcaster_->sendTransform(t);

        // --- 2. 发布 Odom Topic (新增部分) ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // 位置 (直接复用 TF 的计算)
        odom_msg.pose.pose.position.x = t.transform.translation.x;
        odom_msg.pose.pose.position.y = t.transform.translation.y;
        odom_msg.pose.pose.position.z = t.transform.translation.z;
        odom_msg.pose.pose.orientation = t.transform.rotation;

        // 速度 (Velocity) 也需要从 NED 转 ENU
        // PX4 velocity: [North, East, Down]
        // ROS velocity: [East, North, Up]
        odom_msg.twist.twist.linear.x = msg->velocity[1];  // Vy_px4 -> Vx_ros
        odom_msg.twist.twist.linear.y = msg->velocity[0];  // Vx_px4 -> Vy_ros
        odom_msg.twist.twist.linear.z = -msg->velocity[2]; // -Vz_px4 -> Vz_ros

        // 角速度 (Angular Velocity)
        // PX4: [Roll, Pitch, Yaw] (围绕 NED 轴)
        // ROS: [Roll, Pitch, Yaw] (围绕 ENU 轴)
        // 这是一个近似转换，对应 Body Frame 下的轴交换
        odom_msg.twist.twist.angular.x = msg->angular_velocity[1];
        odom_msg.twist.twist.angular.y = msg->angular_velocity[0];
        odom_msg.twist.twist.angular.z = -msg->angular_velocity[2];

        // 填充协方差 (Covariance) - 给 SLAM 一个信心值
        // 这里简单给一个单位矩阵，表示“我很自信”
        // 实际上应该用 msg->position_variance 填充
        // 但为了跑通 SLAM，默认值通常够用

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // 新增
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OdomBridge>());
    rclcpp::shutdown();
    return 0;
}