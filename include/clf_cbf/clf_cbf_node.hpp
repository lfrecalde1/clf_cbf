#ifndef CLF_CBF_NODE_HPP
#define CLF_CBF_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace clf_cbf_node
{
class ClfCbfNode : public rclcpp::Node
{
public:
    explicit ClfCbfNode(const rclcpp::NodeOptions &options);

private:
    void quadrotorOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Helper functions
    void publishCamera();

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Frames names
    std::string world_frame_id_;
    std::string quad_name_;
    std::string camera_name_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr clf_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cbf_publisher_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr quadrotor_odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr payload_odom_subscriber_;

    // Data
    nav_msgs::msg::Odometry quadrotor_odometry_;  
    nav_msgs::msg::Odometry payload_odometry_;

    // Vectors of my system
    Eigen::MatrixXf x_;
    Eigen::MatrixXf result_;
    float* result_ptr_;

};
}  // namespace clf_cbf_node

#endif  // CLF_CBF_NODE_HPP