#ifndef CLF_CBF_NODE_HPP
#define CLF_CBF_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

namespace clf_cbf_node
{
class ClfCbfNode : public rclcpp::Node
{
public:
    explicit ClfCbfNode(const rclcpp::NodeOptions &options);

private:
    void quadrotorOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void payloadPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void positionCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);

    // Helper functions
    void publishCamera();
    void updateStateFromQuadrotorOdometry();  
    void updateStateFromPayloadOdometry();  
    void updateStateFromPayloadCameraPoint();  
    void updatePositionCmd();  

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
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr payload_camera_subscriber_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_subscriber_;

    // Data
    nav_msgs::msg::Odometry quadrotor_odometry_;  
    nav_msgs::msg::Odometry payload_odometry_;
    geometry_msgs::msg::PointStamped payload_camera_point_;
    quadrotor_msgs::msg::PositionCommand position_cmd_;

    // Vectors of my system
    Eigen::MatrixXf x_;
    Eigen::MatrixXf p_;
    Eigen::MatrixXf result_;
    Eigen::MatrixXf lyapunov_value_;
    float* result_ptr_;
    float* lyapunov_ptr_;

};
}  // namespace clf_cbf_node

#endif  // CLF_CBF_NODE_HPP