#include "clf_cbf/clf_cbf_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "clf_cbf/cbf.h"
#include "clf_cbf/cbf_dot.h"
#include "clf_cbf/clf.h"
#include "clf_cbf/clf_dot.h"


namespace clf_cbf_node
{

// Wrapper function to call the CasADi cost function
int call_cbf(float& cbf_result, const Eigen::MatrixXf& x) {
    const casadi_real* arg[1];
    casadi_real* res[1];
    casadi_real w[3] = {0};
    casadi_int iw[0] = {};

    arg[0] = x.data();
    
    casadi_real output_result = 0.0;
    res[0] = &output_result;

    int status = h_f(arg, res, iw, w, 0);

    cbf_result = output_result;
    return status;
}

int call_cbf_dot(float& cbf_result, const Eigen::MatrixXf& x, const Eigen::MatrixXf& u, const Eigen::MatrixXf& p) {
    const casadi_real* arg[3];
    casadi_real* res[1];
    casadi_real w[157] = {0};
    casadi_int iw[0] = {};

    arg[0] = x.data();
    arg[1] = u.data();
    arg[2] = p.data();
    
    casadi_real output_result = 0.0;
    res[0] = &output_result;

    int status = d_dx_f(arg, res, iw, w, 0);

    cbf_result = output_result;
    return status;
}

int call_clf(float& clf_result, const Eigen::MatrixXf& x, const Eigen::MatrixXf& p) {
    const casadi_real* arg[2];
    casadi_real* res[1];
    casadi_real w[33] = {0};
    casadi_int iw[0] = {};

    arg[0] = x.data();
    arg[1] = p.data();
    
    casadi_real output_result = 0.0;
    res[0] = &output_result;

    int status = lyapunov_f(arg, res, iw, w, 0);

    clf_result = output_result;
    return status;
}

int call_clf_dot(float& cbf_result, const Eigen::MatrixXf& x, const Eigen::MatrixXf& u, const Eigen::MatrixXf& p) {
    const casadi_real* arg[3];
    casadi_real* res[1];
    casadi_real w[92] = {0};
    casadi_int iw[0] = {};

    arg[0] = x.data();
    arg[1] = u.data();
    arg[2] = p.data();
    
    casadi_real output_result = 0.0;
    res[0] = &output_result;

    int status = V_dot_f(arg, res, iw, w, 0);

    cbf_result = output_result;
    return status;
}

ClfCbfNode::ClfCbfNode(const rclcpp::NodeOptions &options)
    : Node("clf_cbf_node", options)
{
    // Declare parameters with default values
    this->declare_parameter("world_frame_id", std::string("world"));
    this->declare_parameter("quadrotor_name", std::string("quadrotor"));
    this->declare_parameter("camera_frame", std::string("camera"));
    this->declare_parameter("camera_position.x", 0.0);
    this->declare_parameter("camera_position.y", 0.0);
    this->declare_parameter("camera_position.z", -0.075);
    this->declare_parameter("camera_ori.x", -0.70);
    this->declare_parameter("camera_ori.y", 0.70);
    this->declare_parameter("camera_ori.z", 0.0);
    this->declare_parameter("camera_ori.w", 0.0);

    // Get parameters
    this->get_parameter("world_frame_id", world_frame_id_);
    this->get_parameter("quadrotor_name", quad_name_);
    this->get_parameter("camera_frame", camera_name_);

    // Init Variables
    x_ = Eigen::MatrixXf::Zero(22, 1);
    p_ = Eigen::MatrixXf::Zero(27, 1);
    u_ = Eigen::MatrixXf::Zero(4, 1);

    p_(26, 0) = 1.0;
    result_ = Eigen::MatrixXf::Zero(1, 1);
    lyapunov_value_ = Eigen::MatrixXf::Zero(1, 1);
    h_dot_value_ = Eigen::MatrixXf::Zero(1, 1);
    l_dot_value_ = Eigen::MatrixXf::Zero(1, 1);

    result_ptr_ = result_.data();
    lyapunov_ptr_ = lyapunov_value_.data();
    h_dot_ptr_ = h_dot_value_.data();
    l_dot_ptr_ = l_dot_value_.data();

    // Subscriber to /quadrotor/odom
    quadrotor_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + quad_name_ + "/odom", 10, 
        std::bind(&ClfCbfNode::quadrotorOdomCallback, this, std::placeholders::_1));

    // Subscriber to /quadrotor/payload/odom
    payload_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + quad_name_ + "/payload/odom", 10, 
        std::bind(&ClfCbfNode::payloadOdomCallback, this, std::placeholders::_1));

    payload_camera_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/" + quad_name_ + "/payload/point", 10, 
        std::bind(&ClfCbfNode::payloadPointCallback, this, std::placeholders::_1));

    position_cmd_subscriber_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "/" + quad_name_ + "/position_cmd", 10, 
        std::bind(&ClfCbfNode::positionCmdCallback, this, std::placeholders::_1));
    
    control_cmd_subscriber_ = this->create_subscription<quadrotor_msgs::msg::MotorSpeed>(
        "/" + quad_name_ + "/rpm_cmd", 10, 
        std::bind(&ClfCbfNode::controlCmdCallback, this, std::placeholders::_1));

    // Publisher for vector3_stamped
    clf_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/" + quad_name_ + "/" + "payload" + "/clf", 10);
    cbf_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/" + quad_name_ + "/" + "payload" + "/cbf", 10);


    // Timer to publish at 100 Hz (every 5 milliseconds)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&ClfCbfNode::publishCamera, this));
}

// Functions of the system
void ClfCbfNode::quadrotorOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    quadrotor_odometry_ = *msg;
}

void ClfCbfNode::payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    payload_odometry_ = *msg;
}

void ClfCbfNode::payloadPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    payload_camera_point_ = *msg;
}

void ClfCbfNode::positionCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
    position_cmd_ = *msg;
}

void ClfCbfNode::controlCmdCallback(const quadrotor_msgs::msg::MotorSpeed::SharedPtr msg)
{
    control_cmd_ = *msg;
}

void ClfCbfNode::publishCamera()
{   
    // Updates values
    updateStateFromQuadrotorOdometry();
    updateStateFromPayloadOdometry();
    updateStateFromPayloadCameraPoint();
    updatePositionCmd();
    updateControlCmd();

    int status_cbf;
    int status_cbf_dot;
    int status_clf;
    int status_clf_dot;

    // Using CBF and CLF Casadi
    status_cbf = call_cbf(*(result_ptr_), x_);
    status_clf = call_clf(*(lyapunov_ptr_), x_, p_);
    status_cbf_dot = call_cbf_dot(*(h_dot_ptr_), x_, u_, p_);
    status_clf_dot = call_clf_dot(*(l_dot_ptr_), x_, u_, p_);

    // Publish vector as a point for cbf
    geometry_msgs::msg::PointStamped cbf_msg;
    cbf_msg.header.frame_id = camera_name_;
    cbf_msg.header.stamp = this->get_clock()->now();
    cbf_msg.point.x = 0.0;
    cbf_msg.point.y = result_(0, 0);
    cbf_msg.point.z = h_dot_value_(0, 0);
    cbf_publisher_->publish(cbf_msg);

    // Publish vector as a point for clf
    geometry_msgs::msg::PointStamped clf_msg;
    clf_msg.header.frame_id = camera_name_;
    clf_msg.header.stamp = this->get_clock()->now();
    clf_msg.point.x = 0.0;
    clf_msg.point.y = lyapunov_value_(0, 0);
    clf_msg.point.z = l_dot_value_(0, 0);
    clf_publisher_->publish(clf_msg);

}

void ClfCbfNode::updateStateFromQuadrotorOdometry()
{
    // Ensure x_ is the correct size (22x1 matrix as specified)
    if (x_.rows() != 22 || x_.cols() != 1) {
        RCLCPP_ERROR(this->get_logger(), "Matrix x_ has incorrect dimensions. Expected 22x1.");
        return;
    }
    x_(6, 0) = quadrotor_odometry_.pose.pose.position.x;
    x_(7, 0) = quadrotor_odometry_.pose.pose.position.y;
    x_(8, 0) = quadrotor_odometry_.pose.pose.position.z;

    x_(9, 0) = quadrotor_odometry_.twist.twist.linear.x;
    x_(10, 0) = quadrotor_odometry_.twist.twist.linear.y;
    x_(11, 0) = quadrotor_odometry_.twist.twist.linear.z;

    x_(12, 0) = quadrotor_odometry_.pose.pose.orientation.w;
    x_(13, 0) = quadrotor_odometry_.pose.pose.orientation.x;
    x_(14, 0) = quadrotor_odometry_.pose.pose.orientation.y;
    x_(15, 0) = quadrotor_odometry_.pose.pose.orientation.z;

    x_(16, 0) = quadrotor_odometry_.twist.twist.angular.x;
    x_(17, 0) = quadrotor_odometry_.twist.twist.angular.y;
    x_(18, 0) = quadrotor_odometry_.twist.twist.angular.z;
}

void ClfCbfNode::updateStateFromPayloadOdometry()
{
    // Ensure x_ is the correct size (22x1 matrix as specified)
    if (x_.rows() != 22 || x_.cols() != 1) {
        RCLCPP_ERROR(this->get_logger(), "Matrix x_ has incorrect dimensions. Expected 22x1.");
        return;
    }
    x_(0, 0) = payload_odometry_.pose.pose.position.x;
    x_(1, 0) = payload_odometry_.pose.pose.position.y;
    x_(2, 0) = payload_odometry_.pose.pose.position.z;

    x_(3, 0) = payload_odometry_.twist.twist.linear.x;
    x_(4, 0) = payload_odometry_.twist.twist.linear.y;
    x_(5, 0) = payload_odometry_.twist.twist.linear.z;
}

void ClfCbfNode::updateStateFromPayloadCameraPoint()
{
    // Ensure x_ is the correct size (22x1 matrix as specified)
    if (x_.rows() != 22 || x_.cols() != 1) {
        RCLCPP_ERROR(this->get_logger(), "Matrix x_ has incorrect dimensions. Expected 22x1.");
        return;
    }
    
    x_(19, 0) = payload_camera_point_.point.x;
    x_(20, 0) = payload_camera_point_.point.y;
    x_(21, 0) = payload_camera_point_.point.z;
}

void ClfCbfNode::updatePositionCmd()
{
    // Ensure x_ is the correct size (22x1 matrix as specified)
    if (p_.rows() != 27 || p_.cols() != 1) {
        RCLCPP_ERROR(this->get_logger(), "Matrix p_ has incorrect dimensions. Expected 22x1.");
        return;
    }
    p_(0, 0)= position_cmd_.points[0].position.x;
    p_(1, 0)= position_cmd_.points[0].position.y;
    p_(2, 0)= position_cmd_.points[0].position.z;

    p_(3, 0)= position_cmd_.points[0].velocity.x;
    p_(4, 0)= position_cmd_.points[0].velocity.y;
    p_(5, 0)= position_cmd_.points[0].velocity.z;

    p_(6, 0)= position_cmd_.points[0].position_quad.x;
    p_(7, 0)= position_cmd_.points[0].position_quad.y;
    p_(8, 0)= position_cmd_.points[0].position_quad.z;

    p_(9, 0)=  position_cmd_.points[0].velocity_quad.x;
    p_(10, 0) = position_cmd_.points[0].velocity_quad.y;
    p_(11, 0) = position_cmd_.points[0].velocity_quad.z;

    p_(12, 0) = position_cmd_.points[0].quaternion.w;
    p_(13, 0) = position_cmd_.points[0].quaternion.x;
    p_(14, 0) = position_cmd_.points[0].quaternion.y;
    p_(15, 0) = position_cmd_.points[0].quaternion.z;

    p_(16, 0) = position_cmd_.points[0].angular_velocity.x;
    p_(17, 0) = position_cmd_.points[0].angular_velocity.y;
    p_(18, 0) = position_cmd_.points[0].angular_velocity.z;

    p_(19, 0) = 0.0;
    p_(20, 0) = 0.0;
    p_(21, 0) = 0.0;

    p_(22, 0) = 0.0;
    p_(23, 0) = 0.0;
    p_(24, 0) = 0.0;
    p_(25, 0) = 0.0;

    p_(26, 0) = 1.0;
}

void ClfCbfNode::updateControlCmd()
{
    // Ensure x_ is the correct size (22x1 matrix as specified)
    if (u_.rows() != 4 || u_.cols() != 1) {
        RCLCPP_ERROR(this->get_logger(), "Matrix p_ has incorrect dimensions. Expected 22x1.");
        return;
    }
    u_(0, 0)= control_cmd_.forces[0];
    u_(1, 0)= control_cmd_.forces[1];
    u_(2, 0)= control_cmd_.forces[2];
    u_(3, 0)= control_cmd_.forces[3];
}
// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(clf_cbf_node::ClfCbfNode)
}  // namespace camera_projection_node