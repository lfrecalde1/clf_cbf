#include "clf_cbf/clf_cbf_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "clf_cbf/cbf.h"
#include "clf_cbf/clf.h"

namespace clf_cbf_node
{

// Wrapper function to call the CasADi cost function
int call_cbf(float& cbf_result, const Eigen::MatrixXf& x) {
    const casadi_real* arg[1];
    casadi_real* res[1];
    casadi_real w[h_f_SZ_W] = {0};
    casadi_int iw[h_f_SZ_IW] = {};

    arg[0] = x.data();
    
    casadi_real output_result = 0.0;
    res[0] = &output_result;

    int status = h_f(arg, res, iw, w, 0);

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
    x_(0, 0) = 2.0;
    result_ = Eigen::MatrixXf::Zero(1, 1);
    result_ptr_ = result_.data();

    // Subscriber to /quadrotor/odom
    quadrotor_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + quad_name_ + "/odom", 10, 
        std::bind(&ClfCbfNode::quadrotorOdomCallback, this, std::placeholders::_1));

    // Subscriber to /quadrotor/payload/odom
    payload_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + quad_name_ + "/payload/odom", 10, 
        std::bind(&ClfCbfNode::payloadOdomCallback, this, std::placeholders::_1));
    
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

void ClfCbfNode::publishCamera()
{
    float aux = 25.3;
    int status_1;
    x_ = Eigen::MatrixXf::Random(22, 1);

    // Using CBF Casadi
    status_1 = call_cbf(*(result_ptr_), x_);
    // Publish vector as a point 
    geometry_msgs::msg::PointStamped clf_msg;
    clf_msg.header.frame_id = camera_name_;
    clf_msg.header.stamp = this->get_clock()->now();
    clf_msg.point.x = aux;
    clf_msg.point.y = x_(0, 0);
    clf_msg.point.z = result_(0, 0);
    clf_publisher_->publish(clf_msg);

}

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(clf_cbf_node::ClfCbfNode)
}  // namespace camera_projection_node