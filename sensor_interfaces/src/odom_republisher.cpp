#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class OdomRepublisherNode : public rclcpp::Node
{
public:
  OdomRepublisherNode() : Node("odom_republisher_cpp")
  {
    auto px4_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    px4_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    px4_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    px4_odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      px4_qos,
      std::bind(&OdomRepublisherNode::px4_odometry_callback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::QoS(10)
    );
    RCLCPP_INFO(this->get_logger(), "Odom Republisher Node started");
  }

private:
  void px4_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

    odom_msg->header.stamp = this->get_clock()->now();
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";

    // ==========================================
    // POSICIÓN: NED → ENU con función oficial
    // ==========================================
    Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d pos_enu = 
        px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);

    odom_msg->pose.pose.position.x = pos_enu.x();
    odom_msg->pose.pose.position.y = pos_enu.y();
    odom_msg->pose.pose.position.z = pos_enu.z();

    // ==========================================
    // ORIENTACIÓN: NED→FRD → ENU→FLU con función oficial
    // ==========================================
    Eigen::Quaterniond q_px4(msg->q[3], msg->q[0], msg->q[1], msg->q[2]);
    Eigen::Quaterniond q_ros = 
        px4_ros_com::frame_transforms::px4_to_ros_orientation(q_px4);

    odom_msg->pose.pose.orientation.x = q_ros.x();
    odom_msg->pose.pose.orientation.y = q_ros.y();
    odom_msg->pose.pose.orientation.z = q_ros.z();
    odom_msg->pose.pose.orientation.w = q_ros.w();

    // ==========================================
    // VELOCIDAD LINEAL: NED → ENU con función oficial
    // ==========================================
    Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    Eigen::Vector3d vel_enu = 
        px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);

    odom_msg->twist.twist.linear.x = vel_enu.x();
    odom_msg->twist.twist.linear.y = vel_enu.y();
    odom_msg->twist.twist.linear.z = vel_enu.z();

    // ==========================================
    // VELOCIDAD ANGULAR: FRD → FLU con función oficial
    // ==========================================
    Eigen::Vector3d ang_vel_frd(msg->angular_velocity[0], 
                                msg->angular_velocity[1], 
                                msg->angular_velocity[2]);
    Eigen::Vector3d ang_vel_flu = 
        px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame(ang_vel_frd);

    odom_msg->twist.twist.angular.x = ang_vel_flu.x();
    odom_msg->twist.twist.angular.y = ang_vel_flu.y();
    odom_msg->twist.twist.angular.z = ang_vel_flu.z();

    // Covarianzas (opcional)
    odom_msg->pose.covariance = {
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01
    };

    odom_msg->twist.covariance = {
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01
    };

    odom_publisher_->publish(*odom_msg);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}