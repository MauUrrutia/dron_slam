// name=odometry_tf_transformation.cpp
#include <memory>
#include <array>
#include <cstdint>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>

#include "px4_ros_com/frame_transforms.h" // px4 -> ros frame helpers

using std::placeholders::_1;

class OdomConverterNode : public rclcpp::Node
{
public:
  OdomConverterNode() : Node("odometry_tf_transformation")
  {
    
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Frame names
    odom_frame_ = "px4_odom";
    base_frame_ = "px4_base_footprint";

    // QoS para PX4 (best_effort, volatile)
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      qos,
      std::bind(&OdomConverterNode::odometry_callback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "OdomConverter C++ node started");
  }

private:
  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry odom;
    
    // --- Timestamp mapping (PX4 timestamps are µs since boot) ---
    // Convert PX4 timestamp (µs) to ROS time by estimating boot_time_ros_ = now - px4_ts
    if (msg->timestamp != 0) {
      uint64_t ts_ns = static_cast<uint64_t>(msg->timestamp) * 1000ULL; // µs -> ns

      std::lock_guard<std::mutex> lk(boot_mutex_);
      if (!boot_time_initialized_) {
        boot_time_ros_ = this->get_clock()->now() - rclcpp::Duration::from_nanoseconds(ts_ns);
        boot_time_initialized_ = true;
        RCLCPP_DEBUG(this->get_logger(), "boot_time_ros initialized");
      }
      odom.header.stamp = boot_time_ros_ + rclcpp::Duration::from_nanoseconds(ts_ns);
    } else {
      odom.header.stamp = this->get_clock()->now();
    }

    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // --- Position: use px4_ros_com helper to convert NED -> ENU local frame ---
    Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);

    odom.pose.pose.position.x = pos_enu.x();
    odom.pose.pose.position.y = pos_enu.y();
    odom.pose.pose.position.z = pos_enu.z();

    // --- Orientation: convert PX4 quaternion -> Eigen -> PX4->ROS orientation helper ---
    std::array<float, 4> qarray_px4 = { msg->q[0], msg->q[1], msg->q[2], msg->q[3] };
    Eigen::Quaterniond q_px4 = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(qarray_px4);

    // Convert orientation from PX4 format (aircraft->NED/FRD) to ROS format (baselink->ENU/FLU)
    Eigen::Quaterniond q_ros = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_px4);

    // Ensure consistent sign (optional, helpful to match some Python behavior)
    if (q_ros.w() < 0.0) {
      q_ros.coeffs() *= -1.0; // flip sign of all components (coeffs are x,y,z,w)
    }
    
    odom.pose.pose.orientation.x = q_ros.x();
    odom.pose.pose.orientation.y = q_ros.y();
    odom.pose.pose.orientation.z = q_ros.z();
    odom.pose.pose.orientation.w = q_ros.w();

    // --- Velocity: handle frames using px4_ros_com helpers ---
    Eigen::Vector3d vel_px4(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    Eigen::Vector3d vel_out;

    // velocity_frame values: 1 = VELOCITY_FRAME_NED, 3 = VELOCITY_FRAME_BODY_FRD (matches your python note)
    if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
      vel_out = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_px4);
    } else if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      vel_out = px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame(vel_px4);
    } else {
      vel_out = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_px4);
    }

    odom.twist.twist.linear.x = vel_out.x();
    odom.twist.twist.linear.y = vel_out.y();
    odom.twist.twist.linear.z = vel_out.z();

    // --- Angular velocity: body FRD -> baselink FLU (use aircraft_to_baselink_body_frame) ---
    Eigen::Vector3d av_px4(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
    Eigen::Vector3d av_out = px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame(av_px4);

    odom.twist.twist.angular.x = av_out.x();
    odom.twist.twist.angular.y = av_out.y();
    odom.twist.twist.angular.z = av_out.z();

    // --- Covariances ---
    // If the PX4 message provides covariances and you want to rotate them use:
    // px4_ros_com::frame_transforms::transform_frame(...) helpers for covariances
    // For now we set a simple diagonal as example (customize to your sensor/estimator)
    std::array<double,6> pose_diag = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    std::array<double,6> twist_diag = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    odom.pose.covariance = create_covariance_array(pose_diag);
    odom.twist.covariance = create_covariance_array(twist_diag);

    // Publish /odom
    odom_pub_->publish(odom);

    // Broadcast TF (odom -> base_link)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = odom.header.stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = odom.pose.pose.position.x;
    t.transform.translation.y = odom.pose.pose.position.y;
    t.transform.translation.z = odom.pose.pose.position.z;

    tf2::Quaternion q_tf;
    q_tf.setX(odom.pose.pose.orientation.x);
    q_tf.setY(odom.pose.pose.orientation.y);
    q_tf.setZ(odom.pose.pose.orientation.z);
    q_tf.setW(odom.pose.pose.orientation.w);
    t.transform.rotation.x = q_tf.x();
    t.transform.rotation.y = q_tf.y();
    t.transform.rotation.z = q_tf.z();
    t.transform.rotation.w = q_tf.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::array<double,36> create_covariance_array(const std::array<double,6>& diag)
  {
    std::array<double,36> cov{};
    cov.fill(0.0);
    for (size_t i = 0; i < 6; ++i) {
      cov[i*6 + i] = diag[i];
    }
    return cov;
  }

  // Members
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string odom_frame_;
  std::string base_frame_;

  // boot time mapping
  bool boot_time_initialized_ = false;
  rclcpp::Time boot_time_ros_; // ROS time corresponding to PX4 t = 0 (boot)
  std::mutex boot_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomConverterNode>());
  rclcpp::shutdown();
  return 0;
}