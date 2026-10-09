// name=debug_imu_rpy_node.cpp
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;

static void quaternion_to_rpy(double qw, double qx, double qy, double qz,
                              double &roll, double &pitch, double &yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::fabs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  // yaw (z-axis rotation)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

class DebugImuRpyNode : public rclcpp::Node
{
public:
  DebugImuRpyNode() : Node("debug_imu_rpy")
  {
    // QoS: small queue is fine for debug
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS()); qos.best_effort(); qos.durability_volatile();
    att_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      std::bind(&DebugImuRpyNode::attitude_cb, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", qos,
      std::bind(&DebugImuRpyNode::imu_cb, this, _1));

    RCLCPP_INFO(this->get_logger(), "debug_imu_rpy node started");
  }

private:
  void attitude_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    // VehicleAttitude q order: [w, x, y, z]
    double qw = static_cast<double>(msg->q[0]);
    double qx = static_cast<double>(msg->q[1]);
    double qy = static_cast<double>(msg->q[2]);
    double qz = static_cast<double>(msg->q[3]);

    double roll, pitch, yaw;
    quaternion_to_rpy(qw, qx, qy, qz, roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(),
                "VehicleAttitude: q=[w=%.6f x=%.6f y=%.6f z=%.6f] RPY(deg) roll=%.2f pitch=%.2f yaw=%.2f",
                qw, qx, qy, qz,
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double qw = static_cast<double>(msg->orientation.w);
    double qx = static_cast<double>(msg->orientation.x);
    double qy = static_cast<double>(msg->orientation.y);
    double qz = static_cast<double>(msg->orientation.z);

    double roll, pitch, yaw;
    quaternion_to_rpy(qw, qx, qy, qz, roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(),
                "IMU /imu/data:   q=[w=%.6f x=%.6f y=%.6f z=%.6f] RPY(deg) roll=%.2f pitch=%.2f yaw=%.2f",
                qw, qx, qy, qz,
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr att_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DebugImuRpyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}