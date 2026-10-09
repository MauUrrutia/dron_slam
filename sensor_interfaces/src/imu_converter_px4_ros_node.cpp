#include <memory>
#include <cstdint>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_ros_com/frame_transforms.h" // si está disponible
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Geometry>

class ImuRepublisherNode : public rclcpp::Node
{
public:
  ImuRepublisherNode() : Node("imu_republisher")
  {
    // QoS PX4 (sensor)
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    qos.durability_volatile();

    sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      "/fmu/out/sensor_combined", qos,
      std::bind(&ImuRepublisherNode::sensor_callback, this, std::placeholders::_1));

    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      std::bind(&ImuRepublisherNode::attitude_callback, this, std::placeholders::_1));

    // Publicador usa la misma QoS sensor-data (best-effort/volatile)
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);

    RCLCPP_INFO(this->get_logger(), "IMU Republisher started (publishing /imu/data)");
  }

private:
  // boot time mapping
  bool boot_time_initialized_ = false;
  rclcpp::Time boot_time_ros_; // ROS time corresponding to PX4 t = 0 (boot)
  std::mutex last_mutex_;

  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg;
    {
      std::lock_guard<std::mutex> lk(last_mutex_);
      if (!last_imu_msg_) {
        RCLCPP_DEBUG(this->get_logger(), "No sensor data cached, skipping");
        return;
      }
      imu_msg = last_imu_msg_;
    }

    // --- Quaternion: VehicleAttitude q order = [w, x, y, z]
    Eigen::Quaterniond q_px4(
      static_cast<double>(msg->q[0]), // w
      static_cast<double>(msg->q[1]), // x
      static_cast<double>(msg->q[2]), // y
      static_cast<double>(msg->q[3])  // z
    );

    // Preferir helper oficial (si está disponible)
    Eigen::Quaterniond q_ros;
    try {
      q_ros = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_px4);
    } catch (...) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "px4_to_ros_orientation not available; verify orientation manually");
      // fallback aproximado: (NO garantizado para todos los casos)
      q_ros = q_px4; // temporal: mejor usar el helper
    }

    imu_msg->orientation.x = q_ros.x();
    imu_msg->orientation.y = q_ros.y();
    imu_msg->orientation.z = q_ros.z();
    imu_msg->orientation.w = q_ros.w();

    // --- Timestamp mapping: PX4 timestamps en µs desde boot -> mapear a ROS time
    if (msg->timestamp != 0) {
      uint64_t ts_ns = static_cast<uint64_t>(msg->timestamp) * 1000ULL; // µs -> ns

      if (!boot_time_initialized_) {
        // Estimamos boot_time_ros = now() - timestamp_ns
        boot_time_ros_ = this->get_clock()->now() - rclcpp::Duration::from_nanoseconds(ts_ns);
        boot_time_initialized_ = true;
        RCLCPP_DEBUG(this->get_logger(), "boot_time_ros initialized");
      }

      imu_msg->header.stamp = boot_time_ros_ + rclcpp::Duration::from_nanoseconds(ts_ns);
    } else {
      imu_msg->header.stamp = this->get_clock()->now();
    }

    // publicamos
    imu_pub_->publish(*imu_msg);

    // invalidar cached
    {
      std::lock_guard<std::mutex> lk(last_mutex_);
      last_imu_msg_.reset();
    }
  }

  void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.frame_id = "base_link";

    // FRD -> ROS-body (invertimos y,z para vectores)
    imu_msg->angular_velocity.x =  msg->gyro_rad[0];
    imu_msg->angular_velocity.y = -msg->gyro_rad[1];
    imu_msg->angular_velocity.z = -msg->gyro_rad[2];

    imu_msg->linear_acceleration.x =  msg->accelerometer_m_s2[0];
    imu_msg->linear_acceleration.y = -msg->accelerometer_m_s2[1];
    imu_msg->linear_acceleration.z = -msg->accelerometer_m_s2[2];

    {
      std::lock_guard<std::mutex> lk(last_mutex_);
      last_imu_msg_ = imu_msg;
    }
  }

  // Members
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;
  uint64_t last_sensor_timestamp_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}