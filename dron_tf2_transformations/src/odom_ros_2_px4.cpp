#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "px4_ros_com/frame_transforms.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>

class OdomRos2Px4 : public rclcpp::Node
{
public:
  OdomRos2Px4() : Node("rtabmap_to_px4_odometry")
  {
    // Parameters
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/rtabmap/odom");
    px4_topic_  = declare_parameter<std::string>("px4_odom_topic", "/fmu/in/vehicle_visual_odometry");
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
    qos_reliability_ = declare_parameter<std::string>("qos_reliability", "best_effort"); // best_effort|reliable
    timestamp_offset_us_ = declare_parameter<int>("timestamp_offset_us", 0);
    quality_ = declare_parameter<int>("quality", 100);
    reset_counter_ = declare_parameter<int>("reset_counter", 0);

    position_var_ = declare_parameter<std::vector<double>>("position_var_xyz", {0.01, 0.01, 0.01});
    orientation_var_ = declare_parameter<std::vector<double>>("orientation_var_rpy", {0.01, 0.01, 0.01});
    velocity_var_ = declare_parameter<std::vector<double>>("velocity_var_xyz", {0.01, 0.01, 0.01});

    // apply use_sim_time
    this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time_));

    // QoS
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS())
                 .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    if (qos_reliability_ == "reliable") {
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    } else {
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    }

    pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic_, qos);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos, std::bind(&OdomRos2Px4::vioCallBack, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Sub: %s | Pub: %s", odom_topic_.c_str(), px4_topic_.c_str());
    RCLCPP_INFO(get_logger(), "use_sim_time_arg: %d | qos_reliability_arg: %s", use_sim_time_, qos_reliability_.c_str());
    RCLCPP_INFO(get_logger(), "position_var_xyz: %f %f %f | orientation_var_rpy: %f %f %f | velocity_var_xyz: %f %f %f", position_var_[0], position_var_[1], position_var_[2], orientation_var_[0], orientation_var_[1], orientation_var_[2], velocity_var_[0], velocity_var_[1], velocity_var_[2]);
  }

private:
  /**
 * @brief Debo documentar LOL
 */
  void vioCallBack(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    px4_msgs::msg::VehicleOdometry out{};

    /**
     * @brief Obtiene el tiempo actual del nodo ROS 2 y lo convierte a microsegundos.
     *
     * PX4 define `timestamp` y `timestamp_sample` como `uint64_t` en µs.
     * `now_us` representa el instante en que este nodo publica el mensaje hacia PX4.
     */
    uint64_t now_us = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);

    /**
     * @brief Inicializa el tiempo de muestra con fallback seguro.
     *
     * Si el mensaje de entrada no trae `header.stamp` válido (sec=0 y nanosec=0),
     * se usa `now_us` para evitar timestamps nulos o inconsistentes.
     */
    uint64_t sample_us = now_us;

    /**
     * @brief Convierte `header.stamp` (sec + nanosec) a microsegundos para PX4.
     *
     * Conversión:
     *   stamp_ns  = sec * 1e9 + nanosec
     *   sample_us = stamp_ns / 1000
     *
     * `timestamp_sample` debe representar el instante real de medición de la odometría
     * (cuando fue estimada por VIO/SLAM), no el instante de publicación.
     */
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
      uint64_t stamp_ns =
        static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(msg->header.stamp.nanosec);
      sample_us = stamp_ns / 1000ULL;
    }

    /**
     * @brief Aplica compensación temporal opcional (offset) sobre el tiempo de muestra.
     *
     * Útil para corregir latencias sistemáticas (sensor, red, pipeline VIO).
     * El parámetro `timestamp_offset_us_` está en microsegundos.
     *
     * Nota: evitar offsets grandes o variables, ya que degradan la fusión EKF.
     */
    if (timestamp_offset_us_ != 0) {
      sample_us = static_cast<uint64_t>(
        static_cast<int64_t>(sample_us) + timestamp_offset_us_);
    }

    /**
     * @brief Asigna timestamps finales en formato esperado por PX4.
     *
     * - out.timestamp: instante de publicación hacia PX4 (µs).
     * - out.timestamp_sample: instante original de medición (µs).
     */
    out.timestamp = now_us;
    out.timestamp_sample = sample_us;

    Eigen::Vector3d pos_rtab_enu(msg->pose.pose.position.x, 
                                 msg->pose.pose.position.y, 
                                 msg->pose.pose.position.z);

    Eigen::Vector3d pos_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(pos_rtab_enu);

    out.position[0] = pos_ned.x();
    out.position[1] = pos_ned.y();
    out.position[2] = pos_ned.z();
    
    /**
     * @brief Eigen::Quaterniond espera formato: (w, x, y, z)
     */
    Eigen::Quaterniond q_rtab_enu(msg->pose.pose.orientation.w,
                                  msg->pose.pose.orientation.x, 
                                  msg->pose.pose.orientation.y, 
                                  msg->pose.pose.orientation.z);
    q_rtab_enu.normalize();

    /**
     * @brief PX4 (out) espera formato: (w, x, y, z) y datos tipo flotante
     */
    Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_rtab_enu);
    
    out.q[0] = static_cast<float>(q_ned.w());
    out.q[1] = static_cast<float>(q_ned.x());
    out.q[2] = static_cast<float>(q_ned.y());
    out.q[3] = static_cast<float>(q_ned.z());

    /**
     * @brief Convierte velocidad lineal desde base_link (ROS) a NED (PX4).
     *
     * Supuesto:
     * 
     * - msg->twist.twist.linear está expresada en frame cuerpo ROS (base_link, ENU convention).
     * 
     * - out.velocity se publicará en frame NED.
     */
    Eigen::Vector3d vel_rtab_enu(msg->twist.twist.linear.x, 
                                 msg->twist.twist.linear.y, 
                                 msg->twist.twist.linear.z);
    
    Eigen::Vector3d vel_aircraft_enu = px4_ros_com::frame_transforms::baselink_to_aircraft_body_frame(vel_rtab_enu);

    Eigen::Vector3d vel_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(vel_aircraft_enu);

    out.velocity[0] = static_cast<float>(vel_ned.x());
    out.velocity[1] = static_cast<float>(vel_ned.y());
    out.velocity[2] = static_cast<float>(vel_ned.z());
    
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    /**
     * @brief Convierte velocidad angular desde base_link (ROS) a NED (PX4).
     *
     * Supuesto:
     * 
     * - msg->twist.twist.linear está expresada en frame cuerpo ROS (base_link, ENU convention).
     * 
     * - out.velocity se publicará en frame NED.
     */
    Eigen::Vector3d avel_rtab_enu(msg->twist.twist.angular.x,
                                  msg->twist.twist.angular.y,
                                  msg->twist.twist.angular.z);
                          
    Eigen::Vector3d avel_aircraft_enu = px4_ros_com::frame_transforms::baselink_to_aircraft_body_frame(avel_rtab_enu);

    Eigen::Vector3d avel_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(avel_aircraft_enu);

    out.angular_velocity[0] = static_cast<float>(avel_ned.x());
    out.angular_velocity[1] = static_cast<float>(avel_ned.y());
    out.angular_velocity[2] = static_cast<float>(avel_ned.z());

    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.quality = static_cast<uint8_t>(std::clamp(quality_, 0, 100));
    out.reset_counter = static_cast<uint8_t>(std::clamp(reset_counter_, 0, 255));


    out.position_variance[0] = static_cast<float>(position_var_[0]);
    out.position_variance[1] = static_cast<float>(position_var_[1]);
    out.position_variance[2] = static_cast<float>(position_var_[2]);

    out.orientation_variance[0] = static_cast<float>(orientation_var_[0]);
    out.orientation_variance[1] = static_cast<float>(orientation_var_[1]);
    out.orientation_variance[2] = static_cast<float>(orientation_var_[2]);

    out.velocity_variance[0] = static_cast<float>(velocity_var_[0]);
    out.velocity_variance[1] = static_cast<float>(velocity_var_[1]);
    out.velocity_variance[2] = static_cast<float>(velocity_var_[2]);
    pub_->publish(out);
  }



  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;

  std::string odom_topic_, px4_topic_, qos_reliability_;
  bool use_sim_time_{false};
  int timestamp_offset_us_{0}, quality_{100}, reset_counter_{0};
  std::vector<double> position_var_, orientation_var_, velocity_var_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRos2Px4>());
  rclcpp::shutdown();
  return 0;
}