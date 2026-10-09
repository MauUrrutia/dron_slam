#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>


#include <chrono>
#include <atomic>
#include <cmath>



using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;


class OffboardControlSetpoint : public rclcpp::Node
{
public:
    OffboardControlSetpoint() : Node("offboard_control_by_setpoint"){
        offboard_control_mode_pub_ = 
            create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = 
            create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ = 
            create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        // Tópico de navegación/autonomía (fuente de setpoints)

        nav_setpoint_sub_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>("/nav/setpoint", 10, std::bind(&OffboardControlSetpoint::navSetpointCallback, 
                                                                this, std::placeholders::_1));
    
        //Hovering (setpoint de seguridad (0, 0, 0.2)
        desired_x_ = 0.0f;
        desired_y_ = 0.0f;
        desired_z_ = -0.02f;  
        desired_yaw_ = M_PIf/2;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "New nav setpoint: x=%.2fm y=%.2fm z=%.2fm, yaw=%.2f rad", desired_x_, desired_y_, desired_z_, desired_yaw_);
        timer_ = create_wall_timer(100ms, std::bind(&OffboardControlSetpoint::timerCallback, this));
        }



private: 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_setpoint_sub_;

    uint64_t offboard_setpoint_counter_{0};
    bool armed_{false};
    bool offboard_enabled_{false};


    // setpoint recibido
    float desired_x_, desired_y_, desired_z_, desired_yaw_;

    void navSetpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        // Posición ENU -> NED (si tu tópico viene en ENU)
        // NED: x=north, y=east, z=down
        const float x_enu = static_cast<float>(msg->pose.position.x);
        const float y_enu = static_cast<float>(msg->pose.position.y);
        const float z_enu = static_cast<float>(msg->pose.position.z);
        desired_x_ =  y_enu;
        desired_y_ =  x_enu;
        desired_z_ = -z_enu;
        desired_yaw_= 0.0f;

        tf2::Quaternion q(
            msg->pose.orientation.x,        
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // yaw ENU -> yaw NED
        double yaw_ned = M_PI_2 - yaw;

        // normalizar a [-pi, pi]
        while (yaw_ned > M_PI)  yaw_ned -= 2.0 * M_PI;
        while (yaw_ned < -M_PI) yaw_ned += 2.0 * M_PI;

        desired_yaw_ = static_cast<float>(yaw_ned);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "New nav setpoint: x=%.2fm y=%.2fm z=%.2fm, yaw=%.2f rad", desired_x_, desired_y_, desired_z_, desired_yaw_);
    }

    void timerCallback(){
        // Publicar siempre modo + setpoint para mantener OFFBOARD
        publishOffboardControlMode();
        publishTrajectorySetpoint();
    
        // Enviar algunos setpoints antes de pasar a OFFBOARD
        if (offboard_setpoint_counter_ == 10 && !offboard_enabled_) {
            publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Offboard
            offboard_enabled_ = true;
            RCLCPP_INFO(get_logger(), "Offboard mode command sent");
        }
        
        if (offboard_setpoint_counter_ == 12 && !armed_) {
            arm();
            armed_ = true;
        }

        if (offboard_setpoint_counter_ < 20) {
            offboard_setpoint_counter_++;
        }
    }


    void arm(){
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(get_logger(), "Arm command sent");
    }



    void publishOffboardControlMode(){
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }



    void publishTrajectorySetpoint(){
        TrajectorySetpoint msg{};
        msg.position = {desired_x_, desired_y_, desired_z_};
        msg.yaw = desired_yaw_;
        msg.timestamp = now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }



    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0){
            VehicleCommand msg{};
            msg.param1 = param1;
            msg.param2 = param2;
            msg.command = command;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = now().nanoseconds() / 1000;
            vehicle_command_pub_->publish(msg);
        }


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControlSetpoint>());
  rclcpp::shutdown();
  return 0;
}
