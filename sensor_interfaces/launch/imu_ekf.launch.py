
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_interfaces',
            executable='imu_converter_px4_ros_node',
            name='imu_converter_px4_ros_node',
            output='screen'
        ),
        # Node(
        #     package='sensor_interfaces',
        #     executable='odom_republisher',
        #     name='odom_republisher',
        #     output='screen'
        # ),
        
        # # 2. Nodo EKF de robot_localization
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[os.path.join(get_package_share_directory("tu_paquete"), 'config', 'ekf.yaml')]
        # ),
    ])