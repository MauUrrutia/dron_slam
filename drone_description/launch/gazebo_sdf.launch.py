from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    pkg_path = get_package_share_path('drone_description')
    
    # ========== PATHS ==========
    # Para Gazebo (SDF)
    resource_path = os.path.dirname(os.path.dirname(pkg_path))
    sdf_path = os.path.join(pkg_path, 'worlds', 'drone_sim.sdf')
    
    # Para RVIZ2 (URDF/XACRO)
    urdf_path = os.path.join(pkg_path, 'urdf', 'drone.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'rviz2_config_description.rviz')
    
    # Procesar XACRO para RVIZ
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), 
        value_type=str
    )
    
    # ========== NODOS ==========
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    joint_state_publisher_node_1 = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output='screen'
    )
    
    # Variables de entorno para Gazebo
    gazebo_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(resource_path))
    gazebo_env2 = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', str(resource_path))
    
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', sdf_path, '-v', '4'],
        output='screen'
    )
    
    # Bridge ROS2 ↔ Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/drone/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/drone/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/drone/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
            '/world/multicopter/model/drone/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        output='screen'
    )    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_env,
        gazebo_env2,
        robot_state_publisher,
        joint_state_publisher_node_1,
        gazebo,
        bridge,
        rviz2_node
    ])