from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    # Ruta del archivo XACRO
    urdf_path = os.path.join(
        get_package_share_path('drone_description'),
        'urdf', 'drone.urdf.xacro'
    )
    
    rviz_config_path=os.path.join(get_package_share_path('drone_description'), 
                                  'config', 'rviz2_config_description.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), 
        value_type=str
    )
    

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    joint_state_publisher_node_1=Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )


    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )
    

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "drone",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5"
        ],
        output='screen'
    )
    
    rviz2_node=Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_node_1,
        gazebo,
        spawn_entity,
        rviz2_node
    ])