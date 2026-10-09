import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Ruta del launch de VIsLAM
    vislam_launch_path = os.path.join(
        get_package_share_path('dron_vislam'),
        'launch',
        'vislam.launch.py'
    )

    vislam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vislam_launch_path)
    )

    # Nodo para transformar la odometría a TF
    odometry_tf_transformation_node = Node(
        package='dron_tf2_transformations',
        executable='odometry_tf_transformation',
        name='odometry_tf_transformation',
        output='screen'
    )

    # Ruta del archivo URDF/Xacro
    urdf_path = os.path.join(
        get_package_share_path('drone_description'),
        'urdf',
        'drone.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            urdf_path
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
        output='screen'
    )

    offboard_control_setpoint_node = Node(
        package='dron_control',
        executable='offboard_control_setpoint',
        name='offboard_control_setpoint',
        output='screen'
    )

    return LaunchDescription([
        vislam_launch,
        # odometry_tf_transformation_node,
        # offboard_control_setpoint_node,
        robot_state_publisher_node
    ])