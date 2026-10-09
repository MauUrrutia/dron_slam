from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='dron_tf2_transformations',
            executable='tf2_broadcaster.py',
            name='broadcaster',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
    ])