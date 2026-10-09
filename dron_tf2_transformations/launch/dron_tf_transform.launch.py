from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Frame del cuerpo para ROS'
        ),
        
        # Nodo C++ (publica odom -> base_link con datos ya en ENU/FLU)
        # Node(
        #     package='dron_tf2_transformations',
        #     executable='odometry_tf_transformation',
        #     name='odometry_tf_transformation',
        #     parameters=[{
        #         'base_frame': LaunchConfiguration('base_frame'),
        #         'publish_rate': 50.0
        #     }],
        #     output='screen'
        # ),

        Node(
            package='dron_tf2_transformations',
            executable='odom_ros_2_px4',
            name='odom_ros_2_px4',
            parameters=[{
                'odom_topic': '/rtabmap/odom',
                'px4_odom_topic': '/fmu/in/vehicle_visual_odometry',
                'qos_reliability': 'best_effort',
                'timestamp_offset_us': 0,
                'quality': 100,
                'reset_counter': 0,
                'position_var_xyz': [0.01, 0.01, 0.01],
                'orientation_var_rpy': [0.01, 0.01, 0.01],
                'velocity_var_xyz': [0.01, 0.01, 0.01],
                'use_sim_time': False
            }],
        )
    ])