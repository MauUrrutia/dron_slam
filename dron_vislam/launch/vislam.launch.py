from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Ajusta esta ruta según dónde guardaste bonxai_params.yaml dentro de tu paquete ROS 2
    # Ejemplo recomendado: <tu_paquete>/config/bonxai_params.yaml

    
    realsense_params = {
        'camera_name': 'd435i',
        'align_depth.enable': True,
        'depth_module.depth_profile': '848x480x30',
        'rgb_camera.color_profile': '848x480x30',
        'enable_color': True,
        'enable_depth': True,
        'accelerate_gpu_with_glsl': True,
        'publish_tf': True,
        # 'base_frame_id': 'base_footprint',
        'enable_sync': True,
        'enable_gyro': True,
        'enable_accel': True,
        'pointcloud__neon_.enable': True,
        'pointcloud__neon_.stream_profile': 1,
        'pointcloud__neon_.stream_filter': 2,
        'pointcloud__neon_.stream_index_filter': 0,
        'pointcloud__neon_.ordered_pc': False,
        'pointcloud__neon_.allow_no_texture_points': False,
        'depth_module.enable_auto_exposure': False,
        'unite_imu_method': 2,
    }

    bonxai_params_file = os.path.join(
        get_package_share_directory('rolling_bonxai_ros'),
        'params',
        'bonxai_dron_params.yaml'
    )

    rgb_topic = '/camera/camera/color/image_raw'
    depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
    camera_info_topic = '/camera/camera/color/camera_info'
    imu_topic = '/rtabmap/imu'
    # Este será el tópico de odometría generado por RTAB-Map.
    # Posteriormente puedes usarlo como entrada para robot_localization,
    # otro filtro EKF, etc.
    odom_topic = '/rtabmap/odom'
    # Frame base de la cámara/robot.
    # Debe existir una transformación TF entre este frame y los frames
    # de la cámara publicados por RealSense.
    frame_id = 'base_footprint'
    odom_frame_id = 'odom'
    map_frame_id = 'map'

    rtabmap_odom_params = {
        'frame_id': frame_id,
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_imu': True,
        'wait_imu_to_init': True,
        'subscribe_odom_info': True,
        'use_sim_time': False,
        'approx_sync': False,
        'approx_sync_max_interval': 0.03,
        'sync_queue_size': 10,
        'topic_queue_size': 10,
        'wait_for_transform': 0.5,
        'Rtabmap/ImagesAlreadyRectified': 'true',
        'publish_tf': True,
        'Vis/FeatureType': '8',
        'Vis/EstimationType': '1',
        'Vis/MinInliers': '12',
        'Vis/MaxFeatures': '1500',
        'Vis/CorType': '0',
        'Odom/ResetCountdown': '0',
        'Odom/Strategy': '0',
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '5',
        'GFTT/QualityLevel': '0.00001',
        'Stereo/MaxDisparity': '128',
        'always_process_most_recent_frame': False,
    }
    slam_parameters = {
        # Frames
        'frame_id': frame_id,
        'map_frame_id': map_frame_id,
        'odom_frame_id': odom_frame_id,

        # Entrada RGB-D
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_imu': True,
        'subscribe_odom_info': True,

        # Entrada de odometría
        'odom_topic': odom_topic,

        # Sincronización
        'approx_sync': False,
        'queue_size': 20,
        'wait_for_transform': 0.2,

        # Publicación de TF.
        # Este nodo publica map -> odom.
        'publish_tf': True,
        'publish_tf_map': True,

        # Base de datos
        'database_path': os.path.expanduser('~/.ros/rtabmap_dron.db'),

        # Configuración SLAM
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'true',
        'Mem/STMSize': '10',

        # Gestión de memoria
        'Rtabmap/DetectionRate': '2.0',
        'Rtabmap/TimeThr': '0',
        'Rtabmap/PublishStats': 'true',

        # Cierre de bucles
        'RGBD/LoopClosureReextractFeatures': 'true',
        'RGBD/LoopClosureIdentityGuess': 'true',

        # Parámetros de visualización/publicación
        'Grid/FromDepth': 'true',
        'Grid/3D': 'true',
        'Grid/RangeMax': '5.0',
    }

    ekf_yaml = PathJoinSubstitution([
        FindPackageShare('dron_vislam'), 
        'config',
        'ekf.yaml'
    ])

    
    
    
    return LaunchDescription([
        # 1. RealSense Camera 
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            # name='d435i',
            output='screen',
            parameters=[realsense_params]
        ),

        # 2. IMU Filter (filter IMU data)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('/imu/data_raw', '/camera/camera/imu'),
                ('/imu/data', '/rtabmap/imu')
            ]
        ),
        
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            emulate_tty=True,

            parameters=[rtabmap_odom_params],

            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('imu', imu_topic),

                # Tópicos de salida
                ('odom', odom_topic),
                ('odom_info', '/rtabmap/odom_info'),
            ]
        ),
        
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            emulate_tty=True,

            parameters=[slam_parameters],

            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),

                ('imu', imu_topic),

                ('odom', odom_topic),
                ('odom_info', '/rtabmap/odom_info'),
            ],

            arguments=[
                '--delete_db_on_start'
            ],
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[ekf_yaml],
        #     remappings=[
        #         ('odometry/filtered', '/odometry/filtered')
        #     ]
        # ),
        

        
    ])

