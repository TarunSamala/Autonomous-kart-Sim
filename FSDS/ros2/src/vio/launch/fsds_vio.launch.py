from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf = LaunchConfiguration('publish_tf')
    point_cloud = LaunchConfiguration('point_cloud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the FSDS /clock topic.'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description=(
                'Publish vio/odom -> fsds/FSCar. Keep false while the FSDS '
                'ground-truth TF publisher is enabled.')),
        DeclareLaunchArgument(
            'point_cloud',
            default_value='true',
            description='Generate a registered colored point cloud.'),

        # FSDS v2.2 does not model a separately positioned camera IMU. This
        # adapter creates the correct software contract while preserving the
        # original chassis IMU topic for later outer-filter development.
        Node(
            package='sensor_interface',
            executable='imu_reframer',
            name='camera_imu_adapter',
            output='screen',
            parameters=[{
                'input_topic': '/fsds/imu',
                'output_topic': '/sensors/imu/camera',
                'target_frame': 'fsds/front_camera_imu',
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_camera_imu_tf',
            arguments=[
                '--x', '0.8', '--y', '0.0', '--z', '0.8',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'fsds/FSCar',
                '--child-frame-id', 'fsds/front_camera_imu',
            ],
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),

        Node(
            package='depth_image_proc',
            executable='point_cloud_xyzrgb_node',
            name='front_depth_point_cloud',
            output='screen',
            condition=IfCondition(point_cloud),
            parameters=[{
                'exact_sync': False,
                'queue_size': 20,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[
                ('rgb/image_rect_color', '/fsds/front_rgb/image_color'),
                ('rgb/camera_info', '/fsds/front_rgb/camera_info'),
                ('depth_registered/image_rect', '/fsds/front_depth/image_depth'),
                ('points', '/fsds/front_depth/points'),
            ],
        ),

        # This is the simulator baseline: RTAB-Map RGB-D odometry with IMU
        # initialization/gravity alignment. It validates synchronization,
        # calibration and interfaces before selecting a tightly coupled VIO
        # backend for the physical camera.
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='vio',
            output='screen',
            parameters=[{
                'frame_id': 'fsds/FSCar',
                'odom_frame_id': 'vio/odom',
                'publish_tf': ParameterValue(publish_tf, value_type=bool),
                'wait_for_transform': 0.2,
                'wait_imu_to_init': True,
                'always_check_imu_tf': True,
                'approx_sync': False,
                'topic_queue_size': 30,
                'sync_queue_size': 10,
                'qos': 2,
                'qos_camera_info': 2,
                'qos_imu': 2,
                'Odom/Strategy': '0',
                # D455 optimal working range. Keep the raw FSDS depth stream
                # unchanged so other consumers can choose their own limits.
                'Vis/MinDepth': '0.4',
                'Vis/MaxDepth': '6.0',
                'Vis/MinInliers': '15',
                'Odom/ResetCountdown': '1',
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[
                ('rgb/image', '/fsds/front_rgb/image_color'),
                ('depth/image', '/fsds/front_depth/image_depth'),
                ('rgb/camera_info', '/fsds/front_rgb/camera_info'),
                ('imu', '/sensors/imu/camera'),
                ('odom', '/localization/vio/odometry'),
            ],
        ),
    ])
