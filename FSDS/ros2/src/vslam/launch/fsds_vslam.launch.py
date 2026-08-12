from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    point_cloud = LaunchConfiguration('point_cloud')
    localization = LaunchConfiguration('localization')
    gui = LaunchConfiguration('gui')
    database_path = LaunchConfiguration('database_path')
    delete_db = LaunchConfiguration('delete_db')

    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('vio'),
            '/launch/fsds_vio.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # VSLAM publishes map -> vio/odom; VIO completes the chain with
            # vio/odom -> fsds/FSCar.
            'publish_tf': 'true',
            'point_cloud': point_cloud,
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('rtabmap_launch'),
            '/launch/rtabmap.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'rtabmap_viz': gui,
            'rviz': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'depth': 'true',
            'stereo': 'false',
            'subscribe_rgb': 'true',
            'subscribe_rgbd': 'false',
            'frame_id': 'fsds/FSCar',
            # Empty means consume the odometry topic. RTAB-Map still uses the
            # message's child/parent frame IDs to publish map -> vio/odom.
            'odom_frame_id': '',
            'map_frame_id': 'map',
            'publish_tf_map': 'true',
            'database_path': database_path,
            'rgb_topic': '/fsds/front_rgb/image_color',
            'depth_topic': '/fsds/front_depth/image_depth',
            'camera_info_topic': '/fsds/front_rgb/camera_info',
            'odom_topic': '/localization/vio/odometry',
            'imu_topic': '/sensors/imu/camera',
            'approx_sync': 'false',
            'qos': '2',
            'topic_queue_size': '30',
            'sync_queue_size': '20',
            'namespace': 'vslam',
            'args': PythonExpression([
                "'-d' if '", delete_db, "' == 'true' else ''",
            ]),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the FSDS /clock topic.'),
        DeclareLaunchArgument(
            'point_cloud',
            default_value='false',
            description='Also publish the live registered colored cloud.'),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Load the existing database without adding map nodes.'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Open the RTAB-Map visualization UI.'),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/fsds_vslam.db',
            description='Persistent RTAB-Map database.'),
        DeclareLaunchArgument(
            'delete_db',
            default_value='false',
            description='Delete the selected database before starting.'),
        vio_launch,
        rtabmap_launch,
    ])
