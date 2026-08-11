import launch
import launch_ros.actions

from os.path import expanduser
import json 

CAMERA_FRAMERATE = 30.0

def generate_launch_description():
    with open(expanduser("~")+'/Formula-Student-Driverless-Simulator/settings.json', 'r') as file:
        settings = json.load(file)

    camera_configs = settings['Vehicles']['FSCar']['Cameras']
    if(not camera_configs):
        print('no cameras configured in ~/Formula-Student-Driverless-Simulator/settings.json')

    camera_nodes = []
    paired_depth_cameras = {
        config.get("RosDepthCamera")
        for config in camera_configs.values()
        if config.get("RosDepthCamera")
    }

    for camera_name, camera_config in camera_configs.items():
        # A paired depth camera is captured by its RGB node in the same AirSim
        # simGetImages() request, so it must not get an independent timer/node.
        if camera_name in paired_depth_cameras:
            continue

        if not camera_config.get("RosEnabled", True):
            continue

        capture_config = camera_config["CaptureSettings"][0]
        depth_camera_name = camera_config.get("RosDepthCamera", "")
        if depth_camera_name and depth_camera_name not in camera_configs:
            raise RuntimeError(
                f"Camera '{camera_name}' references missing depth camera "
                f"'{depth_camera_name}'")

        if depth_camera_name:
            depth_capture = camera_configs[depth_camera_name]["CaptureSettings"][0]
            if capture_config["ImageType"] != 0 or depth_capture["ImageType"] != 2:
                raise RuntimeError(
                    f"RGB-D pair '{camera_name}'/'{depth_camera_name}' must use "
                    "ImageType 0 and 2")

        camera_nodes.append(launch_ros.actions.Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge_camera',
            namespace="fsds/camera", 
            name=camera_name,
            output='screen',
            parameters=[
                {'camera_name': camera_name},
                {'frame_id': camera_config.get("RosFrameId", f"fsds/{camera_name}")},
                {'depthcamera': capture_config["ImageType"] == 2},
                {'rgbd_mode': bool(depth_camera_name)},
                {'depth_camera_name': depth_camera_name},
                {'framerate': float(camera_config.get("RosFramerate", CAMERA_FRAMERATE))},
                {'fov_degrees': float(capture_config["FOV_Degrees"])},
                {'host_ip': launch.substitutions.LaunchConfiguration('host')},
            ]
        ))

    ld = launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mission_name',
            default_value='trackdrive'
        ),
        launch.actions.DeclareLaunchArgument(
            name='track_name',
            default_value='A'
        ),
        launch.actions.DeclareLaunchArgument(
            name='competition_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='manual_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='UDP_control',
            default_value='false'
        ),
        *camera_nodes,
        launch_ros.actions.Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge',
            name='ros_bridge',
            namespace='fsds',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'update_odom_every_n_sec': 0.004
                },
                {
                    'update_imu_every_n_sec': 0.004
                },
                {
                    'update_gps_every_n_sec': 0.1
                },
                {
                    'update_gss_every_n_sec': 0.01
                },
                {
                    'publish_static_tf_every_n_sec': 1.0
                },
                {
                    'update_lidar_every_n_sec': 0.1
                },
                {
                    'host_ip': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'mission_name': launch.substitutions.LaunchConfiguration('mission_name')
                },
                {
                    'track_name': launch.substitutions.LaunchConfiguration('track_name')
                },
                {
                    'competition_mode': launch.substitutions.LaunchConfiguration('competition_mode')
                },
                {
                    'manual_mode': launch.substitutions.LaunchConfiguration('manual_mode')
                },
                {
                    'UDP_control': launch.substitutions.LaunchConfiguration('UDP_control')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
