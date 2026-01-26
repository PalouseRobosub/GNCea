#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction, LogInfo, GroupAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():
    pkg = 'gncea_description'

    share = get_package_share_directory(pkg)
    prefix = get_package_prefix(pkg)

    # Paths
    world_path = os.path.join(share, 'worlds', 'water_world.sdf')
    urdf_path  = os.path.join(share, 'urdf', 'cube', 'cube.urdf')

    # Where your plugin .so installs
    plugin_search_path = os.path.join(prefix, 'lib')

    # Start Gazebo Harmonic with your world
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )

    # Spawn the URDF cube
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cube',
        output='screen',
        arguments=[
            '-name', 'cube',
            '-file', urdf_path,
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.5'
        ]
    )

    # Camera bridge for image_raw and camera_info
    camera_bridge = GroupAction(actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_bridge',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            output='screen',
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('force', default_value='100.0'),
        DeclareLaunchArgument('rate_hz', default_value='30.0'),

        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_search_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share),

        LogInfo(msg=f'Loading world: {world_path}'),
        LogInfo(msg=f'Loading URDF:  {urdf_path}'),

        gz_server,

        TimerAction(period=2.0, actions=[spawn_cube]),

        TimerAction(period=4.0, actions=[camera_bridge])
    ])
