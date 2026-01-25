#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    pkg = 'gncea_description'
    share = get_package_share_directory(pkg)
    prefix = get_package_prefix(pkg)

    world_path = os.path.join(share, 'worlds', 'water_world.sdf')
    xacro_file = os.path.join(share, 'urdf', 'cube', 'auv.urdf')  

    robot_description_cmd = Command(['xacro ', xacro_file])
    plugin_search_path = os.path.join(prefix, 'lib')

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )

    spawn_auv = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_auv',
        output='screen',
        arguments=[
            '-name', 'auv',
            '-string', robot_description_cmd,
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.5'
        ]
    )

    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='image_bridge',
        output='screen',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
    )
    caminfo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        output='screen',
        arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/lidar/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '--ros-args', '-r', '/lidar/points/points:=/scan/points'
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_search_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share),

        LogInfo(msg=f'Loading world: {world_path}'),
        LogInfo(msg='Processing xacro and launching AUV'),

        gz_server,

        TimerAction(period=2.0, actions=[spawn_auv]),

        TimerAction(period=4.0, actions=[image_bridge, caminfo_bridge, lidar_bridge]),
    ])
