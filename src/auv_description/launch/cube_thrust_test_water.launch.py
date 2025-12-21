#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():
    pkg = 'auv_description'

    share = get_package_share_directory(pkg)
    prefix = get_package_prefix(pkg)

    # Paths
    world_path = os.path.join(share, 'worlds', 'water_world.sdf')
    urdf_path  = os.path.join(share, 'urdf', 'cube', 'cube.urdf')

    plugin_search_path = os.path.join(prefix, 'lib')

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )

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

    # Keyboard teleop – absolute topic matches your URDF plugin param
    # teleop = Node(
    #     package=pkg,
    #     executable='wasd_teleop.py',
    #     name='wasd_teleop',
    #     output='screen',
    #     parameters=[{
    #         'topic': '/auve1/force_body',
    #         'force': LaunchConfiguration('force'),
    #         'rate_hz': LaunchConfiguration('rate_hz'),
    #         'decay': 0.92
    #     }]
    # )

    return LaunchDescription([
        DeclareLaunchArgument('force', default_value='100.0'),
        DeclareLaunchArgument('rate_hz', default_value='30.0'),

        # Env so Gazebo can find your plugin + package resources
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_search_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share),

        LogInfo(msg=f'Loading world: {world_path}'),
        LogInfo(msg=f'Loading URDF:  {urdf_path}'),

        gz_server,

        # Give the server a moment to advertise services before spawning
        TimerAction(period=2.0, actions=[spawn_cube]),

        # teleop,
    ])
