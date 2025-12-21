#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, LogInfo
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    pkg = 'auv_description'
    share = get_package_share_directory(pkg)
    prefix = get_package_prefix(pkg)

    world      = os.path.join(share, 'worlds', 'water_world.sdf')
    xacro_path = os.path.join(share, 'urdf', 'cube', 'auv.urdf')
    rviz_cfg   = os.path.join(share, 'rviz', 'view_lidar.rviz')

    robot_description = ParameterValue(Command(['xacro', ' ', xacro_path]), value_type=str)

    plugin_path = os.path.join(prefix, 'lib')

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world],
        output='screen'
    )

    spawn_auv = Node(
        package='ros_gz_sim', executable='create', name='spawn_auv', output='screen',
        arguments=[
            '-name', 'auv',
            '-string', Command(['xacro', ' ', xacro_path]),
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.5'
        ]
    )

    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge',
        output='screen',
        arguments=[
            '/lidar/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '--ros-args', '-r', '/lidar/points/points:=/scan/points'
        ]
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    lidar_sensor_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lidar_sensor_tf',
        arguments=['0','0','0','0','0','0','lidar_link','auv/cube_link/lidar_link_sensor'],
        output='screen'
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',      share),
        LogInfo(msg=f'Loading world: {world}'),

        gz_server,
        TimerAction(period=2.0, actions=[spawn_auv]),
        TimerAction(period=3.0, actions=[lidar_bridge, rsp, lidar_sensor_tf]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
