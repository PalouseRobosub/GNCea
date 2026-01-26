#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg_autonomy = 'gncea_autonomy'
    pkg_description = 'gncea_description'

    share_description = get_package_share_directory(pkg_description)
    prefix_description = get_package_prefix(pkg_description)

    world_path = os.path.join(share_description, 'worlds', 'woolletPool_testworld.sdf')

    xacro_file = os.path.join(share_description, 'urdf', 'cubeexperimental', 'auv.urdf')

    robot_description_cmd = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file
    ])

    gz_system_plugin_path = os.path.join(prefix_description, 'lib')

    worldmodels_path = os.path.join(share_description, 'worldmodels')
    sim_models_path = os.path.join(worldmodels_path, 'Sim-Models')
    src_worldmodels = worldmodels_path.replace('/install/', '/src/')
    src_sim_models = os.path.join(src_worldmodels, 'Sim-Models')

    gazebo_model_path = ":".join([p for p in [
        sim_models_path, src_sim_models, worldmodels_path,
        os.environ.get("GAZEBO_MODEL_PATH", "")
    ] if p and os.path.exists(p)])

    gz_resource_path = ":".join([p for p in [
        sim_models_path, src_sim_models, share_description, worldmodels_path,
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ] if p and os.path.exists(p)])


    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )


    spawn_auv = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_auv',
        output='screen',
        arguments=[
            '-world', 'woolletPool_testworld',
            '-name', 'auv',
            '-string', robot_description_cmd,
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '-0.5'
        ]
    )


    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera',
        output='screen',
        arguments=['/cube/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
    )

    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_info',
        output='screen',
        arguments=['/cube/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        output='screen',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
    )

    bridge_altimeter = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_altimeter',
        output='screen',
        arguments=['/altimeter@ros_gz_interfaces/msg/Altimeter@gz.msgs.Altimeter'],
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
    )

    bridge_force = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_force',
        output='screen',
        arguments=['/auve1/force_body@geometry_msgs/msg/Vector3@gz.msgs.Vector3d'],
    )

    bridge_torque = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_torque',
        output='screen',
        arguments=['/auve1/torque_body@geometry_msgs/msg/Vector3@gz.msgs.Vector3d'],
    )


    cmd_bridge_node = Node(
        package=pkg_description,
        executable='cmd_bridge_cpp',
        name='cmd_bridge',
        output='screen',
        parameters=[
            {'k_linear': 10.0},
            {'k_angular': 5.0},
            {'namespace': 'auve1'},
        ],
    )


    lane_node = Node(
        package=pkg_autonomy,
        executable='lane_navigator_cpp',
        name='lane_navigator',
        output='screen',
        parameters=[
            {'enable_debug': True},
            {'show_cv_windows': True},

            {'target_depth': 1.0},
            {'depth_increases_down': True},
            {'kp_depth': 4.0},
            {'ki_depth': 0.1},
            {'kd_depth': 0.5},
            {'max_heave_cmd': 2.0},

            {'kp_roll': 3.0},
            {'kd_roll': 0.8},
            {'kp_pitch': 3.0},
            {'kd_pitch': 0.8},
            {'max_att_cmd': 2.0},

            {'kp_yaw_px': 0.010},
            {'kd_yaw': 0.05},
            {'k_lane_theta': 0.8},
            {'max_yaw': 0.5},
            {'search_yaw': 0.20},

            {'forward_speed': 5.00}, #should be 1.75 by default, but 5.00 for testing
            {'forward_speed_search': 0.20},

            {'hsv_red1_low':  [0,   100, 100]},
            {'hsv_red1_high': [10,  255, 255]},
            {'hsv_red2_low':  [170, 100, 100]},
            {'hsv_red2_high': [180, 255, 255]},
            {'hsv_white_low':  [0,   0,   55]},
            {'hsv_white_high': [179, 45, 255]},

            {'min_contour_area': 300},
            {'min_aspect': 2.0},
            {'blur_ksize': 5},
            {'morph_ksize': 5},
        ],
    )

    return LaunchDescription([

        #  SetEnvironmentVariable('DISPLAY', ':1'),
        # SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        # SetEnvironmentVariable('GDK_BACKEND', 'x11'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),


        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_system_plugin_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

        LogInfo(msg=f"GAZEBO_MODEL_PATH: {gazebo_model_path}"),
        LogInfo(msg=f"GZ_SIM_SYSTEM_PLUGIN_PATH: {gz_system_plugin_path}"),
        LogInfo(msg=f"GZ_SIM_RESOURCE_PATH: {gz_resource_path}"),
        LogInfo(msg=f"Loading world: {world_path}"),
        LogInfo(msg=f"Loading xacro: {xacro_file}"),

        gz_sim,

        TimerAction(period=2.0, actions=[spawn_auv]),

        TimerAction(period=4.0, actions=[
            bridge_camera,
            bridge_camera_info,
            bridge_imu,
            bridge_altimeter,
            bridge_clock,
            bridge_force,
            bridge_torque,
        ]),

        TimerAction(period=6.0, actions=[cmd_bridge_node, lane_node]),
    ])
