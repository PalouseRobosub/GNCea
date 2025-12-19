#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import FindExecutable

from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg_autonomy = 'auv_autonomy'
    pkg_description = 'auv_description'

    share_description = get_package_share_directory(pkg_description)
    prefix_description = get_package_prefix(pkg_description)

    # ---------------- World ----------------
    world_path = os.path.join(share_description, 'worlds', 'woolletPool_testworld.sdf')

    # ---------------- Robot (this file is xacro even if named .urdf) ----------------
    xacro_file = os.path.join(share_description, 'urdf', 'cubeexperimental', 'auv.urdf')

    # Override xacro properties here (important: ensure leading "/" matches your bridges)
    robot_description_cmd = Command([
        FindExecutable(name='xacro'),
        ' ', xacro_file,
        ' ', 'wasd_topic:=/auve1/force_body',
        # optional overrides:
        # ' ', 'cam_topic:=/cube/image_raw',
        # ' ', 'cube_mass:=989.9',
    ])

    # Gazebo system plugin search path (for your custom .so’s)
    plugin_search_path = os.path.join(prefix_description, 'lib')
    gz_system_plugin_path = ":".join(filter(None, [
        plugin_search_path,
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    ]))

    # ---------------- Model / resource paths ----------------
    worldmodels_path = os.path.join(share_description, 'worldmodels')
    sim_models_path = os.path.join(worldmodels_path, 'Sim-Models')

    # allow running from install/ or src/
    src_worldmodels = worldmodels_path.replace('/install/', '/src/')
    src_sim_models = os.path.join(src_worldmodels, 'Sim-Models')

    gazebo_model_path = ":".join(filter(None, [
        sim_models_path if os.path.exists(sim_models_path) else "",
        src_sim_models if os.path.exists(src_sim_models) else "",
        worldmodels_path if os.path.exists(worldmodels_path) else "",
        os.environ.get("GAZEBO_MODEL_PATH", ""),
    ]))

    gz_resource_path = ":".join(filter(None, [
        sim_models_path if os.path.exists(sim_models_path) else "",
        src_sim_models if os.path.exists(src_sim_models) else "",
        share_description if os.path.exists(share_description) else "",
        worldmodels_path if os.path.exists(worldmodels_path) else "",
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]))

    # ---------------- Start Gazebo ----------------
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )

    # ---------------- Spawn the AUV ----------------
    spawn_auv = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_auv',
        output='screen',
        arguments=[
            '-name', 'auv',
            '-string', robot_description_cmd,
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '-0.5'
        ]
    )

    # ---------------- Bridges ----------------
    bridges = [
        # Camera RGB
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_camera',
            output='screen',
            arguments=['/cube/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_camera_info',
            output='screen',
            arguments=['/cube/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        ),

        # Depth image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_depth',
            output='screen',
            arguments=['/cube/depth@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),

        # Lidar
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_lidar',
            output='screen',
            arguments=['/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        ),

        # Force/Torque (bi-directional; your cmd_bridge publishes ROS -> bridge -> GZ)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_force',
            output='screen',
            arguments=['/auve1/force_body@geometry_msgs/msg/Vector3@gz.msgs.Vector3d'],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_torque',
            output='screen',
            arguments=['/auve1/torque_body@geometry_msgs/msg/Vector3@gz.msgs.Vector3d'],
        ),

        # Clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_clock',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        ),

        # IMU + Altimeter  ✅ Altimeter uses ros_gz_interfaces
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_imu_alt',
            output='screen',
            arguments=[
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/altimeter@ros_gz_interfaces/msg/Altimeter@gz.msgs.Altimeter',
            ],
        ),
    ]

    # ---------------- cmd_vel -> force/torque ----------------
    cmd_bridge_node = Node(
        package=pkg_description,
        executable='cmd_bridge_cpp',
        name='cmd_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'k_linear': 10.0},
            {'k_angular': 5.0},
            {'namespace': 'auve1'},
        ],
    )

    # ---------------- Gate Navigator ----------------
    gate_navigator_node = Node(
        package=pkg_autonomy,
        executable='gate_navigator_node.py',
        name='gate_navigator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'enable_debug': True},

            {'target_depth': 1.0},
            {'forward_speed': 0.30},
            {'search_yaw': 0.25},
            {'max_yaw': 0.60},
            {'kp_yaw_gate': 0.003},
            {'center_tol_px': 25},

            {'pass_width_px': 260},
            {'pass_time_s': 1.0},

            {'kp_depth': 4.0},
            {'ki_depth': 0.1},
            {'kd_depth': 0.5},

            {'hsv_red1_low':  [0,   100, 100]},
            {'hsv_red1_high': [10,  255, 255]},
            {'hsv_red2_low':  [170, 100, 100]},
            {'hsv_red2_high': [180, 255, 255]},

            {'hsv_white_low':  [0,   0,   200]},
            {'hsv_white_high': [180, 40,  255]},

            {'min_contour_area': 300},
            {'min_aspect': 2.0},
            {'blur_ksize': 5},
            {'morph_ksize': 5},
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_system_plugin_path),

        LogInfo(msg=f"Loading world: {world_path}"),
        LogInfo(msg=f"Loading xacro: {xacro_file}"),
        LogInfo(msg=f"GAZEBO_MODEL_PATH: {gazebo_model_path}"),
        LogInfo(msg=f"GZ_SIM_RESOURCE_PATH: {gz_resource_path}"),
        LogInfo(msg=f"GZ_SIM_SYSTEM_PLUGIN_PATH: {gz_system_plugin_path}"),

        gz_sim,
        TimerAction(period=2.0, actions=[spawn_auv]),
        TimerAction(period=4.0, actions=bridges),
        TimerAction(period=6.0, actions=[cmd_bridge_node, gate_navigator_node]),
    ])
