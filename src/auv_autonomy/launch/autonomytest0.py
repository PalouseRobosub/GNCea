#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Package names
    pkg_autonomy = 'auv_autonomy'
    pkg_description = 'auv_description'
    
    # Get paths from auv_description (where the robot models are)
    share_description = get_package_share_directory(pkg_description)
    prefix_description = get_package_prefix(pkg_description)
    
    # Paths to world and robot files
    world_path = os.path.join(share_description, 'worlds', 'woolletPool_testworld.sdf')
    xacro_file = os.path.join(share_description, 'urdf', 'cube', 'auv.urdf')
    
    robot_description_cmd = Command(['xacro ', xacro_file])
    plugin_search_path = os.path.join(prefix_description, 'lib')
    
    # CRITICAL: Set up model paths for Gazebo to find woollett_pool_2024
    # Path to worldmodels containing the pool model
    worldmodels_path = os.path.join(share_description, 'worldmodels')
    sim_models_path = os.path.join(worldmodels_path, 'Sim-Models')
    
    # Try to find source directory (useful during development)
    src_worldmodels = worldmodels_path.replace('/install/', '/src/')
    src_sim_models = os.path.join(src_worldmodels, 'Sim-Models')
    
    # Build comprehensive model paths
    gazebo_model_path = ":".join(filter(os.path.exists, [
        sim_models_path,
        src_sim_models,
        worldmodels_path,
        os.environ.get("GAZEBO_MODEL_PATH", "")
    ]))
    
    gz_resource_path = ":".join(filter(os.path.exists, [
        sim_models_path,
        src_sim_models,
        share_description,
        worldmodels_path,
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ]))
    
    # 1) Gazebo
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )
    
    # 2) Spawn the AUV from the processed xacro string
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
    
    # 3) Camera bridges (RGB, Depth, and Camera Info)
    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='image_bridge',
        output='screen',
        arguments=['/cube/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
    )
    
    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_bridge',
        output='screen',
        arguments=['/cube/depth@sensor_msgs/msg/Image@gz.msgs.Image'],
    )
    
    caminfo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        output='screen',
        arguments=['/cube/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
    )
    
    # 4) LiDAR bridge: GZ PointCloudPacked -> ROS PointCloud2
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
    )
    
    # 5) Force/Torque bridges for control
    force_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='force_bridge',
        output='screen',
        arguments=[
            '/auve1/force_body@geometry_msgs/msg/Vector3@gz.msgs.Vector3d',
        ],
    )
    
    torque_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='torque_bridge',
        output='screen',
        arguments=[
            '/auve1/torque_body@geometry_msgs/msg/Vector3@gz.msgs.Vector3d',
        ],
    )
    
    # 6) Clock bridge for sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )
    
    return LaunchDescription([
        # CRITICAL: Set GAZEBO_MODEL_PATH so Gazebo can find woollett_pool_2024
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_search_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        
        LogInfo(msg=f'GAZEBO_MODEL_PATH: {gazebo_model_path}'),
        LogInfo(msg=f'Loading world: {world_path}'),
        LogInfo(msg=f'Loading robot: {xacro_file}'),
        
        gz_server,
        
        # Give the server a moment, then spawn the robot
        TimerAction(period=2.0, actions=[spawn_auv]),
        
        # After the robot exists, start bridges (each node only ONCE)
        TimerAction(period=4.0, actions=[
            image_bridge,
            depth_bridge,
            caminfo_bridge,
            lidar_bridge,
            force_bridge,
            torque_bridge,
            clock_bridge,
        ]),
    ])