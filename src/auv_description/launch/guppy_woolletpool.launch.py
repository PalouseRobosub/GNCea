#!/usr/bin/env python3
# Launch guppy in Gazebo Harmonic (ROS 2 Jazzy) in a portable way.

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

PKG = "auv_description"

def _nodes(context):
    # args
    model_path = LaunchConfiguration("model").perform(context)
    world_path = LaunchConfiguration("world").perform(context)
    use_gui   = LaunchConfiguration("gui").perform(context).lower() in ("1","true","yes")
    use_rviz  = LaunchConfiguration("rviz").perform(context).lower() in ("1","true","yes")
    rviz_cfg  = LaunchConfiguration("rviz_config").perform(context)
    name      = LaunchConfiguration("name").perform(context)

    pkg_share = get_package_share_directory(PKG)

    # 1) Read URDF
    with open(model_path, "r") as f:
        urdf_txt = f.read()

    # 2) Replace package:// & model://<PKG>/ with absolute paths
    urdf_txt = urdf_txt.replace(f"package://{PKG}/", pkg_share + "/")
    urdf_txt = urdf_txt.replace(f"model://{PKG}/",   pkg_share + "/")

    # 3) Temp URDF for the spawner
    ros_home = os.getenv("ROS_HOME", os.path.expanduser("~/.ros"))
    os.makedirs(ros_home, exist_ok=True)
    tmp_urdf = os.path.join(ros_home, "guppy_abs.urdf")
    with open(tmp_urdf, "w") as f:
        f.write(urdf_txt)

    nodes = []

    # Gazebo (ros_gz_sim)
    ros_gz = get_package_share_directory("ros_gz_sim")
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 3 {world_path}"}.items()
    ))

    # # Joint state publisher
    # if use_gui:
    #     nodes.append(Node(
    #         package="joint_state_publisher_gui",
    #         executable="joint_state_publisher_gui",
    #         name="joint_state_publisher_gui",
    #         output="screen",
    #     ))
    # else:
    #     nodes.append(Node(
    #         package="joint_state_publisher",
    #         executable="joint_state_publisher",
    #         name="joint_state_publisher",
    #         output="screen",
    #     ))

    # # Robot state publisher
    # nodes.append(Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="screen",
    #     parameters=[{"robot_description": urdf_txt}],
    # ))

    # Spawn URDF in Gazebo
    nodes.append(Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_guppy",
        output="screen",
        arguments=["-name", name, "-file", tmp_urdf],
    ))

    # # RViz2 (optional)
    # if use_rviz:
    #     nodes.append(Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         name="rviz2",
    #         output="screen",
    #         arguments=["-d", rviz_cfg],
    #     ))

    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    default_model = os.path.join(pkg_share, "urdf", "guppyExperimental", "guppytest1Experimental.urdf")
    default_world = os.path.join(pkg_share, "worlds", "woolletPool_world.sdf")
    default_rviz  = os.path.join(pkg_share, "rviz", "view_lidar.rviz")

    # Paths that contain 'woollett_pool_2024' model folder
    sim_models_install = os.path.join(pkg_share, "worldmodels", "Sim-Models")
    # Try to derive the source-space Sim-Models as well (useful while developing)
    sim_models_src = os.path.join(
        os.path.dirname(pkg_share).replace("install", "src"),
        "auv_description", "worldmodels", "Sim-Models"
    )

    # Build env vars:
    # - GAZEBO_MODEL_PATH: critical for resolving model://<name>/<mesh>
    # - GZ_SIM_RESOURCE_PATH & IGN_GAZEBO_RESOURCE_PATH: helpful for worlds/plugins/resources
    gazebo_model_path = ":".join([
        sim_models_src,
        sim_models_install,
        os.environ.get("GAZEBO_MODEL_PATH", "")
    ])

    gz_resource_path = ":".join([
        sim_models_src,
        sim_models_install,
        pkg_share,
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ])

    ign_resource_path = ":".join([
        sim_models_src,
        sim_models_install,
        pkg_share,
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ])

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=gazebo_model_path
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_resource_path
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=ign_resource_path
    )

    return LaunchDescription([
        set_gazebo_model_path,
        set_gz_resource_path,
        set_ign_resource_path,
        DeclareLaunchArgument("model", default_value=default_model, description="Path to URDF"),
        DeclareLaunchArgument("world", default_value=default_world, description="Path to Gazebo world SDF"),
        DeclareLaunchArgument("name",  default_value="guppy",        description="Entity name in Gazebo"),
        DeclareLaunchArgument("gui",   default_value="true",         description="Use joint_state_publisher_gui"),
        DeclareLaunchArgument("rviz",  default_value="true",         description="Launch RViz2"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz, description="RViz config file"),
        OpaqueFunction(function=_nodes),
    ])
