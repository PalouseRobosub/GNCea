#!/usr/bin/env python3


import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

GNCEA_PKG = "gncea_description"

def _nodes(context):
    # args
    model_path = LaunchConfiguration("model").perform(context)
    model_package = LaunchConfiguration("model_package").perform(context)
    world_path = LaunchConfiguration("world").perform(context)
    use_gui   = LaunchConfiguration("gui").perform(context).lower() in ("1","true","yes")
    use_rviz  = LaunchConfiguration("rviz").perform(context).lower() in ("1","true","yes")
    rviz_cfg  = LaunchConfiguration("rviz_config").perform(context)
    name      = LaunchConfiguration("name").perform(context)

    gncea_pkg_share = get_package_share_directory(GNCEA_PKG)
    urdf_pkg_share = get_package_share_directory(model_package)

    with open(model_path, "r") as f:
        urdf_txt = f.read()

    urdf_txt = urdf_txt.replace(f"package://{model_package}/", "file://" + urdf_pkg_share + "/")
    urdf_txt = urdf_txt.replace(f"model://{model_package}/",   "file://" + urdf_pkg_share + "/")

    # ros_home = os.getenv("ROS_HOME", os.path.expanduser("~/.ros"))
    # os.makedirs(ros_home, exist_ok=True)
    # tmp_urdf = os.path.join(ros_home, "guppy_abs.urdf")
    # with open(tmp_urdf, "w") as f:
    #     f.write(urdf_txt)

    nodes = []

    ros_gz = get_package_share_directory("ros_gz_sim")
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 3 {world_path}"}.items()
    ))

    if use_gui:
        nodes.append(Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="log",
        ))
    else:
        nodes.append(Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="log",
        ))

    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[{"robot_description": urdf_txt}],
    ))

    nodes.append(Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_guppy",
        output="log",
        arguments=["-name", name, "-file", tmp_urdf],
    ))

    if use_rviz:
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_cfg],
        ))

    return nodes

def generate_launch_description():
    gncea_pkg_share = get_package_share_directory(GNCEA_PKG)

    default_model = os.path.join(gncea_pkg_share, "urdf", "cube", "auv.urdf")
    default_world = os.path.join(gncea_pkg_share, "worlds", "woolletPool_world.sdf")
    default_rviz  = os.path.join(gncea_pkg_share, "rviz", "view_lidar.rviz")

    sim_models_install = os.path.join(gncea_pkg_share, "worldmodels", "Sim-Models")
    sim_models_src = os.path.join(
        os.path.dirname(gncea_pkg_share).replace("install", "src"),
        "gncea_description", "worldmodels", "Sim-Models"
    )
    gazebo_model_path = ":".join([
        sim_models_src,
        sim_models_install,
        os.environ.get("GAZEBO_MODEL_PATH", "")
    ])

    gz_resource_path = ":".join([
        sim_models_src,
        sim_models_install,
        gncea_pkg_share,
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ])

    ign_resource_path = ":".join([
        sim_models_src,
        sim_models_install,
        gncea_pkg_share,
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
        DeclareLaunchArgument("model_package", default_value="gncea_description", description="package containing URDF"),
        DeclareLaunchArgument("world", default_value=default_world, description="Path to Gazebo world SDF"),
        DeclareLaunchArgument("name",  default_value="auv",        description="Entity name in Gazebo"),
        DeclareLaunchArgument("gui",   default_value="false",         description="Use joint_state_publisher_gui"),
        DeclareLaunchArgument("rviz",  default_value="false",         description="Launch RViz2"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz, description="RViz config file"),
        OpaqueFunction(function=_nodes),
    ])
