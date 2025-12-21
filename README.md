# GNCea
GNCea | Auv Simulator Experimental 2 (Work In Progress)

An AUV simulator developed for Palouse RoboSub based on ROS 2 Jazzy and Gazebo Harmonic, featuring custom C++ plugins for thrust allocation, buoyancy compensation, and hydrodynamic drag modeling. The simulator supports real-time 6-DOF teleoperation, autonomy modules for depth and altitude regulation, and integrated onboard perception through a forward-facing camera and 3D LiDAR sensor for image and point cloud generation, enabling environmental mapping, navigation, and autonomy testing in dynamic underwater environments.

<img width="1028" height="911" alt="image" src="https://github.com/user-attachments/assets/00de4348-a3af-4ce9-9a0a-f30b88ac70f2" />


<img width="1028" height="911" alt="image" src="https://github.com/user-attachments/assets/0adf25f2-3b9e-4b20-826e-90a6322660bf" />


https://github.com/user-attachments/assets/06dadf81-c7be-401f-846d-b3b123910d5f

https://github.com/user-attachments/assets/2d361c2b-8436-4f90-ae8d-5ad14b407905

# PREREQUISITES

Assuming ROS2 jazzy and gazebo harmonic is installed, other dependencies need to be installed to run this project

```
cd GNCea
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-xacro \
  python3-colcon-common-extensions python3-rosdep git && \
sudo rosdep init || true && rosdep update && \
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

To build:

```
colcon build
source install/setup.bash
```

Use whatever commands below to launch any desired launch file.

# cube.urdf, an example AUV plugin implementation that is simple, not parameterized

To launch

```
ros2 launch auv_description cube_thrust_test.launch.py
```

```
ros2 launch auv_description cube_thrust_test_water.launch.py
```

To activate teleoperation

```
ros2 run auv_description wasd_teleop.py   --ros-args -p topic:=/auve1/force_body -p force:=50.0 -p decay:=1.0 -p rate_hz:=500000000.0
```

force, decay and rate_hz can be edited as you see fit, just change the number in the teleop launch command.

Some commands I used to test stuff out:

```
ros2 run auv_description wasd_teleop.py   --ros-args     -p force_topic:=/auve1/force_body     -p force:=100.0     -p decay:=1.0     -p rate_hz:=12000000000.0 -p torque:=1.0
```

```
ros2 run auv_description wasd_teleop.py   --ros-args     -p force_topic:=/auve1/force_body     -p force:=500.0     -p decay:=1.0     -p rate_hz:=1200000000.0
```

# auv.urdf, an example AUV plugin implementation that is parameterized

To launch auv.urdf

```
ros2 launch auv_description testing.launch.py

```

Activate camera bridge

```
ros2 run ros_gz_bridge parameter_bridge   /cube/image_raw@sensor_msgs/msg/Image@gz.msgs.Image   /cube/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo   --ros-args -r /cube/image_raw:=/camera/image_raw -r /cube/camera_info:=/camera/camera_info
```

Show camera feed

```
ros2 run image_tools showimage -r image:=/camera/image_raw
```

Activate teleop using the previous commands

To test lidar

```
ros2 launch auv_description view_lidar.launch.py
```

Rviz will open up automatically, set fixed frame to auv/cube_link/lidar_link_sensor. Add pointcloud2 by topic, and set topic to scan/points.

Teleoperate using previous commands. 

To fire torpedos

```
ros2 topic pub /shoot_torpedo std_msgs/msg/Bool "data: true"
```

# GUPPY, AUV for 2026 robosub competition

To launch guppy:

```
ros2 launch auv_description guppy_display.launch.py
```

To launch slider teleoperation:

```
ros2 run auv_description guppy_control_test.py
```

# Experimental Control system environment (Gravity & Buoyancy turned off)

To launch experimental control world:

```
ros2 launch auv_description controlsystemExperimental.launch.py 
```

To use sliders:

```
ros2 run auv_description guppy_control_test.py
```

# Woollet Pool world

To launch Woollet pool world:

```
ros2 launch auv_description guppy_woolletpool.launch.py 
```

To use sliders:

```
ros2 run auv_description guppy_control_test.py
```

# TO ACTIVATE IMU/ALTIMETER ON GUPPY

activate IMU bridge:

```
ros2 run ros_gz_bridge parameter_bridge \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

activate altimeter bridge:

```
ros2 run ros_gz_bridge parameter_bridge \
  /altimeter@ros_gz_interfaces/msg/Altimeter@gz.msgs.Altimeter
```

bridge both IMU & altimeter at the same time:

```
ros2 run ros_gz_bridge parameter_bridge \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /altimeter@ros_gz_interfaces/msg/Altimeter@gz.msgs.Altimeter
```

# 2025 AUTONOMY CHALLENGE TESTING

This is my attempt at 2025s game. This was made so I could test opencv navigation and slaloming. 

To test autonomy test 2:

```
ros2 launch auv_autonomy autonomytest2.py
```



