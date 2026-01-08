<p align="center">
  <img src="https://github.com/user-attachments/assets/ec2885ba-db83-4356-ac3a-4d351a17c34e"
       alt="GNCealogo"
       width="400" />
</p>


# GNCea | Guidance Navigation Control for SEA 

<p align="center">
    <img alt="GitHub last commit" src="https://img.shields.io/github/last-commit/palouserobosub/gncea">
    <img alt="ROS2 version" src="https://img.shields.io/badge/ros2-jazzy-red?logo=ros">
</p>

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
# TO ACTIVATE RGBD AND CAMERAS ON GUPPY

bridge rgbd on guppy:
```
ros2 run ros_gz_bridge parameter_bridge   /guppy/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked   /guppy/camera/image@sensor_msgs/msg/Image@gz.msgs.Image   /guppy/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image   /guppy/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

bridge corner cameras on guppy:
```
ros2 run ros_gz_bridge parameter_bridge \
  /guppy/raw_1/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /guppy/raw_1/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /guppy/raw_2/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /guppy/raw_2/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /guppy/raw_3/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /guppy/raw_3/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /guppy/raw_4/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /guppy/raw_4/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

# TO ECHO DVL ON GUPPY

<img width="800" height="676" alt="guppydvl" src="https://github.com/user-attachments/assets/ebaa00eb-71bb-4cbe-8e93-b735d6d3310a" />

```
gz topic -e -t /guppy/dvl
```

Please not that a node will be neccesary to bridge dvl data so that you can listen to the topic.

# 2025 AUTONOMY CHALLENGE TESTING

This is my attempt at 2025s game. This was made so I could test opencv navigation and slaloming. 

https://drive.google.com/file/d/1oP0x0Nx4gbNLD_p_7pBtzQwXXJvDacup/view?usp=sharing

To test autonomy test 2:

```
ros2 launch auv_autonomy autonomytest2.py
```

# Contact And Sponsorship

## Contact and Sponsorship
Need to get in touch? Reach out to `robosub.vcea@wsu.edu`.

We are sponsored by many generous companies and people, including:
- OSH Park PCBs
- Blue Robotics
- Real Digital
- Solidworks
- Vectornav
- JoeScan
- WaterLinked
- LattePanda
- Tektronix
- And many many other personal supporters through their generous donations!

Do you like our work? Consider [sponsoring](https://foundation.wsu.edu/give/?fund=ffdf2195-2497-4361-b697-44e5024bf0b0) our team!

