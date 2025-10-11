# GNCea
GNCea | Auv Simulator Experimental 2 (Work In Progress)

An AUV simulator developed for Palouse RoboSub based on ROS 2 Jazzy and Gazebo Harmonic, featuring custom C++ plugins for thrust allocation, buoyancy compensation, and hydrodynamic drag modeling. The simulator supports real-time 6-DOF teleoperation, autonomy modules for depth and altitude regulation, and integrated onboard perception through a forward-facing camera and 3D LiDAR sensor for image and point cloud generation, enabling environmental mapping, navigation, and autonomy testing in dynamic underwater environments.

https://github.com/user-attachments/assets/06dadf81-c7be-401f-846d-b3b123910d5f

https://github.com/user-attachments/assets/2d361c2b-8436-4f90-ae8d-5ad14b407905

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

To launch guppy:

```
ros2 launch auv_description guppy_display.launch.py
```

To launch slider teleoperation:

```
ros2 run auv_description guppy_control_test.py
``
