cmake_minimum_required(VERSION 3.8)
project(auv_description)

# C++ settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Gazebo Harmonic (gz-sim 8)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# --- Plugin: WASD body thrust ---
add_library(wasd_body_thrust_plugin SHARED
  src/wasd_body_thrust_plugin.cpp
)
ament_target_dependencies(wasd_body_thrust_plugin
  rclcpp
  geometry_msgs
  gz-sim8
  gz-plugin2
  gz-math7
)

install(TARGETS wasd_body_thrust_plugin
  EXPORT wasd_body_thrust_pluginTargets
  LIBRARY DESTINATION lib
)

# --- Plugin: torpedo shooter ---
add_library(torpedo_shooter SHARED src/gazebo_ros_torpedo_shooter.cpp)

ament_target_dependencies(torpedo_shooter
  rclcpp
  std_msgs
  gz-sim8
  gz-plugin2
  gz-common5
)


install(TARGETS torpedo_shooter
  EXPORT torpedo_shooterTargets
  LIBRARY DESTINATION lib
)

# --- Plugin: cmd_bridge_cpp ---
add_executable(cmd_bridge_cpp src/cmd_bridge_cpp.cpp)
ament_target_dependencies(cmd_bridge_cpp rclcpp geometry_msgs)

install(
  TARGETS cmd_bridge_cpp
  DESTINATION lib/${PROJECT_NAME}
)


# --- Plugin: Simple buoyancy ---
add_library(simple_buoyancy SHARED
  src/simple_buoyancy.cc
)
ament_target_dependencies(simple_buoyancy
  rclcpp
  geometry_msgs
  gz-sim8
  gz-plugin2
  gz-math7
)

install(TARGETS simple_buoyancy
  EXPORT simple_buoyancyTargets
  LIBRARY DESTINATION lib
)

# --- Install worldmodels ---
install(
  DIRECTORY worldmodels/
  DESTINATION share/${PROJECT_NAME}/worldmodels
)


# --- Keyboard / Slider teleop script ---
install(PROGRAMS
  scripts/wasd_teleop.py
  scripts/guppy_control_test.py
  scripts/cmd_bridge.py
  scripts/cmd_wasd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

#CPP teleop script
add_executable(cmd_wasd_teleop scripts/cmd_wasd_teleop.cpp)
ament_target_dependencies(cmd_wasd_teleop rclcpp geometry_msgs)
install(TARGETS cmd_wasd_teleop DESTINATION lib/${PROJECT_NAME})

# --- Share description assets ---
install(DIRECTORY urdf meshes launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(wasd_body_thrust_pluginTargets)
ament_export_libraries(wasd_body_thrust_plugin)

ament_export_targets(simple_buoyancyTargets)
ament_export_libraries(simple_buoyancy)

ament_package()
