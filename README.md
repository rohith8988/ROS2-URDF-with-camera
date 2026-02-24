# ROS 2 Workspace: `ros2course2_ws/src`

This workspace contains a small mobile robot description package and a Gazebo bringup package for ROS 2 Jazzy.

<img width="2321" height="1844" alt="image" src="https://github.com/user-attachments/assets/740c19d2-8d0a-4ecc-addf-863029d79f92" />
<img width="3659" height="2215" alt="image" src="https://github.com/user-attachments/assets/fab6ebc8-5f80-4ee4-8d8d-e487ea25a4d4" />



**Packages**
- `my_robot_description`: URDF/Xacro, RViz config, and a display launch file.
- `my_robot_bringup`: Gazebo world(s), bridge config, and a Gazebo launch file.

**Prerequisites**
- ROS 2 Jazzy installed and sourced.
- `ros_gz_sim` and `ros_gz_bridge` installed (for Gazebo).
- Standard ROS tools: `xacro`, `robot_state_publisher`, `rviz2`, `joint_state_publisher_gui`.

**Build**
```bash
cd ~/ros2course2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

**Run**
Gazebo bringup:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

Description display (no Gazebo):
```bash
ros2 launch my_robot_description display.launch.xml
```

**Key Files**
- `my_robot_description/urdf/my_robot.urdf.xacro`: Robot description entry point.
- `my_robot_description/urdf/mobile_base.xacro`: Base and wheels.
- `my_robot_description/urdf/camera.xacro`: Camera link and Gazebo sensor.
- `my_robot_description/rviz/my_robot.rviz`: RViz configuration.
- `my_robot_bringup/config/gazebo_bridge.yaml`: ROS <-> Gazebo topic bridge.
- `my_robot_bringup/worlds/new_world.sdf`: Default Gazebo world.
- `my_robot_bringup/worlds/jetty.sdf`: Alternate world.

**Topics**
The Gazebo bridge publishes:
- `/clock` (simulation time)
- `/joint_states`
- `/camera/image_raw`
- `/camera/camera_info`

The bridge subscribes:
- `/cmd_vel` (velocity commands)

**Troubleshooting**
- If `colcon build` fails with `ModuleNotFoundError: No module named 'catkin_pkg'`, install the missing Python module for the Python `ament_cmake` is using (often `sudo apt-get install python3-catkin-pkg`).
- If you see TF timing warnings in RViz, check that all nodes use the same time source (`/clock` + `use_sim_time=true` when running Gazebo).
