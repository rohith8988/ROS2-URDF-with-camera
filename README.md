# ROS 2 Workspace: `ros2course2_ws/src`

This workspace contains a ROS 2 Jazzy mobile robot with:
- Differential-drive base
- Camera sensor
- 2-DOF arm mounted on top of the base

## Packages
- `my_robot_description`: URDF/Xacro model, Gazebo model plugins, RViz config, display launch.
- `my_robot_bringup`: Gazebo world files, ROS<->Gazebo bridge config, Gazebo launch.

## Prerequisites
- ROS 2 Jazzy installed and sourced.
- `ros_gz_sim` and `ros_gz_bridge`.
- Standard tools: `xacro`, `robot_state_publisher`, `rviz2`, `joint_state_publisher_gui`.

## Build
```bash
cd ~/ros2course2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run
Gazebo + bridge + RViz:
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

Description only (no Gazebo):
```bash
ros2 launch my_robot_description display.launch.xml
```

## Arm Feature
The arm is attached to `base_link` and includes:
- `base_forearm` revolute joint
- `forearm_hand` revolute joint

Gazebo arm control is done with two `JointPositionController` plugins:
- `base_forearm` with `p_gain=5.0`
- `forearm_hand` with `p_gain=3.0`

## Arm Command Topics
Publish desired joint positions (radians):
```bash
ros2 topic pub -1 /arm/base_forearm/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"
ros2 topic pub -1 /arm/forearm_hand/cmd_pos std_msgs/msg/Float64 "{data: 0.7}"
```

## Key Files
- `my_robot_description/urdf/my_robot.urdf.xacro`: top-level robot model.
- `my_robot_description/urdf/mobile_base.xacro`: base and wheels.
- `my_robot_description/urdf/arm.xacro`: arm links and joints.
- `my_robot_description/urdf/mobile_base_gazebo.xacro`: diff-drive, joint state publisher, and arm joint position controllers.
- `my_robot_description/urdf/camera.xacro`: camera link and sensor.
- `my_robot_bringup/config/gazebo_bridge.yaml`: bridge mappings.
- `my_robot_bringup/worlds/new_world.sdf`: main Gazebo world.

## Bridged Topics
Gazebo -> ROS:
- `/clock`
- `/joint_states`
- `/camera/image_raw`
- `/camera/camera_info`
- `/tf` (currently enabled in bridge config)

ROS -> Gazebo:
- `/cmd_vel`
- `/arm/base_forearm/cmd_pos`
- `/arm/forearm_hand/cmd_pos`

## Troubleshooting
- If build fails with `ModuleNotFoundError: No module named 'catkin_pkg'`, install `python3-catkin-pkg` for the active Python used by `ament_cmake`.
- If arm links show `No transform` in RViz, verify `/joint_states` includes `base_forearm` and `forearm_hand`.
- If you see unexpected TF frames from other users/machines, isolate with `ROS_DOMAIN_ID` or `ROS_LOCALHOST_ONLY=1`.
