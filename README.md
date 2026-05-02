# Mobile Robot ROS 2 Workspace

This repository contains a ROS 2 workspace for simulating and visualizing a differential-drive mobile robot in Gazebo. The robot model includes wheel odometry, an IMU, magnetometer, stereo camera, PTZ-style Arducam camera, and 2D lidar. The package also includes small Python driving nodes for basic motion, obstacle avoidance, and wall-following behavior.

The main package is `mobile_robot`, located under `src/mobile_robot`.

## What It Does

- Defines a differential-drive robot using Xacro/URDF.
- Adds Gazebo simulation plugins and simulated sensors.
- Launches the robot into a Gazebo obstacle-course world.
- Bridges Gazebo topics into ROS 2 using `ros_gz_bridge`.
- Opens RViz with a sensor-focused visualization layout.
- Provides simple command-velocity drivers for testing robot movement.
- Includes localization and Nav2 parameter files for future navigation workflows.

## Repository Structure

```text
.
├── src/
│   └── mobile_robot/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   ├── ekf.yaml
│       │   └── nav2_params.yaml
│       ├── launch/
│       │   ├── gazebo_model.launch.py
│       │   ├── rviz_sensors.launch.py
│       │   └── sim.launch.py
│       ├── model/
│       │   ├── robot.gazebo
│       │   └── robot.xacro
│       ├── parameters/
│       │   └── bridge_parameters.yaml
│       ├── rviz/
│       │   └── mobile_robot_sensors.rviz
│       ├── scripts/
│       │   ├── circle_driver.py
│       │   ├── random_walk_driver.py
│       │   └── simple_obstacle_driver.py
│       └── worlds/
│           └── obstacle_course.sdf
├── .gitignore
└── README.md
```

Generated ROS 2 workspace folders such as `build/`, `install/`, and `log/` are intentionally ignored by Git.

## Package Contents

### `model/`

`robot.xacro` defines the robot's physical layout: chassis, wheels, caster, lidar, stereo camera, IMU, magnetometer, and Arducam-style pan/tilt camera. `robot.gazebo` keeps Gazebo-specific simulation systems separate from the base robot description.

### `worlds/`

`obstacle_course.sdf` defines a small Gazebo world with a ground plane, lighting, and static obstacles. It gives the lidar and obstacle-avoidance scripts useful geometry to detect.

### `launch/`

- `sim.launch.py` starts both Gazebo and RViz.
- `gazebo_model.launch.py` starts Gazebo, expands the Xacro model, spawns the robot, starts `robot_state_publisher`, and launches the Gazebo-ROS bridge.
- `rviz_sensors.launch.py` opens RViz with the included sensor visualization config.

### `parameters/`

`bridge_parameters.yaml` maps Gazebo topics to ROS 2 topics. It bridges simulation time, joint states, odometry, TF, velocity commands, IMU, magnetometer, camera streams, lidar scans, and lidar point clouds.

### `config/`

- `ekf.yaml` configures `robot_localization` for 2D state estimation using odometry and IMU data.
- `nav2_params.yaml` contains navigation stack parameters for Nav2-related experimentation.

### `scripts/`

- `circle_driver.py` publishes `/cmd_vel` commands that reverse briefly, move forward briefly, then drive in a circle.
- `simple_obstacle_driver.py` drives forward until the lidar sees an obstacle, turns away, then continues.
- `random_walk_driver.py` is a forward-first wall follower that uses `/scan` to avoid frontal obstacles and track a wall on the left.

## Requirements

This workspace expects a ROS 2 environment with the dependencies listed in `src/mobile_robot/package.xml`, including:

- `ament_cmake`
- `rclpy`
- `robot_state_publisher`
- `robot_localization`
- `ros_gz_sim`
- `ros_gz_bridge`
- `rviz2`
- `xacro`
- `nav2_bringup` / `nav2_common`

Install missing ROS dependencies with your normal ROS 2 package manager or `rosdep`.

## Build

From the repository root:

```bash
colcon build
source install/setup.bash
```

If you are using a fresh shell, source your ROS 2 distribution first, for example:

```bash
source /opt/ros/<ros-distro>/setup.bash
```

## Run the Simulation

Launch Gazebo and RViz together:

```bash
ros2 launch mobile_robot sim.launch.py
```

Launch only Gazebo and the robot model:

```bash
ros2 launch mobile_robot gazebo_model.launch.py
```

Launch only RViz:

```bash
ros2 launch mobile_robot rviz_sensors.launch.py
```

## Run a Driver Node

After the simulation is running, start one of the test drivers in another sourced terminal:

```bash
ros2 run mobile_robot circle_driver.py
```

```bash
ros2 run mobile_robot simple_obstacle_driver.py
```

```bash
ros2 run mobile_robot random_walk_driver.py
```

The drivers publish velocity commands to `/cmd_vel` and, where needed, read lidar data from `/scan`.

## Useful Topics

Common ROS topics bridged or published by this workspace include:

- `/cmd_vel`
- `/odom`
- `/tf`
- `/joint_states`
- `/scan`
- `/scan/points`
- `/imu`
- `/magnetometer`
- `/camera/left/image_raw`
- `/camera/right/image_raw`
- `/arducam/image_raw`

## Notes

The package is currently focused on simulation, sensor visualization, and lightweight motion behaviors. The Nav2 and EKF configuration files are present as groundwork for localization and navigation experiments, but the primary launch path is the Gazebo/RViz simulation stack.
