# Navigation-Stack

ROS 1 catkin workspace for an unmanned ground vehicle (UGV) that combines simulation, SLAM, map-based navigation, and custom path-planning nodes.

The repository contains:

- A differential-drive UGV model for Gazebo and RViz.
- A custom global and local planning pipeline implemented in Python.
- `move_base` integrations using both DWA and TEB local planners.
- Hector SLAM and map-server based workflows.
- A hardware control path for ROSserial, RPLidar, and wheel command forwarding.

## What the Project Does

This project is aimed at end-to-end mobile robot navigation for a UGV in both simulation and real-world setups.

Main capabilities:

- Spawn a UGV in a Gazebo lab world.
- Build or load occupancy-grid maps.
- Localize and navigate with Hector SLAM.
- Plan global paths with a custom A* global planner.
- Refine short-horizon motion with a custom local A* planner.
- Run standard ROS navigation with `move_base` using either DWA or TEB.
- Detect nearby obstacles from LiDAR and trigger replanning logic.
- Forward `/cmd_vel` commands to embedded hardware through ROSserial.

## Repository Layout

```text
Navigation-Stack/
├── assembly_urdf/          # Additional robot/assembly URDF assets
├── robot_path_planning/    # Main navigation package
│   ├── config/             # Costmap, controller, EKF, DWA, TEB, AMCL params
│   ├── launch/             # Simulation, SLAM, navigation, and hardware launch files
│   ├── maps/               # Saved occupancy maps
│   ├── rviz/               # RViz presets for each workflow
│   ├── src/                # Custom ROS Python nodes
│   └── world/              # Gazebo world files
├── rplidar_ros/            # RPLidar ROS driver
└── ugv/                    # UGV URDF package
```

## Main Packages

### `robot_path_planning`

The core package in this repository. It includes:

- Custom planners such as `global_planner.py`, `local_a_star.py`, and `local_static.py`.
- Obstacle handling nodes such as `dynamic_obstacle.py`.
- Hardware integration nodes such as `hardware_interface.py`, `rosserial.py`, and `ugv_control.py`.
- Launch files for Gazebo simulation, SLAM, `move_base`, and real robot operation.

### `ugv`

Contains the differential-drive UGV URDF used by Gazebo and `robot_state_publisher`.

### `assembly_urdf`

Contains additional URDF assets for assembly visualization or related mechanical integration.

### `rplidar_ros`

LiDAR driver package used in hardware mapping and navigation workflows.

## Key Workflows

### 1. Custom planner simulation

Launches Gazebo, Hector SLAM, the custom costmap and planning nodes, dynamic obstacle handling, controllers, and RViz.

```bash
roslaunch robot_path_planning custom_sim.launch
```

This workflow starts custom nodes including:

- `costmap.py`
- `global_planner.py`
- `local_a_star.py`
- `local_static.py`
- `dynamic_obstacle.py`

### 2. `move_base` with DWA in simulation

Uses the standard ROS navigation stack with `navfn` and the DWA local planner.

```bash
roslaunch robot_path_planning dwa_sim.launch
```

### 3. `move_base` with TEB in simulation

Uses the standard ROS navigation stack with `navfn` and the TEB local planner.

```bash
roslaunch robot_path_planning teb_sim.launch
```

### 4. Map generation with hardware sensors

Starts ROSserial, RPLidar, Hector SLAM, the UGV control node, and RViz for map creation.

```bash
roslaunch robot_path_planning map_generation.launch
```

### 5. Real robot navigation with Hector SLAM and custom planners

Runs the hardware-oriented navigation path using ROSserial, RPLidar, a saved map, Hector SLAM, custom planners, runtime control, and the hardware interface.

```bash
roslaunch robot_path_planning real_dynamic_hector.launch
```

## Requirements

This repository is structured as a ROS 1 catkin workspace and is best suited to Ubuntu 18.04 based environments.

Recommended base stack:

- Ubuntu 18.04
- ROS 1 catkin environment
- Gazebo
- RViz

Core ROS packages used by the launch files:

- `rospy`
- `roscpp`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf2_ros`
- `robot_state_publisher`
- `joint_state_publisher`
- `controller_manager`
- `gazebo_ros`
- `move_base`
- `navfn`
- `dwa_local_planner`
- `teb_local_planner`
- `costmap_2d`
- `map_server`
- `hector_mapping`
- `hector_trajectory_server`
- `rosserial_python`

Python dependencies used by custom nodes may include:

- `numpy`
- `scipy`
- `websocket_server`

If your environment is missing any of these, install the ROS packages with `apt` and the Python modules with your preferred package manager.

## Build Instructions

From the catkin workspace root:

```bash
cd ~/fyp_ws/Navigation-Stack
catkin_make
source devel/setup.bash
```

If you open a new terminal, source the workspace again before launching anything:

```bash
source ~/fyp_ws/Navigation-Stack/devel/setup.bash
```

## How to Run

### Simulation setup

1. Start ROS if needed.
2. Build the workspace with `catkin_make`.
3. Source `devel/setup.bash`.
4. Launch one of the simulation pipelines.
5. In RViz, use `2D Nav Goal` to send a target pose.

Example:

```bash
roslaunch robot_path_planning custom_sim.launch
```

Alternative planners:

```bash
roslaunch robot_path_planning dwa_sim.launch
roslaunch robot_path_planning teb_sim.launch
```

### Real robot setup

Before launching the hardware stack, review these settings:

- Serial device in `robot_path_planning/launch/rosserial_control.launch`
- Baud rate in `robot_path_planning/launch/rosserial_control.launch`
- Robot LiDAR launch in `rplidar_ros`
- The map path loaded by `robot_path_planning/launch/load_map.launch`

Then run:

```bash
roslaunch robot_path_planning real_dynamic_hector.launch
```

### Mapping workflow

To generate or inspect a map using the hardware-connected robot:

```bash
roslaunch robot_path_planning map_generation.launch
```

## Important Launch Files

| Launch file | Purpose |
| --- | --- |
| `custom_sim.launch` | Gazebo simulation with custom planning stack |
| `dwa_sim.launch` | Gazebo simulation with `move_base` + DWA |
| `teb_sim.launch` | Gazebo simulation with `move_base` + TEB |
| `map_generation.launch` | Hardware-assisted mapping with RPLidar + Hector SLAM |
| `real_dynamic_hector.launch` | Real robot navigation using custom planners |
| `hector_slam.launch` | Hector SLAM node configuration |
| `load_map.launch` | Loads the saved map from `maps/dld_lab.yaml` |
| `rosserial_control.launch` | Serial bridge to embedded controller |

## Custom Nodes Overview

Some of the main Python nodes in `robot_path_planning/src` are:

- `global_planner.py`: grid-based A* global planner that publishes `/global_path`.
- `local_a_star.py`: local planner that builds a short-range LiDAR costmap and replans near the robot.
- `dynamic_obstacle.py`: obstacle avoidance and replanning trigger logic using LiDAR data.
- `hardware_interface.py`: converts `/cmd_vel` into wheel velocity commands for the robot controller.
- `odom_to_base.py`: helper node for frame/odometry integration.
- `ugv_control.py`: control path used during mapping and hardware operation.

## Notes and Assumptions

- The hardware workflows assume a differential-drive UGV.
- The default serial port in the current launch setup is `/dev/ttyUSB1` at `57600` baud.
- The default saved map loaded by the map server is `maps/dld_lab.yaml`.
- Some helper scripts contain network- or hardware-specific values that may need to be adapted for your machine or robot.

## Suggested Demo Commands

```bash
cd ~/fyp_ws/Navigation-Stack
catkin_make
source devel/setup.bash
roslaunch robot_path_planning custom_sim.launch
```

Then in RViz:

1. Wait for the map, robot model, and scans to appear.
2. Select `2D Nav Goal`.
3. Click a target in the map.
4. Watch the global and local planners generate and follow a path.

## Future Improvements

- Add a dependency installation script for a clean machine setup.
- Add screenshots or GIFs of simulation and hardware runs.
- Clean up package metadata such as maintainer information in `package.xml`.
- Add launch argument documentation for selecting maps and planner modes.

## License

The package metadata currently declares `Apache 2.0`. Confirm and update the repository license file if needed.