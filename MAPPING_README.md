# Mapping README

This note focuses only on the mapping workflow in this project.

## Goal

This project uses a 2D occupancy-grid map built with a tuned 2D LiDAR GMapping pipeline.
Given the current workspace dependencies and the sensors already wired into the robot model, this is the best mapping algorithm that can be run immediately without adding new ROS packages or violating the README constraints.

The recommended stack is:

- front 2D LiDAR on `/front/scan`
- wheel odometry from the Jackal base
- the simulated IMU for state estimation support and operator monitoring
- `slam_gmapping` for occupancy-grid SLAM

This project keeps a 2D mapping pipeline because the downstream navigation stack also consumes a 2D occupancy grid.

The mapping pipeline is:

- `world.launch` starts Gazebo and spawns the Jackal with the `front_laser` configuration
- `mapping.launch` starts keyboard teleop, tuned `gmapping`, and RViz
- `full_mapping.launch` combines the two into one command

The resulting map is saved as:

- `my_map.pgm`
- `my_map.yaml`

under `src/me5413_world/maps/`.

## Required Packages

Before running mapping, make sure the workspace can build and the required ROS packages are installed.

Recommended commands:

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Core packages used by the mapping workflow:

- `me5413_world`
- `jackal_description`
- `interactive_tools`
- `jackal_navigation`
- `jackal_gazebo`
- `jackal_control`
- `gazebo_ros`
- `gazebo_plugins`
- `map_server`
- `teleop_twist_keyboard`
- `tf2_eigen`
- `pointgrey_camera_description`
- `flir_camera_description`
- `rviz_imu_plugin`
- `sick_tim`
- `lms1xx`
- `velodyne_description`

Packages that were needed during setup on this machine:

```bash
sudo apt update
sudo apt install -y \
  qtbase5-dev \
  libgazebo11-dev \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-tf2-eigen \
  ros-noetic-jackal-navigation \
  ros-noetic-jackal-gazebo \
  ros-noetic-jackal-control \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-pointgrey-camera-description \
  ros-noetic-flir-camera-description \
  ros-noetic-rviz-imu-plugin \
  ros-noetic-sick-tim \
  ros-noetic-lms1xx \
  ros-noetic-velodyne-description
```

Useful checks:

```bash
rospack find me5413_world
rospack find jackal_navigation
rospack find teleop_twist_keyboard
rospack find gazebo_ros
```

Then build:

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Gazebo Models

The world needs models inside `~/.gazebo/models`.

```bash
cd ~
mkdir -p ~/.gazebo/models
git clone https://github.com/osrf/gazebo_models.git
cp -r ~/gazebo_models/* ~/.gazebo/models
cp -r ~/ME5413_Final_Project/src/me5413_world/models/* ~/.gazebo/models
```

If `~/gazebo_models` already exists, you only need the `cp` commands.

## Algorithm Choice

The selected algorithm for this repository is tuned `slam_gmapping`.

Why this is the recommended choice here:

- It is already available in the current ROS Noetic setup.
- It works directly with the robot's existing `/front/scan` topic and wheel odometry.
- It produces the 2D occupancy grid required by the provided navigation stack.
- It stays fully within the project README constraints because it uses realistic onboard sensors and does not depend on Gazebo ground truth.

Why we are not recommending another algorithm in this repository right now:

- `cartographer_ros` would likely outperform GMapping on loop closure and drift, but it is not installed in the current workspace.
- `rtabmap_ros` is also not installed and would require a larger integration effort around camera or LiDAR fusion.
- `hector_mapping` is not installed, and in this project it would be less attractive than GMapping because the workflow already has usable wheel odometry.

So, for this codebase as it exists today, tuned GMapping is the best practical choice.

## Recommended Mapping Workflow

The most stable workflow used here is to start the world and mapping separately.

Terminal 1:

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch me5413_world world.launch
```

Terminal 2:

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch me5413_world mapping.launch
```

This follows the original README workflow, but with the robot spawned in the `front_laser` configuration and a tuned GMapping parameter set.

## One-Command Option

The project also provides:

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch me5413_world full_mapping.launch
```

This launch file starts both the world and mapping stack together, and disables the large static obstacle by default through `spawn_static_box:=false`.

If the separate workflow is already working for you, use that one first.

## Driving During Mapping

`mapping.launch` starts `teleop_twist_keyboard`, so you must control the robot from the terminal where `mapping.launch` is running.

Common keys:

```text
u i o
j k l
m , .
```

- `i`: forward
- `,`: backward
- `j`: turn left
- `l`: turn right
- `k`: stop

Use an English keyboard layout when sending commands.

## What Normal Mapping Looks Like

When tuned `gmapping` is working, the terminal prints messages like:

```text
update frame ...
Average Scan Matching Score=...
Registering Scans:Done
```

In RViz, you should see:

- `/map` gradually expanding
- laser scan points in front of the robot
- the robot pose updating while you drive

For best map quality:

- drive slowly, especially before and after turns
- avoid sudden in-place spins unless needed for scan coverage
- prefer large loops that revisit previously seen corridors and rooms
- keep moving obstacles disabled during map collection when possible
- save the map only after you have closed at least one long loop in the environment

## Saving the Map

After the map is complete, open a third terminal:

```bash
cd ~/ME5413_Final_Project/src/me5413_world/maps
rosrun map_server map_saver -f my_map map:=/map
```

Generated files:

- `src/me5413_world/maps/my_map.pgm`
- `src/me5413_world/maps/my_map.yaml`

## Viewing the Saved Map

Quick check:

```bash
ls ~/ME5413_Final_Project/src/me5413_world/maps
```

Open the image directly:

```bash
xdg-open ~/ME5413_Final_Project/src/me5413_world/maps/my_map.pgm
```

Or use it later in navigation:

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch me5413_world navigation.launch
```

## Notes

- For this project, the naive pipeline is based on a 2D map, not a 3D map.
- The robot is expected to go up the ramp and reach the upper level.
- Mapping can still work even if RViz reports the IMU display plugin missing; that only affects visualization.
- The RViz control panel mentioned in the main README is an optional helper for respawning or clearing random objects. Mapping itself does not depend on it.
- The repository now defaults to the `front_laser` Jackal configuration during mapping so the SLAM stack always has an explicit 2D LiDAR source.
- The tuned GMapping launch file lives at `src/me5413_world/launch/include/gmapping.launch` if you want to keep iterating on the parameters.
