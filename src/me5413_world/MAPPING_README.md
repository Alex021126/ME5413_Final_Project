# Mapping README

## Install Packages

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Start Mapping

```bash
cd ~/ME5413_Final_Project
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch me5413_world full_mapping.launch
```

## Save the Map

```bash
cd ~/ME5413_Final_Project/src/me5413_world/maps
rosrun map_server map_saver -f my_map map:=/map
```
