# New BCR Robot

## About

This repository contains a Gazebo simulation for a differential drive robot, equipped with an IMU, a depth camera and a 2D LiDAR. The primary contriution of this project is to support multiple ROS and Gazebo distros. Currently, the project supports the following versions - 

Each of the following sections describes depedencies, build and run instructions for each of the above combinations

## Noetic + Classic (Ubuntu 20.04)

### Dependencies

In addition to ROS1 Noetic and Gazebo Classic installations, the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
catkin_make
```

### Run
To Run all launch files below,
```bash
roslaunch bcr_bot multi_bot_core.launch
```
To launch the multi robots in Gazebo,
```bash
roslaunch bcr_bot multi_gazebo.launch
```
To navigate the multi robots,
```bash
roslaunch bcr_bot multi_navigation.launch
```
To view in rviz,
```bash
roslaunch bcr_bot multi_rviz.launch
```

### Simulation and Visualization
1. Gz Sim (Ignition Gazebo) (small_warehouse World):
	![](res/gz.jpg)

2. Rviz (Depth camera) (small_warehouse World):
	![](res/rviz.jpg)
