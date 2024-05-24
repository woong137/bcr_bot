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
catkin build --packages-select bcr_bot
```

### Run

To launch the multi robots in Gazebo,
```bash
roslaunch bcr_bot multi_gazebo.launch
```
To navigate the multi robots
```bash
roslaunch bcr_bot multi_navigation.launch
```
To view in rviz, (The multi-robot rviz has not been implemented yet)
```bash
roslaunch bcr_bot rviz.launch
```

### Configuration

The launch file accepts multiple launch arguments,
```bash
roslaunch bcr_bot gazebo.launch \
	camera_enabled:=True \
	two_d_lidar_enabled:=True \
	stereo_camera_enabled:=True \
	position_x:=0.0 \
	position_y:=0.0 \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.world \
	robot_namespace:="bcr_bot"
```
**Note:** To use stereo_image_proc with the stereo images excute following command: 
```bash
ROS_NAMESPACE=bcr_bot/stereo_camera rosrun stereo_image_proc stereo_image_proc
```

**Note:** To use stereo_image_proc with the stereo images excute following command: 
```bash
ros2 launch stereo_image_proc stereo_image_proc.launch.py left_namespace:=bcr_bot/stereo_camera/left right_namespace:=bcr_bot/stereo_camera/right
```
### Simulation and Visualization
1. Gz Sim (Ignition Gazebo) (small_warehouse World):
	![](res/gz.jpg)

2. Rviz (Depth camera) (small_warehouse World):
	![](res/rviz.jpg)
