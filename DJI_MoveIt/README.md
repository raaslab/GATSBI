# DJI_MoveIt

Required Packages
- DJI ROS SDK
- [Velodyne Pointcloud](http://wiki.ros.org/velodyne_pointcloud)
- MoveIt

Run DJI ROS SDK in 1st terminal
```
roslaunch dji_sdk sdk.launch

Launch Velodyne LiDAR ROS package in 2nd terminal
```
roslaunch velodyne_pointcloud VLP16_points.launch
```
Launch tf transformations for DJI and LiDAR in 3rd terminal
```
roslaunch point_cloud_processing hardware_transform.launch
```
Launch DJI MoveIt configuration
```
roslaunch airsim_moveit_config gatsbi.launch
```
Run DJI + MoveIt Client in 3rd terminal
```
rosrun airsim_moveit_navigation airsim_navigator
```
The DJI + MoveIt client is a ROS action server. An example action client is setup in
```
/airsim_moveit_navigation/src/airsim_navigator_client.cpp
```