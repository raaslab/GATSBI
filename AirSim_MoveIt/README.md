# AirSim_MoveIt

Required Packages
- AirSim
- MoveIt

Start AirSim in Unreal Engine environment

Run AirSim ROS Wrapper in 1st terminal
```
roslaunch airsim_ros_pkgs airsim_node.launch

Launch MoveIt configuration in 2nd terminal
```
roslaunch airsim_moveit_config gatsbi.launch
```
Run AirSim + MoveIt Client in 3rd terminal
```
rosrun airsim_moveit_navigation airsim_navigator
```

The AirSim + MoveIt client is a ROS action server. An example action client is setup in
```
/airsim_moveit_navigation/src/airsim_navigator_client.cpp
```