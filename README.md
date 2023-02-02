# GATSBI

GATSBI folder contains GATSBI Algorithm
AirSim_MoveIt folder contains the AirSim and MoveIt integrated ROS package
DJI_MoveIt folder contains the DJI SDK and MoveIt integrated ROS package


## Running GATSBI in Simulation
Required Simulation Packages
- AirSim
- MoveIt
- [GTSP Solver](https://github.com/hsd1121/gtsp)

Start AirSim in Unreal Engine environment

Run AirSim ROS Wrapper in first terminal
```
roslaunch airsim_ros_pkgs airsim_node.launch