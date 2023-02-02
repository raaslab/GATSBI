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

Run AirSim ROS Wrapper in 1st terminal
```
roslaunch airsim_ros_pkgs airsim_node.launch

Launch OctoMap GATSBI launch file in 2nd terminal. This runs the following:
- Envrionment Octomap
- Bridge Segmented Octomap
- Ground Filter
- Visited Points Tracker
```
roslaunch octomap_server octo_gatsbi.launch

Run GTSP solver in 3rd terminal
```
rosrun gtsp gtsp_solver

Launch MoveIt configuration in 4th terminal
```
roslaunch airsim_moveit_config gatsbi.launch

Run AirSim + MoveIt Client in 5th terminal
```
rosrun airsim_moveit_navigation airsim_navigator

Run GATSBI
```
rosrun octomap_server airsim_MODGATSBI