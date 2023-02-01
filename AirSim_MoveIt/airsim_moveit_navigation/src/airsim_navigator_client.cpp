#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <airsim_moveit_navigation/AirSim_NavigationAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nagvigation_client");

  actionlib::SimpleActionClient<airsim_moveit_navigation::AirSim_NavigationAction> ac("airsim_navigator", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");

  airsim_moveit_navigation::AirSim_NavigationGoal goal;

  goal.goal_pose.position.x = 1;
  goal.goal_pose.position.y = 1;
  goal.goal_pose.position.z = 4;

  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = 0;
  goal.goal_pose.orientation.w = 1;

  ROS_INFO("Send action goal");
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult();

  if(finished_before_timeout)
  {
  	actionlib::SimpleClientGoalState state = ac.getState();
  	ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  	ROS_INFO("Action did not finish before the time out.");

  goal.goal_pose.position.x = -2;
  goal.goal_pose.position.y = -2;
  goal.goal_pose.position.z = 6;

  ac.sendGoal(goal);

  finished_before_timeout = ac.waitForResult();

  if(finished_before_timeout)
  {
  	actionlib::SimpleClientGoalState state = ac.getState();
  	ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  	ROS_INFO("Action did not finish before the time out.");

  return 0;
}