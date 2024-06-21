#include "ros/ros.h"
#include "std_msgs/String.h"
#include <moveit_msgs/DisplayTrajectory.h>

void chatterCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
  ROS_INFO("Got plan");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/move_group/display_planned_path", 1000, chatterCallback);

  ros::spin();

  return 0;
}