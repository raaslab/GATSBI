#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ego_goal");

  ros::NodeHandle n;


  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);


  geometry_msgs::PoseStamped goal;

  goal.pose.position.x = 5;
  goal.pose.position.y = 0;
  goal.pose.position.z = 3;

  goal_pub.publish(goal);

  ros::spinOnce();


  return 0;
}