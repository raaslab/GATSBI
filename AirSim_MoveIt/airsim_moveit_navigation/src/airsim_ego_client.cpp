#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <airsim_moveit_navigation/AirSim_NavigationAction.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#define EPSILON 1e-4

msr::airlib::MultirotorRpcLibClient client;
tf2_ros::Buffer tf_buffer;
double velocity = 1.0;
geometry_msgs::Pose odometry_information;
geometry_msgs::Pose odometry_information_ned;
bool odom_received = false;
double previousX = 0;
double previousY = 0;
double previousZ = 0;

bool pos_com_received = false;
double com_x = -1;
double com_y = -1;
double com_z = -1;

bool double_equals(double a, double b)
{
    return (fabs(a - b)<EPSILON);
}

geometry_msgs::Pose transformPose(geometry_msgs::Pose in, std::string target_frame, std::string source_frame)
{
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::PoseStamped in_stamped;
    geometry_msgs::PoseStamped transformed_stamped;

    in_stamped.pose.position.x = in.position.x;
    in_stamped.pose.position.y = in.position.y;
    in_stamped.pose.position.z = in.position.z;
    in_stamped.pose.orientation.x = in.orientation.x;
    in_stamped.pose.orientation.y = in.orientation.y;
    in_stamped.pose.orientation.z = in.orientation.z;
    in_stamped.pose.orientation.w = in.orientation.w;

    in_stamped.header.seq = 1;
    in_stamped.header.frame_id = source_frame;
    
    geometry_msgs::TransformStamped base_link_to_world_ned;
    base_link_to_world_ned = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(5.0) );
    tf2::doTransform(in_stamped, transformed_stamped, base_link_to_world_ned);
    geometry_msgs::Pose out;
    out = transformed_stamped.pose;

    return out;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    odometry_information_ned.position.x = msg->pose.pose.position.x;
    odometry_information_ned.position.y = msg->pose.pose.position.y;
    odometry_information_ned.position.z = msg->pose.pose.position.z;
    odometry_information_ned.orientation.x = msg->pose.pose.orientation.x;
    odometry_information_ned.orientation.y = msg->pose.pose.orientation.y;
    odometry_information_ned.orientation.z = msg->pose.pose.orientation.z;
    odometry_information_ned.orientation.w = msg->pose.pose.orientation.w;

    odometry_information = transformPose(msg->pose.pose, "world_enu", "world_ned");
    odometry_information.orientation.x = msg->pose.pose.orientation.x;
    odometry_information.orientation.y = msg->pose.pose.orientation.y;
    odometry_information.orientation.z = msg->pose.pose.orientation.z;
    odometry_information.orientation.w = msg->pose.pose.orientation.w;
    odom_received = true;

    if(double_equals(previousX, odometry_information.position.x) &&
       double_equals(previousY, odometry_information.position.y) &&
       double_equals(previousZ, odometry_information.position.z))
        return;

    previousX = odometry_information.position.x;
    previousY = odometry_information.position.y;
    previousZ = odometry_information.position.z;

    /*
    geometry_msgs::PoseStamped pose;
    pose.pose = odometry_information;
    path.header.seq = path.header.seq + 1;
    pose.header.seq = path.header.seq;
    path.header.stamp = ros::Time::now();
    pose.header.stamp = path.header.stamp;
    path.poses.push_back(pose);

    path_pub.publish(path);
    */
}

void planCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  
  ROS_INFO("Got list");

  
  auto path_size = msg->points.size();
  for(int i = 0; i < path_size; i++) {
    ROS_INFO("Moving to Path Point [%d]: [%.2f, %.2f, %.2f]", (i+1), msg->points[i].x, msg->points[i].y, msg->points[i].z);
  
    geometry_msgs::Pose pose_enu;
    pose_enu.position.x = msg->points[i].x;
    pose_enu.position.y = msg->points[i].y;
    pose_enu.position.z = msg->points[i].z + 1;

    double dist = sqrt(pow(pose_enu.position.x - odometry_information.position.x,2) + pow(pose_enu.position.y - odometry_information.position.y,2) + pow(pose_enu.position.z - odometry_information.position.z,2));
    double wait_Time = dist * velocity; 
    wait_Time += 1.5;
    wait_Time = 3;
    
    ROS_INFO("Wait time: %f", wait_Time);
    geometry_msgs::Pose transformed = transformPose(pose_enu, "world_ned", "world_enu");
    client.moveToPositionAsync(transformed.position.x, transformed.position.y, transformed.position.z, 1, 3);
    ros::Duration(wait_Time + 1.1).sleep();
  }
  
}

void posComCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
  //ROS_INFO("Got command");
  geometry_msgs::Pose pose_enu;
  pose_enu.position.x = msg->position.x;
  pose_enu.position.y = msg->position.y;
  pose_enu.position.z = msg->position.z;

  geometry_msgs::Pose transformed = transformPose(pose_enu, "world_ned", "world_enu");

  //if(pos_com_received == false)
  //{
  ROS_INFO("Drone command (enu): [%.2f, %.2f, %.2f]", pose_enu.position.x, pose_enu.position.y, pose_enu.position.z);
  com_x = transformed.position.x;
  com_y = transformed.position.y;
  com_z = transformed.position.z;

  client.moveToPositionAsync(com_x, com_y, com_z, 1, 3);
  pos_com_received = true;
  //}
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "airsim_ego_client");
  ros::NodeHandle n;

  //ros::Subscriber path_sub = n.subscribe("/ego_planner_node/optimal_list", 1000, planCallback);
  ros::Subscriber position_command_sub = n.subscribe("/planning/pos_cmd", 1000, posComCallback);
  ros::Subscriber odom_sub = n.subscribe("/airsim_node/drone_1/odom_local_ned", 1000, odomCallback);

  /*
  ros::Rate rate(10);
  while(ros::ok())
  {
    if(pos_com_received)
    {
      double wait_Time = 3;
      ROS_INFO("Wait time: %f", wait_Time);
      ROS_INFO("Current odom: [%.2f, %.2f, %.2f]", odometry_information_ned.position.x, odometry_information_ned.position.y, odometry_information_ned.position.z);
      ROS_INFO("Drone command: [%.2f, %.2f, %.2f]", com_x, com_y, com_z);
      ROS_INFO("Moving to position: [%.2f, %.2f, %.2f]", odometry_information_ned.position.x + com_x, odometry_information_ned.position.y + com_y, odometry_information_ned.position.z + com_z);
      //client.moveToPositionAsync(odometry_information_ned.position.x + com_x, odometry_information_ned.position.y + com_y, odometry_information_ned.position.z + com_z, 1, 3);
      client.moveToPositionAsync(com_x, com_y, com_z, 1, 3);
      //ros::Duration(wait_Time).sleep();
      pos_com_received = false;
    }
    ros::spinOnce();
    rate.sleep();
  }
  */
  ros::spin();
  return 0;
}