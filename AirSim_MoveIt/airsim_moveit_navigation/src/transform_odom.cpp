#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher pub_;
float linear_x;
tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped base_link_to_world_ned;

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
    
    base_link_to_world_ned = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(5.0) );
    tf2::doTransform(in_stamped, transformed_stamped, base_link_to_world_ned);
    geometry_msgs::Pose out;
    out = transformed_stamped.pose;

    return out;
}



void callback(const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_INFO("Got odom");

  geometry_msgs::Pose odom_ned_transformed = transformPose(msg->pose.pose, "world_ned", "drone_1/odom_local_ned");
  geometry_msgs::Pose odom_enu_transformed = transformPose(odom_ned_transformed, "world_enu", "world_ned");
  linear_x = (msg->twist.twist.linear.x );

  nav_msgs::Odometry pose_gt_frame;

  pose_gt_frame.header.frame_id = "world_enu";

  //set the velocity
  pose_gt_frame.pose.pose = odom_enu_transformed;

  //publish the message
  pub_.publish(pose_gt_frame);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "odom_transform");

  ros::NodeHandle n;
  pub_ = n.advertise<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_enu", 5);
  ros::Subscriber sub = n.subscribe("/airsim_node/drone_1/odom_local_ned", 1000, callback);

  ros::spin();
  return 0;
}