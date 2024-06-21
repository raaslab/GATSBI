#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

bool rF = false; //reset flag
geometry_msgs::Pose currentPose;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr visitedPointsList (new pcl::PointCloud<pcl::PointXYZRGBA>);
tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped base_link_to_world_ned;
geometry_msgs::PoseArray pose_array_;

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

void resetFlag_cb(const std_msgs::Bool& msg){ // reset flag
  rF = msg.data;
  ROS_INFO("I heard: [%d]", (msg.data) ? 1 : 0);
}

void imu_cb(const nav_msgs::Odometry::ConstPtr msg){ // imu
  geometry_msgs::Pose odometry_information = transformPose(msg->pose.pose, "world_enu", "world_ned");
  pcl::PointXYZ tempPoint(odometry_information.position.x,odometry_information.position.y,odometry_information.position.z);
  currentPose = odometry_information;
}

int main(int argc, char **argv){
  std::ofstream myfile;
  myfile.open("/home/user/bridgeInspection/realDistance.csv");
  ros::init(argc, argv, "visitedPointPublisher");
  ros::NodeHandle n;
  ros::Publisher pointList_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGBA>> ("/visited_point_list",1,true);
  ros::Publisher poseList_pub = n.advertise<geometry_msgs::PoseArray> ("/visited_poses_list",1,true);
  ros::Subscriber resetFlag = n.subscribe("/resetFlag", 1, resetFlag_cb);
  ros::Subscriber uavIMU_sub = n.subscribe("/airsim_node/drone_1/odom_local_ned",1,imu_cb);
  ros::Rate loop_rate(1);
  visitedPointsList->header.frame_id = "/world_enu";
  pose_array_.header.frame_id = "/world_enu";

  ROS_INFO("visitedPoints_publisher");
  int count = 0;

  while (ros::ok()){
    ros::spinOnce();
    //ROS_INFO("while loop: %d", count);
    myfile<<ros::Time::now()<<","<<currentPose.position.x<<","<<currentPose.position.y<<","<<currentPose.position.z<<std::endl;
    visitedPointsList->width = count+1; visitedPointsList->height = 1; visitedPointsList->points.resize (visitedPointsList->width * visitedPointsList->height);
    if(!rF){ // adding points
      visitedPointsList->points[count].x = currentPose.position.x; visitedPointsList->points[count].y = currentPose.position.y; visitedPointsList->points[count].z = currentPose.position.z;
      visitedPointsList->points[count].r = currentPose.orientation.x;
      visitedPointsList->points[count].g = currentPose.orientation.y; 
      visitedPointsList->points[count].b = currentPose.orientation.z; 
      visitedPointsList->points[count].a = currentPose.orientation.w;  
      pose_array_.poses.push_back(currentPose);
      if(visitedPointsList->size()){
        pointList_pub.publish(visitedPointsList);
        poseList_pub.publish(pose_array_);
        //ROS_INFO("size of visitedPointsList: %lu", visitedPointsList->size());
      }
      count++;
    }
    else{
      ROS_INFO("Clearing Visited Points and Pose List");
      visitedPointsList->clear();
      pose_array_.poses.clear();
      ROS_INFO("Pose List size: [%d]", pose_array_.poses.size());
      count = 0;
    }
    loop_rate.sleep();
  }
  return 0;
}
