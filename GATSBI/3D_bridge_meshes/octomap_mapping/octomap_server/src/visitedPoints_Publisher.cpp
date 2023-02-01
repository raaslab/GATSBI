#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>


float rF = 0; //reset flag
geometry_msgs::Pose currentPose;
pcl::PointCloud<pcl::PointXYZ>::Ptr visitedPointsList (new pcl::PointCloud<pcl::PointXYZ>);

void resetFlag_cb(const std_msgs::Float64& msg){ // reset flag
  rF = msg.data;
  ROS_INFO("I heard: [%f]", msg.data);
}

void imu_cb(const geometry_msgs::PoseStamped& msg){ // imu
  pcl::PointXYZ tempPoint(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
  currentPose = msg.pose;
}

int main(int argc, char **argv){
  std::ofstream myfile;
  myfile.open("/home/klyu/bridgeInspection/realDistance.csv");
  ros::init(argc, argv, "visitedPointPublisher");
  ros::NodeHandle n;
  ros::Publisher pointList_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/visited_point_list",1,true);
  ros::Subscriber resetFlag = n.subscribe("/resetFlag", 1, resetFlag_cb);
  ros::Subscriber uavIMU_sub = n.subscribe("/ground_truth_to_tf/pose",1,imu_cb);
  ros::Rate loop_rate(1);
  visitedPointsList->header.frame_id = "/world";
  ROS_INFO("visitedPoints_publisher");
  int count = 0;
  while (ros::ok()){
    ros::spinOnce();
    ROS_INFO("while loop: %d", count);
    myfile<<ros::Time::now()<<","<<currentPose.position.x<<","<<currentPose.position.y<<","<<currentPose.position.z<<std::endl;
    visitedPointsList->width = count+1; visitedPointsList->height = 1; visitedPointsList->points.resize (visitedPointsList->width * visitedPointsList->height);
    if(!rF){ // adding points
      visitedPointsList->points[count].x = currentPose.position.x; visitedPointsList->points[count].y = currentPose.position.y; visitedPointsList->points[count].z = currentPose.position.z;
      if(visitedPointsList->size()){
        pointList_pub.publish(visitedPointsList);
        ROS_INFO("size of visitedPointsList: %lu", visitedPointsList->size());
      }
      count++;
    }
    else{
      visitedPointsList->clear();
      count = 0;
    }
    loop_rate.sleep();
  }
  return 0;
}
