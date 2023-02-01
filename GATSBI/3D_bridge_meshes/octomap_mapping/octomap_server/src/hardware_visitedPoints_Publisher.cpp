#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

float rF = 0; //reset flag
geometry_msgs::Pose currentPose;
pcl::PointCloud<pcl::PointXYZ>::Ptr visitedPointsList (new pcl::PointCloud<pcl::PointXYZ>);

void resetFlag_cb(const std_msgs::Float64& msg){ // reset flag
  rF = msg.data;
  ROS_INFO("I heard: [%f]", msg.data);
}

void callback(const geometry_msgs::PointStampedConstPtr& position_data, const geometry_msgs::QuaternionStampedConstPtr& attitude_data) {

  std::cout << "Position Data: " << position_data->point.x << " " << position_data->point.y << " " << position_data->point.z << std::endl;
  std::cout << "Attitude Data: " << attitude_data->quaternion.x << " " << attitude_data->quaternion.y << " " << attitude_data->quaternion.z << " " << attitude_data->quaternion.w << std::endl;
  currentPose.position = position_data->point;
  currentPose.orientation = attitude_data->quaternion;
}

int main(int argc, char **argv){
  std::ofstream myfile;
  myfile.open("/home/user/bridgeInspection/realDistance.csv");
  ros::init(argc, argv, "visitedPointPublisher");
  ros::NodeHandle n;
  ros::Publisher pointList_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/visited_point_list",1,true);
  ros::Subscriber resetFlag = n.subscribe("/resetFlag", 1, resetFlag_cb);
  //ros::Subscriber uavIMU_sub = n.subscribe("/airsim_node/drone_1/odom_local_ned",1,imu_cb);
  message_filters::Subscriber<geometry_msgs::PointStamped> position_sub(n, "/dji_sdk/local_position", 1);
  message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub(n, "/dji_sdk/attitude", 1);

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::QuaternionStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), position_sub, attitude_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::Rate loop_rate(1);
  visitedPointsList->header.frame_id = "/world_enu";
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
