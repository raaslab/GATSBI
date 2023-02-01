#define _USE_MATH_DEFINES
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <pcl/octree/octree_search.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <pcl/io/pcd_io.h>
#include "../include/octomap_server/kevin_functions.h"
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hector_moveit_navigation/NavigationAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <gtsp/Tour.h>
#include <gtsp/GTSPData.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <string>
#include <list>
#include <set>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#define MAX_TOUR_SIZE 25
// global variables
int resolution = 1;
pcl::PointCloud<pcl::PointXYZ>::Ptr runningVisitedVoxels (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<double> tempzFilteredSize;
octomap::OcTree* fullOcTree = new octomap::OcTree(resolution);
octomap::OcTree* trimmedOcTree = new octomap::OcTree(resolution);
pcl::PointCloud<pcl::PointXYZ>::Ptr visitedPointsList (new pcl::PointCloud<pcl::PointXYZ>);
geometry_msgs::Pose currentPose;



void imu_cb(const geometry_msgs::PoseStamped& msg){ // imu
  pcl::PointXYZ tempPoint(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
  currentPose = msg.pose;
}

void zFiltered_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
  tempCloudOccTrimmed->clear();
  BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points){
    tempCloudOccTrimmed->points.push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
  }
}

void zFilteredSize_cb(const std_msgs::Float64MultiArray& msg){
  tempzFilteredSize.clear();
  for(int i = 0;i<msg.data.size();i++){
    tempzFilteredSize.push_back(msg.data.at(i));
  }
}

void full_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
  fullOcTree = dynamic_cast<octomap::OcTree*>(absTree);
  // countCB++;
}

void trimmed_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
  trimmedOcTree = dynamic_cast<octomap::OcTree*>(absTree);
  // countCB++;
}

void visitedPointList_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
  visitedPointsList->clear();
  BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points){
    visitedPointsList->points.push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
  }
}

int main(int argc, char** argv){
  std::ofstream myfileT;
  myfileT.open("/home/klyu/bridgeInspection/timeVSTrimmedOcc_BL.csv");
  std::ofstream myfileBLD;
  myfileBLD.open("/home/klyu/bridgeInspection/timeVSDistance_BL.csv");
  // initializing ROS everything
  ros::init(argc, argv, "baseline");
  ros::NodeHandle n;
  ros::Rate r(0.5); // less than 1 is slower
  ros::Time beginT = ros::Time::now();
  ros::Time updateT;
  ros::Subscriber uavIMU_sub = n.subscribe("/ground_truth_to_tf/pose",1,imu_cb);
  ros::Subscriber zFiltered_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/zFiltered",1,zFiltered_cb);
  ros::Subscriber zFilteredSize_sub = n.subscribe("/zFilteredSize", 1, zFilteredSize_cb);
  ros::Subscriber fullTree_sub = n.subscribe("/octomap_full",1,full_cb);
  ros::Subscriber trimmedTree_sub = n.subscribe("/octomap_full_trimmed",1,trimmed_cb);
  ros::Subscriber visitedPointList_sub = n.subscribe("/visited_point_list", 1, visitedPointList_cb);

  float thresholdOcc = 1.0;
  float thresholdFree = 0.0;
  float sizeOfUAV = 1.0;
  float minRadius = 2;
  float maxRadius = 5;
  octomap::OcTree::leaf_iterator it;
  octomap::OcTree::leaf_iterator endLeaf;
  int id4Markers;
  int loopNumber = 0;
  float xMin = -17;
  float xMax = 17;
  float yMin = -3;
  float yMax = 13;
  float zMin = 0.2;
  float zMax = 10;

  while (ros::ok()){
    ROS_INFO("Loop: %d", loopNumber);
    ros::spinOnce();
    // initializing variables
    id4Markers = 0; int countFreeFull = 0; int countOccFull = 0; int countUnknownFull = 0; int allCountOccInBounds = 0;
    // getting sizes of free, occupied, and unknown for full octree
    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it){
      if(it->getValue()>thresholdOcc){
        if(xMin<it.getX()<xMax && yMin<it.getY()<yMax && zMin<it.getZ()<zMax){
          allCountOccInBounds++;
        }
        countOccFull = countOccFull + 1;
      } else if(it->getValue()<thresholdFree){
        countFreeFull = countFreeFull + 1;
      } else{
        countUnknownFull = countUnknownFull + 1;
      }
      id4Markers = id4Markers + 1;
    }
    // getting sizes of free, occupied, and unknown for trimmed octree
    int countFreeTrimmed = 0; int countOccTrimmed = 0; int countUnknownTrimmed = 0;
    for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
      if(it->getValue()>thresholdOcc){
        countOccTrimmed = countOccTrimmed + 1;
      } else if(it->getValue()<thresholdFree){
        countFreeTrimmed = countFreeTrimmed + 1;
      } else{
        countUnknownTrimmed = countUnknownTrimmed + 1;
      }
      id4Markers = id4Markers + 1;
    }

    // creating point clouds for free, occupied, and unknown voxels for full point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFreeFull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccFull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudUnknownFull (new pcl::PointCloud<pcl::PointXYZ>);
    cloudFreeFull->width = countFreeFull; cloudFreeFull->height = 1; cloudFreeFull->points.resize (cloudFreeFull->width * cloudFreeFull->height);
    cloudOccFull->width = countOccFull; cloudOccFull->height = 1; cloudOccFull->points.resize (cloudOccFull->width * cloudOccFull->height);
    cloudUnknownFull->width = countUnknownFull; cloudUnknownFull->height = 1; cloudUnknownFull->points.resize (cloudUnknownFull->width * cloudUnknownFull->height);
    std::vector<double> freeSizeFull;
    std::vector<double> occSizeFull;
    countFreeFull = 0; countOccFull = 0; countUnknownFull = 0;
    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it){
      if(it->getValue()>thresholdOcc){
        cloudOccFull->points[countOccFull].x = it.getX(); cloudOccFull->points[countOccFull].y = it.getY(); cloudOccFull->points[countOccFull].z = it.getZ();
        occSizeFull.push_back(it.getSize());
        countOccFull = countOccFull + 1;
      } else if(it->getValue()<thresholdFree){
        cloudFreeFull->points[countFreeFull].x = it.getX(); cloudFreeFull->points[countFreeFull].y = it.getY(); cloudFreeFull->points[countFreeFull].z = it.getZ();
        freeSizeFull.push_back(it.getSize());
        countFreeFull = countFreeFull + 1;
      } else{
        cloudUnknownFull->points[countUnknownFull].x = it.getX(); cloudUnknownFull->points[countUnknownFull].y = it.getY(); cloudUnknownFull->points[countUnknownFull].z = it.getZ();
        countUnknownFull = countUnknownFull + 1;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<double> zFilteredSize;
    int removeVoxel;
    for(int j=0;j<tempCloudOccTrimmed->size();j++){
      removeVoxel = 0;
      for(int i=0;i<runningVisitedVoxels->size();i++){
        if(tempCloudOccTrimmed->at(j).x==runningVisitedVoxels->at(i).x && tempCloudOccTrimmed->at(j).y==runningVisitedVoxels->at(i).y && tempCloudOccTrimmed->at(j).z==runningVisitedVoxels->at(i).z){
          removeVoxel = 1;
        }
      }
      if(removeVoxel == 0){
        cloudOccTrimmed->push_back(tempCloudOccTrimmed->at(j));
        zFilteredSize.push_back(tempzFilteredSize.at(j));
      }
    }
    updateT = ros::Time::now();
    myfileT << updateT-beginT << "," << runningVisitedVoxels->size() << "," << tempCloudOccTrimmed->size()-runningVisitedVoxels->size()<<","<< countOccFull-tempCloudOccTrimmed->size() << ","<< countFreeFull << std::endl;

    float tempX1; float tempY1; float tempZ1; float tempX2; float tempY2; float tempZ2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr viewPoints (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> point2ClusterMapping;
    std::vector<double> tempRes;
    std::vector<float> tempOrientation;
    std::vector<double> tempRes1;
    std::vector<float> tempOrientation1;
    std::vector<float> orientationOfCPs;
    int numOfCluster = 0;
    for(int j = 0; j<cloudOccTrimmed->size();j++){
      tempX1 = cloudOccTrimmed->points[j].x;
      tempY1 = cloudOccTrimmed->points[j].y;
      tempZ1 = cloudOccTrimmed->points[j].z;
      tempPoints->clear();
      tempPoints1->clear();
      findIfVoxelCanBeSeen(tempX1,tempY1,tempZ1,zFilteredSize.at(j),cloudFreeFull,freeSizeFull,minRadius,maxRadius,sizeOfUAV,tempPoints1,&tempRes1,&tempOrientation1);
      removeVoxelsTooClose(tempPoints1,cloudOccFull,sizeOfUAV+(zFilteredSize.at(j)/2),tempPoints,&tempRes1,&tempRes,&tempOrientation1,&tempOrientation);
      if(tempPoints->size()>0){
        viewPoints->push_back(cloudOccTrimmed->points[j]);
        numOfCluster++;
      }
      for(int i = 0; i<tempPoints->size();i++){
        clusteredPoints->push_back(tempPoints->points[i]);
        point2ClusterMapping.push_back(numOfCluster);
        orientationOfCPs.push_back(tempOrientation[i]);
      }
    }
    float realDistance = 0;
    bool found;
    for(int i=0;i<visitedPointsList->size();i++){
      for(int j=0;j<clusteredPoints->size();j++){
        if(checkIfPointIsInVoxel(visitedPointsList->at(i), clusteredPoints->at(j), tempRes.at(j))){
          pcl::PointXYZ tempPoint = viewPoints->at(point2ClusterMapping.at(j)-1);
          found = false;
          for(int k=0;k<runningVisitedVoxels->size();k++){
            if(runningVisitedVoxels->at(k).x==tempPoint.x && runningVisitedVoxels->at(k).y==tempPoint.y && runningVisitedVoxels->at(k).z==tempPoint.z){
              found = true;
              break;
            }
          }
          if(!found){
            runningVisitedVoxels->push_back(tempPoint);
          }
          break;
        }
      }
    }

    if(visitedPointsList->size()>0){
      for(int i=0;i<visitedPointsList->size()-1;i++){
        realDistance += sqrt(pow(visitedPointsList->at(i+1).x-visitedPointsList->at(i).x,2)+pow(visitedPointsList->at(i+1).y-visitedPointsList->at(i).y,2)+pow(visitedPointsList->at(i+1).z-visitedPointsList->at(i).z,2));
      }
    }
    updateT = ros::Time::now();
    // myfileT << updateT-beginT << "," << runningVisitedVoxels->size() << "," << allCountOccInBounds << std::endl;
    myfileBLD << updateT-beginT<<","<<realDistance<<std::endl;
    loopNumber++;
    r.sleep();
  }
  myfileT.close();

}
