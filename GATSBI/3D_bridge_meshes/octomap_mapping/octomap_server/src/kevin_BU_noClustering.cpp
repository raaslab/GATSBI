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
#include "/home/user/catkin_ws/src/octomap_mapping/octomap_server/include/octomap_server/kevin_functions.h"
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hector_moveit_navigation/NavigationAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <gtsp/Tour.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <string>

#define MAX_TOUR_SIZE 25


// global variables
int resolution = 1;
octomap::OcTree* fullOcTree = new octomap::OcTree(resolution);
octomap::OcTree* trimmedOcTree = new octomap::OcTree(resolution);
int countCB = 0;
int fileSave = 0;
int markerSize = 1;
float xData=0.0;
float yData=0.0;
float zData=0.0;
pcl::PointCloud<pcl::PointXYZ>::Ptr runningVisitedPC (new pcl::PointCloud<pcl::PointXYZ>);
int positionCount = 0;
std::vector<int> tour;
double moveit_distance = -1;
bool tour_ready = false;
bool length_ready = false;
geometry_msgs::Pose currentPose;


void tourCallback(const gtsp::Tour::ConstPtr& msg)
{
  ROS_INFO("Got tour");
  tour = msg->tour;
  std::string output = "TOUR: ";
  for(int i = 0; i < tour.size(); i++)
  {
    output.append(std::to_string(tour[i]));
    output.append(" ");
  }
  ROS_INFO("%s", output.c_str());
  tour_ready = true;
}

void lengthCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Got length");
  moveit_distance = msg->data;
  ROS_INFO("MoveIt Distance: %f", moveit_distance);
  length_ready = true;
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

void imu_cb(const geometry_msgs::PoseStamped &msg){ // imu call back
  pcl::PointXYZ tempPoint(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
  currentPose = msg.pose;
  runningVisitedPC->push_back(tempPoint);
}



int main(int argc, char** argv){
// initializing ROS everything
  ros::init(argc, argv, "kevin");
  actionlib::SimpleActionClient<hector_moveit_navigation::NavigationAction> ac("hector_navigator", true);
  ros::NodeHandle n;
  ros::Rate r(0.2); // less than 1 is slower
  ros::Publisher occArrayTrimmed_pub = n.advertise<visualization_msgs::MarkerArray>("/occArrayTrimmed_pub",1,true);
  ros::Publisher freeArrayTrimmed_pub = n.advertise<visualization_msgs::MarkerArray>("/freeArrayTrimmed_pub",1,true);
  ros::Publisher occArrayFull_pub = n.advertise<visualization_msgs::MarkerArray>("/occArrayFull_pub",1,true);
  ros::Publisher freeArrayFull_pub = n.advertise<visualization_msgs::MarkerArray>("/freeArrayFull_pub",1,true);
  ros::Publisher impossibleArray_pub = n.advertise<visualization_msgs::MarkerArray>("/impossibleArray_pub",1,true);
  ros::Publisher usableArray_pub = n.advertise<visualization_msgs::MarkerArray>("/usableArray_pub",1,true);
  ros::Publisher point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/gtsp_point_cloud", 1);
  ros::Publisher goal_distance_publisher = n.advertise<geometry_msgs::Point>("/compute_path/point", 1);

  ros::Subscriber fullTree_sub = n.subscribe("/octomap_full",1,full_cb);
  ros::Subscriber trimmedTree_sub = n.subscribe("/octomap_full_trimmed",1,trimmed_cb);
  ros::Subscriber uavIMU_pub = n.subscribe("/ground_truth_to_tf/pose",1,imu_cb);
  ros::Subscriber tour_sub = n.subscribe("/gtsp_tour_list", 1, tourCallback);
  ros::Subscriber distance_sub = n.subscribe("/compute_path/length", 1, lengthCallback);

// initializing arrays
  pcl::PointCloud<pcl::PointXYZ>::Ptr runningImpossiblePC (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr runningVisitedPC (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ searchPoint;
  visualization_msgs::MarkerArray occArrayTrimmed;
  visualization_msgs::MarkerArray freeArrayTrimmed;
  visualization_msgs::MarkerArray occArrayFull;
  visualization_msgs::MarkerArray freeArrayFull;
  visualization_msgs::MarkerArray impossibleArray;
  visualization_msgs::MarkerArray usableArray;

// initializing variables and markers
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int loopNumber = 0;
  float minRadius = 2.0;
  float maxRadius = 5.0;
  float thresholdOcc = 1.0;
  float thresholdFree = 0.0;
  int sizeOfUAV = 1;
  float tempID; int id4Markers; float markerSize;
  bool tspDone = false;
  pcl::PointXYZ searchPoint1;
  pcl::PointXYZ searchPoint2;
  octomap::OcTree::leaf_iterator it;
  octomap::OcTree::leaf_iterator endLeaf;

  visualization_msgs::Marker fullOccupiedMarkers; fullOccupiedMarkers.header.frame_id = "/world"; fullOccupiedMarkers.header.stamp = ros::Time::now(); fullOccupiedMarkers.ns = "occupied_markers_full";
  fullOccupiedMarkers.type = shape; fullOccupiedMarkers.action = visualization_msgs::Marker::ADD;
  fullOccupiedMarkers.color.r = 1.0f; fullOccupiedMarkers.color.g = 0.0f; fullOccupiedMarkers.color.b = 0.0f; fullOccupiedMarkers.color.a = 1.0;

  visualization_msgs::Marker fullFreeMarkers; fullFreeMarkers.header.frame_id = "/world"; fullFreeMarkers.header.stamp = ros::Time::now(); fullFreeMarkers.ns = "free_markers_full";
  fullFreeMarkers.type = shape; fullFreeMarkers.action = visualization_msgs::Marker::ADD;
  fullFreeMarkers.color.r = 0.0f; fullFreeMarkers.color.g = 1.0f; fullFreeMarkers.color.b = 0.0f; fullFreeMarkers.color.a = 1.0;

  visualization_msgs::Marker fullUnknownMarkers; fullUnknownMarkers.header.frame_id = "/world"; fullUnknownMarkers.header.stamp = ros::Time::now(); fullUnknownMarkers.ns = "unknown_markers_full";
  fullUnknownMarkers.type = shape; fullUnknownMarkers.action = visualization_msgs::Marker::ADD;
  fullUnknownMarkers.color.r = 1.0f; fullUnknownMarkers.color.g = 1.0f; fullUnknownMarkers.color.b = 0.0f; fullUnknownMarkers.color.a = 1.0;

  visualization_msgs::Marker impossibleMarkers; impossibleMarkers.header.frame_id = "/world"; impossibleMarkers.header.stamp = ros::Time::now(); impossibleMarkers.ns = "impossible_markers";
  impossibleMarkers.type = shape; impossibleMarkers.action = visualization_msgs::Marker::ADD;
  impossibleMarkers.color.r = 0.4f; impossibleMarkers.color.g = 0.0f; impossibleMarkers.color.b = 0.8f; impossibleMarkers.color.a = 1.0;

  visualization_msgs::Marker usableMarkers; usableMarkers.header.frame_id = "/world"; usableMarkers.header.stamp = ros::Time::now(); usableMarkers.ns = "usable_markers";
  usableMarkers.type = shape; usableMarkers.action = visualization_msgs::Marker::ADD;
  usableMarkers.color.r = 0.0f; usableMarkers.color.g = 0.0f; usableMarkers.color.b = 1.0f; usableMarkers.color.a = 0.4;

  visualization_msgs::Marker trimmedOccupiedMarkers; trimmedOccupiedMarkers.header.frame_id = "/world"; trimmedOccupiedMarkers.header.stamp = ros::Time::now(); trimmedOccupiedMarkers.ns = "occupied_markers_trimmed";
  trimmedOccupiedMarkers.type = shape; trimmedOccupiedMarkers.action = visualization_msgs::Marker::ADD;
  trimmedOccupiedMarkers.color.r = 1.0f; trimmedOccupiedMarkers.color.g = 0.0f; trimmedOccupiedMarkers.color.b = 0.0f; trimmedOccupiedMarkers.color.a = 1.0;

  visualization_msgs::Marker trimmedFreeMarkers; trimmedFreeMarkers.header.frame_id = "/world"; trimmedFreeMarkers.header.stamp = ros::Time::now(); trimmedFreeMarkers.ns = "free_markers_trimmed";
  trimmedFreeMarkers.type = shape; trimmedFreeMarkers.action = visualization_msgs::Marker::ADD;
  trimmedFreeMarkers.color.r = 0.0f; trimmedFreeMarkers.color.g = 1.0f; trimmedFreeMarkers.color.b = 0.0f; trimmedFreeMarkers.color.a = 1.0;

  visualization_msgs::Marker trimmedUnknownMarkers; trimmedUnknownMarkers.header.frame_id = "/world"; trimmedUnknownMarkers.header.stamp = ros::Time::now(); trimmedUnknownMarkers.ns = "unknown_markers_trimmed";
  trimmedUnknownMarkers.type = shape; trimmedUnknownMarkers.action = visualization_msgs::Marker::ADD;
  trimmedUnknownMarkers.color.r = 1.0f; trimmedUnknownMarkers.color.g = 1.0f; trimmedUnknownMarkers.color.b = 0.0f; trimmedUnknownMarkers.color.a = 1.0;

  hector_moveit_navigation::NavigationGoal goal;
  ROS_INFO("Waiting for action server to start");
  ac.waitForServer();
  ROS_INFO("Action server started");

//taking off
  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = 0;
  goal.goal_pose.position.z = 5;
  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = 0;
  goal.goal_pose.orientation.w = 1;
  ac.sendGoal(goal);
  
// checking takeoff
  bool finished_before_timeout = ac.waitForResult();
  if(finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Takeoff finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Takeoff did not finish before the time out");

  ROS_INFO("Waiting for gtsp_solver and distance_publisher to subscribe");
  ros::Rate poll_rate(100);
  while(point_cloud_publisher.getNumSubscribers() == 0)
    poll_rate.sleep();
  while(goal_distance_publisher.getNumSubscribers() == 0)
    poll_rate.sleep();

// TODO: have a wait if no new information is gained so that GLNS does not get an empty graph or have something in GLNS that skips empty graphs
// problem is that GLNS will crash if empty set
  while (ros::ok()){
    ros::spinOnce();
// initializing variables
    id4Markers = 0; int countFreeFull = 0; int countOccFull = 0; int countUnknownFull = 0; markerSize = 0;
// getting sizes of free, occupied, and unknown for full octree
    ROS_INFO("Getting occupied, free, occupied trimmed, and free trimmed");
    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it){
      markerSize = it.getSize();
      if(it->getValue()>thresholdOcc){
        fullOccupiedMarkers.pose.position.x = it.getX(); fullOccupiedMarkers.pose.position.y = it.getY(); fullOccupiedMarkers.pose.position.z = it.getZ();
        fullOccupiedMarkers.id = id4Markers;
        fullOccupiedMarkers.scale.x = markerSize; fullOccupiedMarkers.scale.y = markerSize; fullOccupiedMarkers.scale.z = markerSize;
        occArrayFull.markers.push_back(fullOccupiedMarkers);
        countOccFull = countOccFull + 1;
      }
      else if(it->getValue()<thresholdFree){
        fullFreeMarkers.pose.position.x = it.getX(); fullFreeMarkers.pose.position.y = it.getY(); fullFreeMarkers.pose.position.z = it.getZ();
        fullFreeMarkers.id = id4Markers;
        fullFreeMarkers.scale.x = markerSize; fullFreeMarkers.scale.y = markerSize; fullFreeMarkers.scale.z = markerSize;
        freeArrayFull.markers.push_back(fullFreeMarkers);
        countFreeFull = countFreeFull + 1;
      }
      else{
        fullUnknownMarkers.pose.position.x = it.getX(); fullUnknownMarkers.pose.position.y = it.getY(); fullUnknownMarkers.pose.position.z = it.getZ();
        fullUnknownMarkers.id = id4Markers;
        fullUnknownMarkers.scale.x = markerSize; fullUnknownMarkers.scale.y = markerSize; fullUnknownMarkers.scale.z = markerSize;
        countUnknownFull = countUnknownFull + 1;
      }
      id4Markers = id4Markers + 1;
    }
// getting sizes of free, occupied, and unknown for trimmed octree
    int countFreeTrimmed = 0; int countOccTrimmed = 0; int countUnknownTrimmed = 0; markerSize = 0;
    for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
      markerSize = it.getSize();
      if(it->getValue()>thresholdOcc){
        trimmedOccupiedMarkers.pose.position.x = it.getX(); trimmedOccupiedMarkers.pose.position.y = it.getY(); trimmedOccupiedMarkers.pose.position.z = it.getZ();
        trimmedOccupiedMarkers.id = id4Markers;
        trimmedOccupiedMarkers.scale.x = markerSize; trimmedOccupiedMarkers.scale.y = markerSize; trimmedOccupiedMarkers.scale.z = markerSize;
        occArrayTrimmed.markers.push_back(trimmedOccupiedMarkers);
        countOccTrimmed = countOccTrimmed + 1;
      }
      else if(it->getValue()<thresholdFree){
        trimmedFreeMarkers.pose.position.x = it.getX(); trimmedFreeMarkers.pose.position.y = it.getY(); trimmedFreeMarkers.pose.position.z = it.getZ();
        trimmedFreeMarkers.id = id4Markers;
        trimmedFreeMarkers.scale.x = markerSize; trimmedFreeMarkers.scale.y = markerSize; trimmedFreeMarkers.scale.z = markerSize;
        freeArrayTrimmed.markers.push_back(trimmedFreeMarkers);
        countFreeTrimmed = countFreeTrimmed + 1;
      }
      else{
        trimmedUnknownMarkers.pose.position.x = it.getX(); trimmedUnknownMarkers.pose.position.y = it.getY(); trimmedUnknownMarkers.pose.position.z = it.getZ();
        trimmedUnknownMarkers.id = id4Markers;
        trimmedUnknownMarkers.scale.x = markerSize; trimmedUnknownMarkers.scale.y = markerSize; trimmedUnknownMarkers.scale.z = markerSize;
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
    countFreeFull = 0; countOccFull = 0; countUnknownFull = 0;
    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it){
      if(it->getValue()>thresholdOcc){
        cloudOccFull->points[countOccFull].x = it.getX(); cloudOccFull->points[countOccFull].y = it.getY(); cloudOccFull->points[countOccFull].z = it.getZ();
        countOccFull = countOccFull + 1;
      }
      else if(it->getValue()<thresholdFree){
        cloudFreeFull->points[countFreeFull].x = it.getX(); cloudFreeFull->points[countFreeFull].y = it.getY(); cloudFreeFull->points[countFreeFull].z = it.getZ();
        freeSizeFull.push_back(it.getSize());
        countFreeFull = countFreeFull + 1;
      }
      else{
        cloudUnknownFull->points[countUnknownFull].x = it.getX(); cloudUnknownFull->points[countUnknownFull].y = it.getY(); cloudUnknownFull->points[countUnknownFull].z = it.getZ();
        countUnknownFull = countUnknownFull + 1;
      }
    }
// creating point clouds for free, occupied, and unknown voxels for trimmed point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFreeTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudUnknownTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
    cloudFreeTrimmed->width = countFreeTrimmed; cloudFreeTrimmed->height = 1; cloudFreeTrimmed->points.resize (cloudFreeTrimmed->width * cloudFreeTrimmed->height);
    cloudOccTrimmed->width = countOccTrimmed; cloudOccTrimmed->height = 1; cloudOccTrimmed->points.resize (cloudOccTrimmed->width * cloudOccTrimmed->height);
    cloudUnknownTrimmed->width = countUnknownTrimmed; cloudUnknownTrimmed->height = 1; cloudUnknownTrimmed->points.resize (cloudUnknownTrimmed->width * cloudUnknownTrimmed->height);
    std::vector<double> freeSizeTrimmed;
    countFreeTrimmed = 0; countOccTrimmed = 0; countUnknownTrimmed = 0;
    for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
      if(it->getValue()>thresholdOcc){
        cloudOccTrimmed->points[countOccTrimmed].x = it.getX(); cloudOccTrimmed->points[countOccTrimmed].y = it.getY(); cloudOccTrimmed->points[countOccTrimmed].z = it.getZ();
        countOccTrimmed = countOccTrimmed + 1;
      }
      else if(it->getValue()<thresholdFree){
        cloudFreeTrimmed->points[countFreeTrimmed].x = it.getX(); cloudFreeTrimmed->points[countFreeTrimmed].y = it.getY(); cloudFreeTrimmed->points[countFreeTrimmed].z = it.getZ();
        freeSizeTrimmed.push_back(it.getSize());
        countFreeTrimmed = countFreeTrimmed + 1;
      }
      else{
        cloudUnknownTrimmed->points[countUnknownTrimmed].x = it.getX(); cloudUnknownTrimmed->points[countUnknownTrimmed].y = it.getY(); cloudUnknownTrimmed->points[countUnknownTrimmed].z = it.getZ();
        countUnknownTrimmed = countUnknownTrimmed + 1;
      }
    }

// remove running PCs
    ROS_INFO("Removing visited and impossible voxels");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFreeNew1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFreeNew2 (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> cloudFreeNew1ID;
    std::vector<int> cloudFreeNew2ID;
    std::vector<double> cloudFreeSizeNew1;
    std::vector<double> cloudFreeSizeNew2;
    for(int j = 0; j<countFreeFull; j++){ // removes "runningImpossiblePC"
      searchPoint1 = cloudFreeFull->at(j);
      int checkPoint = 0;
      for(int i = 0; i<runningImpossiblePC->size();i++){
        searchPoint2 = runningImpossiblePC->at(i);
        if(checkIfPointsAreTheSame(searchPoint1,searchPoint2)==1){
          checkPoint = 1;
        }
      }
      if(checkPoint == 0){
        cloudFreeNew1->push_back(cloudFreeFull->at(j));
        cloudFreeNew1ID.push_back(j);
        cloudFreeSizeNew1.push_back(freeSizeFull.at(j));
      }
    }
    for(int j = 0; j<cloudFreeNew1ID.size();j++){ // removing "runningVisitedPC"
      searchPoint1 = cloudFreeNew1->at(j);
      if(radiusSearch(runningVisitedPC,searchPoint1,sizeOfUAV)==0){ //if greater than 0 then searchPoint1 is within the radius, of sizeOfUAV, in runningVisitedPC
        cloudFreeNew2->push_back(cloudFreeNew1->at(j));
        cloudFreeNew2ID.push_back(j);
        cloudFreeSizeNew2.push_back(cloudFreeSizeNew1.at(j));
      }
    }

// getting impossible and usable voxels
    ROS_INFO("Finding usable voxels");
    pcl::PointCloud<pcl::PointXYZ>::Ptr usableCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr usableCloudKNN (new pcl::PointCloud<pcl::PointXYZ>);
    if(cloudFreeNew2->size()!=0 && countOccFull!=0 && countUnknownFull!=0){
      std::vector<int> impossibleVoxelIDs;
      std::vector<int> possibleVoxelIDs;
      std::vector<int> usableVoxelIDs;
      for(int j = 0; j<cloudFreeNew2ID.size(); j++){
        searchPoint = cloudFreeNew2->at(j);
        if(radiusSearch(cloudOccFull,searchPoint,minRadius) > 0){ // gets impossible voxels. if greater than 0 then occupied points are within the radius
          impossibleVoxelIDs.push_back(j);
        }
        else{ // gets all possible voxels. if 0 then no occupied points within radius
          possibleVoxelIDs.push_back(j);
        }
      }
      for(int j = 0; j<possibleVoxelIDs.size(); j++){ // gets all usable voxels
        tempID = possibleVoxelIDs.at(j);
        searchPoint = cloudFreeNew2->at(tempID);
        if (radiusSearch(cloudOccTrimmed,searchPoint,maxRadius) > 0){
          usableVoxelIDs.push_back(tempID);
          usableCloud->push_back(searchPoint);
        }
      }
      pcl::KdTree<pcl::PointXYZ>::Ptr usableCloudKNNTree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      usableCloudKNNTree->setInputCloud(usableCloud);
      std::vector<int> UCKNN_indices (MAX_TOUR_SIZE);
      std::vector<float> UCKNN_dist (MAX_TOUR_SIZE);
      int sizeOfKNNReturn = 0;
      sizeOfKNNReturn = usableCloudKNNTree->nearestKSearch(pcl::PointXYZ(currentPose.position.x, currentPose.position.y, currentPose.position.z), MAX_TOUR_SIZE, UCKNN_indices, UCKNN_dist);
      for(int j = 0; j<sizeOfKNNReturn;j++){ // making the truncated usable cloud
        usableCloudKNN->push_back(usableCloud->points[UCKNN_indices[j]]);
      }

// creating voxel arrays to publishing
      pcl::PointCloud<pcl::PointXYZ>::Ptr impossibleCloud (new pcl::PointCloud<pcl::PointXYZ>);
      for(int j = 0; j<impossibleVoxelIDs.size(); j++){
        impossibleMarkers.pose.position.x = cloudFreeNew2->at(impossibleVoxelIDs[j]).x; impossibleMarkers.pose.position.y = cloudFreeNew2->at(impossibleVoxelIDs[j]).y; impossibleMarkers.pose.position.z = cloudFreeNew2->at(impossibleVoxelIDs[j]).z;
        impossibleMarkers.id = id4Markers;
        // impossibleMarkers.scale.x = cloudFreeSizeNew2.at(impossibleVoxelIDs[j]); impossibleMarkers.scale.y = cloudFreeSizeNew2.at(impossibleVoxelIDs[j]); impossibleMarkers.scale.z = cloudFreeSizeNew2.at(impossibleVoxelIDs[j]);
        impossibleMarkers.scale.x = 1; impossibleMarkers.scale.y = 1; impossibleMarkers.scale.z = 1;
        impossibleArray.markers.push_back(impossibleMarkers);
        id4Markers++;
        runningImpossiblePC->push_back(cloudFreeNew2->at(impossibleVoxelIDs[j]));
      }
      for(int j = 0; j<usableVoxelIDs.size(); j++){
        usableMarkers.pose.position.x = cloudFreeNew2->at(usableVoxelIDs[j]).x; usableMarkers.pose.position.y = cloudFreeNew2->at(usableVoxelIDs[j]).y; usableMarkers.pose.position.z = cloudFreeNew2->at(usableVoxelIDs[j]).z;
        usableMarkers.id = id4Markers;
        // usableMarkers.scale.x = cloudFreeSizeNew2.at(usableVoxelIDs[j]); usableMarkers.scale.y = cloudFreeSizeNew2.at(usableVoxelIDs[j]); usableMarkers.scale.z = cloudFreeSizeNew2.at(usableVoxelIDs[j]);
        usableMarkers.scale.x = 1; usableMarkers.scale.y = 1; usableMarkers.scale.z = 1;
        usableArray.markers.push_back(usableMarkers);
        id4Markers++;
        runningVisitedPC->push_back(cloudFreeNew2->at(usableVoxelIDs[j]));
      }
    }

    if(usableCloudKNN->size() <= 1){
      ROS_INFO("Done with everything.");
      break;
    }
// sending usable point cloud to Naik's code
    ROS_INFO("Sending usable point cloud to gtsp solver");
    pcl::PCLPointCloud2 pcl_pc;
    pcl::toPCLPointCloud2(*usableCloudKNN, pcl_pc);
    sensor_msgs::PointCloud2 usablePC2;
    pcl_conversions::fromPCL(pcl_pc, usablePC2);
    // ROS_INFO("Point cloud size: %zu", usableCloud->size());

// publishing voxel arrays
    occArrayTrimmed_pub.publish(occArrayTrimmed);
    // std::cout << "occupied array trimmed: " << occArrayTrimmed.markers.size() << std::endl;
    freeArrayTrimmed_pub.publish(freeArrayTrimmed);
    // std::cout << "free array trimmed: " << freeArrayTrimmed.markers.size() << std::endl;
    occArrayFull_pub.publish(occArrayFull);
    // std::cout << "occupied array full: " << occArrayFull.markers.size() << std::endl;
    freeArrayFull_pub.publish(freeArrayFull);
    // std::cout << "free array full: " << freeArrayFull.markers.size() << std::endl;
    impossibleArray_pub.publish(impossibleArray);
    // std::cout << "impossible array: " << impossibleArray.markers.size() << std::endl;
    usableArray_pub.publish(usableArray);
    // std::cout << "usable array: " << usableArray.markers.size() << std::endl;

    ROS_INFO("Publishing point cloud and waiting on gtsp tour");
    point_cloud_publisher.publish(usablePC2);
    while(!tour_ready)
    {
      ros::spinOnce();
      poll_rate.sleep();
    }
    tour_ready = false;
    ROS_INFO("Tour ready");

    ros::Time begin = ros::Time::now();
    ros::Duration elapsed = ros::Time::now()-begin;
    ros::Time moveitBegin = ros::Time::now();
    ros::Duration moveitElapsed = ros::Time::now()-moveitBegin;

    pcl::KdTree<pcl::PointXYZ>::Ptr gtspTree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    gtspTree->setInputCloud(usableCloudKNN);
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    gtspTree->nearestKSearch(pcl::PointXYZ(currentPose.position.x, currentPose.position.y, currentPose.position.z), 1, nn_indices, nn_dists);
    std::vector<int>::iterator it=std::find(tour.begin(),tour.end(),tour[nn_indices[0]]);
    ROS_INFO("Starting tour index: %d", nn_indices[0]);
// executes tour based on GTSP output
    int currentPointNumber = std::distance(tour.begin(),it);
    int countMoveit = 0;
    int orientationStatus = 0;
    while(elapsed.sec<60 && !tspDone){
      ROS_INFO("Starting while loop for TSP");
      ROS_INFO("Current point number: %d", currentPointNumber);
      ROS_INFO("Current tour value: %d", tour[currentPointNumber]);
      // ROS_INFO("Current pont number: %d", currentPointNumber);
      // goal point for moveit
      goal.goal_pose.position.x = usableCloudKNN->points[tour[currentPointNumber]].x;
      goal.goal_pose.position.y = usableCloudKNN->points[tour[currentPointNumber]].y;
      goal.goal_pose.position.z = usableCloudKNN->points[tour[currentPointNumber]].z;
      ROS_INFO("Moving to point: (%f,%f,%f)", goal.goal_pose.position.x,goal.goal_pose.position.y,goal.goal_pose.position.z);
      tf2::Quaternion UAVOrientation;
      switch(orientationStatus){
        case 0:
          UAVOrientation.setRPY(0,0,0);
          orientationStatus++;
          break;
        case 1:
          UAVOrientation.setRPY(0,0,M_PI_2);
          orientationStatus++;
          break;
        case 2:
          UAVOrientation.setRPY(0,0,M_PI);
          orientationStatus++;
          break;
        case 3:
          UAVOrientation.setRPY(0,0,-M_PI_2);
          orientationStatus = 0;
          break;
        default:
          ROS_INFO("Error: goal orientation is not working");
      }
      goal.goal_pose.orientation.x = UAVOrientation.x();
      goal.goal_pose.orientation.y = UAVOrientation.y();
      goal.goal_pose.orientation.z = UAVOrientation.z();
      goal.goal_pose.orientation.w = UAVOrientation.w();

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      // waiting for 30 seconds to reach goal before sending next point
      finished_before_timeout = false;
      finished_before_timeout = ac.waitForResult(ros::Duration(30));
      if(finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
      }
      else{
        ac.cancelAllGoals();
        ROS_INFO("Action did not finish before the time out.");
      }
      if(currentPointNumber >= tour.size()-1){ // check if the current tour point is the last one and loop
        currentPointNumber = 0;
      }
      else{
        currentPointNumber++;
      }
      if(tour.size()-1 == countMoveit){ // check if all tour points have been visited
        tspDone = true;
      }
      else{
        countMoveit++;
        elapsed = ros::Time::now()-begin;
      }
      ROS_INFO("Current tour[%d] elapsed time: %d seconds",loopNumber+1, elapsed.sec);
    }

    ROS_INFO("Clearing all voxel maps stored");
    occArrayTrimmed.markers.clear();
    freeArrayTrimmed.markers.clear();
    occArrayFull.markers.clear();
    freeArrayFull.markers.clear();
    impossibleArray.markers.clear();
    usableArray.markers.clear();

// saving data to .pcd if "fileSave" is 1
    // if(cloudFreeNew2->size()!=0 && countOccFull!=0 && countUnknownFull!=0 && fileSave==1){ // saving point clouds to .PCD
    //   pcl::io::savePCDFileASCII("free.pcd", *cloudFreeNew2);
    //   pcl::io::savePCDFileASCII("occ.pcd", *cloudOccFull);
    //   pcl::io::savePCDFileASCII("unknown.pcd", *cloudUnknownFull);
    // }

    // std::cout << "running sizes: " << runningImpossiblePC->size() << " " << runningVisitedPC->size() << std::endl;
    std::cout << "=============loop number: " << loopNumber+1 << "========================" << std::endl; // outputting loop number
    loopNumber++;
    r.sleep();
  }
  return 0;
}
