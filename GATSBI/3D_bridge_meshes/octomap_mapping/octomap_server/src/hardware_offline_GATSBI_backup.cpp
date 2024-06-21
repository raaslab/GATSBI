#define _USE_MATH_DEFINES
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
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
#include <nav_msgs/Odometry.h>
#include <mutex>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define MAX_TOUR_SIZE 25

// random comment here

// global variables
int parameter_resolution = 0.5;
int resolution = 1; // need to change this in the launch file to have an affect
octomap::OcTree* fullOcTree = NULL;
octomap::OcTree* trimmedOcTree = NULL;
int countCB = 0;
int fileSave = 0;
int markerSize = parameter_resolution;
float xData=0.0;
float yData=0.0;
float zData=0.0;
int positionCount = 0;
std::vector<int> tour;
double moveit_distance = -1;
bool tour_ready = false;
bool length_ready = false;
geometry_msgs::Pose currentPose;
pcl::PointCloud<pcl::PointXYZ>::Ptr runningVisitedVoxels (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<double> tempzFilteredSize;
std_msgs::Float64 resetFlag_msg;
pcl::PointCloud<pcl::PointXYZ>::Ptr visitedPointsList (new pcl::PointCloud<pcl::PointXYZ>);
tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped base_link_to_world_ned;
std::mutex mutex_;
std::mutex mutex_trimmed;

double viewpoint_check_distance = 2 / resolution;

float checkDistance = 25; // the allowed difference in the expected distance and actual distance
int replanningTime = 15; // replanning time

double flight_Distance = 0;
double previousX = 0;
double previousY = 0;
double previousZ = 0;

bool checkIfDroneInVoxel(pcl::PointXYZ POI, pcl::PointXYZ voxelPoint, float orientation){

  if((voxelPoint.x - 0.5) <= POI.x && POI.x <= (voxelPoint.x + 0.5) &&
     (voxelPoint.y - 0.5) <= POI.y && POI.y <= (voxelPoint.y + 0.5) &&
     (voxelPoint.z - 0.5) <= POI.z && POI.z <= (voxelPoint.z + 0.5))
  {
    return true;
  }
  return false;

}

bool near_occupied_cell(octomap::OcTreeKey cell)
{
  for(int x_it = (-1 * viewpoint_check_distance); x_it < viewpoint_check_distance; x_it++)
  {
    for(int y_it = (-1 * viewpoint_check_distance); y_it < viewpoint_check_distance; y_it++)
    {
      octomap::OcTreeKey neighbor_key = cell;
      neighbor_key[0] = neighbor_key[0] + x_it;
      neighbor_key[1] = neighbor_key[1] + y_it;
      octomap::OcTreeNode* neighbor_result = fullOcTree->search(neighbor_key);
      if(neighbor_result)
      {
        if(fullOcTree->isNodeOccupied(neighbor_result))
        {
          return true;
        }
      }
    }
  }
  return false;
}

void findViewPoints(float POIX,float POIY,float POIZ,float minRadius,float maxRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr outputPoints, std::vector<float>* outputOrientationYaw)
{
  octomap::point3d end_point(POIX, POIY, POIZ);
  octomap::point3d start_pos_x(POIX + maxRadius, POIY, POIZ);
  octomap::point3d start_neg_x(POIX - maxRadius, POIY, POIZ);
  octomap::point3d start_pos_y(POIX, POIY + maxRadius, POIZ);
  octomap::point3d start_neg_y(POIX, POIY - maxRadius, POIZ);

  octomap::KeyRay ray;
  std::unique_lock<std::mutex> lock(mutex_);
  if(fullOcTree->computeRayKeys(start_pos_x, end_point, ray))
  {
    octomap::OcTreeNode* result;
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      octomap::point3d point = fullOcTree->keyToCoord(*it);    
      result = fullOcTree->search(*it);
      if(result)
      {
          if(fullOcTree->isNodeOccupied(result))
          {
            break;
          }
          
          if(!near_occupied_cell(*it))
          {
            if(point.x() >= POIX + minRadius)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
              //outputOrientationYaw->push_back(270);ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 270", POIX, POIY, POIZ, point.x(), point.y(), point.z());
            }
          }
      }
      else {
        if(!near_occupied_cell(*it)) 
        {
          if(point.x() >= POIX + minRadius)
          {
            outputPoints->push_back({point.x(), point.y(), point.z()});
            outputOrientationYaw->push_back(270);
          }
        }
      }
    }
  }

  ray.reset();
  if(fullOcTree->computeRayKeys(start_neg_x, end_point, ray))
  {
    octomap::OcTreeNode* result;
    
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      result = fullOcTree->search(*it);
      octomap::point3d point = fullOcTree->keyToCoord(*it);    
      if(result)
      {
          if(fullOcTree->isNodeOccupied(result))
          {
            break;
          }
          
          if(!near_occupied_cell(*it))
          {
            if(point.x() <= POIX - minRadius)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(90);
              //ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 90", POIX, POIY, POIZ, point.x(), point.y(), point.z());
            }
          }

      }
      else {
        if(!near_occupied_cell(*it))
        {
          if(point.x() <= POIX - minRadius)
          {
            outputPoints->push_back({point.x(), point.y(), point.z()});
            outputOrientationYaw->push_back(90);
          }
        }
      }
    }
  }

  ray.reset();
  if(fullOcTree->computeRayKeys(start_pos_y, end_point, ray))
  {
    octomap::OcTreeNode* result;
      
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      result = fullOcTree->search(*it);
      octomap::point3d point = fullOcTree->keyToCoord(*it);  
      if(result)
      {
          if(fullOcTree->isNodeOccupied(result))
          {
            break;
          }
          
          if(!near_occupied_cell(*it))
          {
            if(point.y() >= POIY + minRadius)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(180);
              //ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 180", POIX, POIY, POIZ, point.x(), point.y(), point.z());
            }
          }
      }
      else {
        if(!near_occupied_cell(*it))
        {
          if(point.y() >= POIY + minRadius)
          {
            outputPoints->push_back({point.x(), point.y(), point.z()});
            outputOrientationYaw->push_back(180);
          }
        }
      }
    }
  }

  ray.reset();
  if(fullOcTree->computeRayKeys(start_neg_y, end_point, ray))
  {
    octomap::OcTreeNode* result;
     
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      result = fullOcTree->search(*it);
      octomap::point3d point = fullOcTree->keyToCoord(*it);   
      if(result)
      {
          if(fullOcTree->isNodeOccupied(result))
          {
            break;
          }
         
          if(!near_occupied_cell(*it))
          { 
            if(point.y() <= POIY - minRadius)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(0);
              //ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 0", POIX, POIY, POIZ, point.x(), point.y(), point.z());
            }
          }
      }
      else {
        if(!near_occupied_cell(*it))
        {
          if(point.y() <= POIY - minRadius)
          {
            outputPoints->push_back({point.x(), point.y(), point.z()});
            outputOrientationYaw->push_back(0);
          }
        }
      }
    }
  }
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
    
    base_link_to_world_ned = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(5.0) );
    tf2::doTransform(in_stamped, transformed_stamped, base_link_to_world_ned);
    geometry_msgs::Pose out;
    out = transformed_stamped.pose;

    return out;
}

void tourCallback(const gtsp::Tour::ConstPtr& msg){
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

void lengthCallback(const std_msgs::Float64::ConstPtr& msg){
  ROS_INFO("Got length");
  moveit_distance = msg->data;
  ROS_INFO("MoveIt Distance: %f", moveit_distance);
  length_ready = true;
}

void full_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);

  std::unique_lock<std::mutex> lock(mutex_);

  fullOcTree = dynamic_cast<octomap::OcTree*>(absTree);

  // countCB++;
}


void trimmed_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
  std::unique_lock<std::mutex> lock_trimmed(mutex_trimmed);

  trimmedOcTree = dynamic_cast<octomap::OcTree*>(absTree);
  // countCB++;
}


void imu_cb(const geometry_msgs::PointStampedConstPtr& position_data, const geometry_msgs::QuaternionStampedConstPtr& attitude_data){ // imu
  geometry_msgs::Pose odometry_information;
  odometry_information.position = position_data->point;
  odometry_information.orientation = attitude_data->quaternion;

  pcl::PointXYZ tempPoint(odometry_information.position.x,odometry_information.position.y,odometry_information.position.z);
  currentPose = odometry_information;

  if(double_equals(previousX, odometry_information.position.x) &&
     double_equals(previousY, odometry_information.position.y) &&
     double_equals(previousZ, odometry_information.position.z))
    return;

  flight_Distance += sqrt(pow(previousX - odometry_information.position.x,2) + 
                          pow(previousY - odometry_information.position.y,2) + 
                          pow(previousZ - odometry_information.position.z,2));

  previousX = odometry_information.position.x;
  previousY = odometry_information.position.y;
  previousZ = odometry_information.position.z;

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

void visitedPointList_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
  visitedPointsList->clear();
  BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points){
    visitedPointsList->points.push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
  }
}


int main(int argc, char** argv){
  std::ofstream myfile;
  myfile.open("/home/user/bridgeInspection/testInfo.txt");
  std::ofstream myfile1;
  myfile1.open("/home/user/bridgeInspection/testClusters.txt");
  std::ofstream myfileT;
  myfileT.open("/home/user/bridgeInspection/timeVSTrimmedOcc.csv");
  std::ofstream myfileDA; // actual distance
  myfileDA.open("/home/user/bridgeInspection/timeVSDistanceActual.csv");
  std::ofstream myfileDE; // expected distance
  myfileDE.open("/home/user/bridgeInspection/timeVSDistanceExpected.csv");
  std::ofstream myfileCompTime;
  myfileCompTime.open("/home/user/bridgeInspection/timeVSComputationalTime.csv");
  std::ofstream myfileDR; // real distance
  myfileDR.open("/home/user/bridgeInspection/timeVSDistanceReal.csv");
  std::ofstream myfileTestClusters; // real distance
  myfileTestClusters.open("/home/user/bridgeInspection/clusters.csv");
  std::ofstream myFileInspectableVoxels;
  myFileInspectableVoxels.open("/home/user/bridgeInspection/inspectableVoxels.csv");

  std::ofstream myfileClusters;
  myfileClusters.open("/home/user/bridgeInspection/clustersVSTime.csv");

  std::ofstream myFileFlightDistance;
  myFileFlightDistance.open("/home/user/bridgeInspection/flightDistance.csv");


  std::ofstream inspectionPath;
  inspectionPath.open("/home/user/bridgeInspection/inspectionPath.csv");

  myfileClusters << "Time,Inspectable Voxels,Candidate Viewpoints\n";
  myfileTestClusters << "Number," << "Bridge X," << "Bridge Y," << "Bridge Z," << "View X," << "View Y," << "View Z," << "Orientation\n";
  myFileInspectableVoxels << "Number," << "X," << "Y," << "Z," << "Inspected?," << "Notes\n";
  myFileFlightDistance << "Time,Flight Distance, Replanning Time: " << replanningTime << "\n";
  inspectionPath << "X,Y,Z,Yaw\n";

// initializing ROS everything
  ros::init(argc, argv, "kevin");

  ros::NodeHandle n;

  ros::Rate r(0.05); // less than 1 is slower
  ros::Rate distanceSleep(10);
  ros::Rate waitSleep(10);
  
  ros::Time beginT = ros::Time::now();
  ros::Time updateT = ros::Time::now();
  ros::Time updateClusters = ros::Time::now();
  ros::Time compT_begin;
  ros::Time compT_endAlgo;
  ros::Time compT_endGTSP;
  ros::Time compT_endFlight;
  ros::Publisher occArrayTrimmed_pub = n.advertise<visualization_msgs::MarkerArray>("/occArrayTrimmed_pub",1,true);
  ros::Publisher inspectedVoxels_pub = n.advertise<visualization_msgs::MarkerArray>("/inspected_voxels",1,true);
  ros::Publisher freeArrayTrimmed_pub = n.advertise<visualization_msgs::MarkerArray>("/freeArrayTrimmed_pub",1,true);
  ros::Publisher occArrayFull_pub = n.advertise<visualization_msgs::MarkerArray>("/occArrayFull_pub",1,true);
  ros::Publisher freeArrayFull_pub = n.advertise<visualization_msgs::MarkerArray>("/freeArrayFull_pub",1,true);
  ros::Publisher impossibleArray_pub = n.advertise<visualization_msgs::MarkerArray>("/impossibleArray_pub",1,true);
  ros::Publisher usableArray_pub = n.advertise<visualization_msgs::MarkerArray>("/usableArray_pub",1,true);
  ros::Publisher point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/gtsp_point_cloud", 1);

  ros::Publisher gtspData_pub = n.advertise<gtsp::GTSPData>("/gtsp_data", 1);
  ros::Publisher resetFlag_pub = n.advertise<std_msgs::Float64>("/resetFlag",1);

  ros::Subscriber fullTree_sub = n.subscribe("/octomap_full",1,full_cb);
  ros::Subscriber trimmedTree_sub = n.subscribe("/octomap_full_trimmed",1,trimmed_cb);
  ros::Subscriber tour_sub = n.subscribe("/gtsp_tour_list", 1, tourCallback);
  ros::Subscriber distance_sub = n.subscribe("/compute_path/length", 1, lengthCallback);
  ros::Subscriber zFiltered_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/zFiltered",1,zFiltered_cb);
  ros::Subscriber zFilteredSize_sub = n.subscribe("/zFilteredSize", 1, zFilteredSize_cb);
  ros::Subscriber visitedPointList_sub = n.subscribe("/visited_point_list", 1, visitedPointList_cb);

  message_filters::Subscriber<geometry_msgs::PointStamped> position_sub(n, "/dji_sdk/local_position", 1);
  message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub(n, "/dji_sdk/attitude", 1);

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::QuaternionStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), position_sub, attitude_sub);
  sync.registerCallback(boost::bind(&imu_cb, _1, _2));

  myfileT << updateT-beginT << "," << "Inspected Bridge" << "," << "Not Inspected Bridge"<<","<< "Occupied(Bridge)," << "Occupied(Total)" << ","<< "Free,Unknown" << std::endl;

// initializing arrays
  pcl::PointXYZ searchPoint;
  visualization_msgs::MarkerArray occArrayTrimmed;
  visualization_msgs::MarkerArray freeArrayTrimmed;
  visualization_msgs::MarkerArray occArrayFull;
  visualization_msgs::MarkerArray freeArrayFull;
  visualization_msgs::MarkerArray impossibleArray;
  visualization_msgs::MarkerArray usableArray;
  visualization_msgs::MarkerArray inspectedVoxels;

// initializing variables and markers
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int loopNumber = 0;
  float minRadius = 3;
  float maxRadius = 10;
  float thresholdOcc = 1.0;
  float thresholdFree = 0.0;
  float sizeOfUAV = 1.0;
  float tempID; int id4Markers; float markerSize;
  bool tspDone = false;
  int structureCovered = 0;
  pcl::PointXYZ searchPoint1;
  pcl::PointXYZ searchPoint2;
  octomap::OcTree::leaf_iterator it;
  octomap::OcTree::leaf_iterator endLeaf;
  myfile<<"startTime: "<<ros::Time::now()<<std::endl;
  myfile<<"minRadius: "<<minRadius<<std::endl;
  myfile<<"maxRadius: "<<maxRadius<<std::endl;
  myfile<<"sizeOfUAV: "<<sizeOfUAV<<std::endl;
  float xMin = -30;
  float xMax = 30;
  float yMin = -30;
  float yMax = 30;
  float zMin = 0.2;
  float zMax = 15;

  int totalFrontierPlans = 0;

  std::vector<geometry_msgs::Pose> explored;

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

  visualization_msgs::Marker inspectedVoxelMarkers; inspectedVoxelMarkers.header.frame_id = "/world"; inspectedVoxelMarkers.header.stamp = ros::Time::now(); inspectedVoxelMarkers.ns = "inspected_voxel_markers";
  inspectedVoxelMarkers.type = shape; inspectedVoxelMarkers.action = visualization_msgs::Marker::ADD;
  inspectedVoxelMarkers.color.r = 1.0f; inspectedVoxelMarkers.color.g = 0.0f; inspectedVoxelMarkers.color.b = 0.0f; inspectedVoxelMarkers.color.a = 1.0;
  
  ros::Time totalRunTime = ros::Time::now();


  ROS_INFO("Waiting for gtsp_solver and distance_publisher to subscribe");
  ros::Rate poll_rate(100);
  while(point_cloud_publisher.getNumSubscribers() == 0)
  {
    ROS_INFO("Inside wait");
    poll_rate.sleep();
  }

  ROS_INFO("Finished");

  while (ros::ok()){ //Main while loop
    compT_begin = ros::Time::now();
    resetFlag_msg.data=1;
    resetFlag_pub.publish(resetFlag_msg);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
// initializing variables
    id4Markers = 0; int countFreeFull = 0; int countOccFull = 0; int countUnknownFull = 0; markerSize = 0; int allCountOccInBounds = 0;
// getting sizes of free, occupied, and unknown for full octree
    fullOccupiedMarkers.header.stamp = ros::Time::now();
    fullFreeMarkers.header.stamp = ros::Time::now();
    fullUnknownMarkers.header.stamp = ros::Time::now();
   
    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it){
      markerSize = it.getSize();
      if(it->getValue()>thresholdOcc){
        fullOccupiedMarkers.pose.position.x = it.getX(); fullOccupiedMarkers.pose.position.y = it.getY(); fullOccupiedMarkers.pose.position.z = it.getZ();
        fullOccupiedMarkers.id = id4Markers;
        fullOccupiedMarkers.scale.x = markerSize; fullOccupiedMarkers.scale.y = markerSize; fullOccupiedMarkers.scale.z = markerSize;
        occArrayFull.markers.push_back(fullOccupiedMarkers);
        countOccFull = countOccFull + 1;
        if(xMin<it.getX()<xMax && yMin<it.getY()<yMax && zMin<it.getZ()<zMax){
          allCountOccInBounds++;
        }
      } else if(it->getValue()<thresholdFree){
        fullFreeMarkers.pose.position.x = it.getX(); fullFreeMarkers.pose.position.y = it.getY(); fullFreeMarkers.pose.position.z = it.getZ();
        fullFreeMarkers.id = id4Markers;
        fullFreeMarkers.scale.x = markerSize; fullFreeMarkers.scale.y = markerSize; fullFreeMarkers.scale.z = markerSize;
        freeArrayFull.markers.push_back(fullFreeMarkers);
        countFreeFull = countFreeFull + 1;
      } else{
        fullUnknownMarkers.pose.position.x = it.getX(); fullUnknownMarkers.pose.position.y = it.getY(); fullUnknownMarkers.pose.position.z = it.getZ();
        fullUnknownMarkers.id = id4Markers;
        fullUnknownMarkers.scale.x = markerSize; fullUnknownMarkers.scale.y = markerSize; fullUnknownMarkers.scale.z = markerSize;
        countUnknownFull = countUnknownFull + 1;
      }
      id4Markers = id4Markers + 1;
    }

// getting sizes of free, occupied, and unknown for trimmed octree
    int countFreeTrimmed = 0; int countOccTrimmed = 0; int countUnknownTrimmed = 0; markerSize = 0;
    trimmedOccupiedMarkers.header.stamp = ros::Time::now();
    trimmedFreeMarkers.header.stamp = ros::Time::now();
    trimmedUnknownMarkers.header.stamp = ros::Time::now();
    for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
      markerSize = it.getSize();
      if(it->getValue()>thresholdOcc){
        trimmedOccupiedMarkers.pose.position.x = it.getX(); trimmedOccupiedMarkers.pose.position.y = it.getY(); trimmedOccupiedMarkers.pose.position.z = it.getZ();
        trimmedOccupiedMarkers.id = id4Markers;
        trimmedOccupiedMarkers.scale.x = markerSize; trimmedOccupiedMarkers.scale.y = markerSize; trimmedOccupiedMarkers.scale.z = markerSize;
        occArrayTrimmed.markers.push_back(trimmedOccupiedMarkers);
        countOccTrimmed = countOccTrimmed + 1;
      } else if(it->getValue()<thresholdFree){
        trimmedFreeMarkers.pose.position.x = it.getX(); trimmedFreeMarkers.pose.position.y = it.getY(); trimmedFreeMarkers.pose.position.z = it.getZ();
        trimmedFreeMarkers.id = id4Markers;
        trimmedFreeMarkers.scale.x = markerSize; trimmedFreeMarkers.scale.y = markerSize; trimmedFreeMarkers.scale.z = markerSize;
        freeArrayTrimmed.markers.push_back(trimmedFreeMarkers);
        countFreeTrimmed = countFreeTrimmed + 1;
      } else{
        trimmedUnknownMarkers.pose.position.x = it.getX(); trimmedUnknownMarkers.pose.position.y = it.getY(); trimmedUnknownMarkers.pose.position.z = it.getZ();
        trimmedUnknownMarkers.id = id4Markers;
        trimmedUnknownMarkers.scale.x = markerSize; trimmedUnknownMarkers.scale.y = markerSize; trimmedUnknownMarkers.scale.z = markerSize;
        countUnknownTrimmed = countUnknownTrimmed + 1;
      }
      id4Markers = id4Markers + 1;
    }


    markerSize = parameter_resolution;
    inspectedVoxelMarkers.header.stamp = ros::Time::now();
    for(int markerIt = 0; markerIt < runningVisitedVoxels->size(); markerIt++)
    {
      inspectedVoxelMarkers.pose.position.x = runningVisitedVoxels->at(markerIt).x; inspectedVoxelMarkers.pose.position.y = runningVisitedVoxels->at(markerIt).y; inspectedVoxelMarkers.pose.position.z = runningVisitedVoxels->at(markerIt).z;
      inspectedVoxelMarkers.id = id4Markers;
      inspectedVoxelMarkers.scale.x = markerSize; inspectedVoxelMarkers.scale.y = markerSize; inspectedVoxelMarkers.scale.z = markerSize;
      inspectedVoxels.markers.push_back(inspectedVoxelMarkers);
      id4Markers = id4Markers + 1;
    }
    inspectedVoxels_pub.publish(inspectedVoxels);


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

    ROS_INFO("\nRemoving previously seen voxels");
    std::cout<<"trimmed point cloud size: " << tempCloudOccTrimmed->size() << std::endl;
    std::cout<<"running visited voxel size: " << runningVisitedVoxels->size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<double> zFilteredSize;
    bool removeVoxel;
    for(int j=0;j<tempCloudOccTrimmed->size();j++){
      removeVoxel = false;
      for(int i=0;i<runningVisitedVoxels->size();i++){
        if(double_equals(tempCloudOccTrimmed->at(j).x, runningVisitedVoxels->at(i).x) &&
           double_equals(tempCloudOccTrimmed->at(j).y,runningVisitedVoxels->at(i).y) &&
           double_equals(tempCloudOccTrimmed->at(j).z,runningVisitedVoxels->at(i).z)){
          removeVoxel = true;
          break;
        }
      }
      if(!removeVoxel){
        cloudOccTrimmed->push_back(tempCloudOccTrimmed->at(j));
        zFilteredSize.push_back(tempzFilteredSize.at(j));
      }
    }

    for(int j=0;j<runningVisitedVoxels->size();j++){
        myFileInspectableVoxels << j << "," << runningVisitedVoxels->at(j).x << ","
                                << runningVisitedVoxels->at(j).y << "," << runningVisitedVoxels->at(j).z
                                << ",Yes\n";
    }
    updateT = ros::Time::now();

    myfileT << updateT-beginT << "," << runningVisitedVoxels->size() << "," << tempCloudOccTrimmed->size()-runningVisitedVoxels->size()<<","<< countOccTrimmed << "," << countOccFull << ","<< countFreeFull << "," << countUnknownFull << std::endl;
    myFileFlightDistance << updateT-beginT << "," << flight_Distance << std::endl;

    std::cout<<"size after removing running visited from trimmed point cloud: " << cloudOccTrimmed->size() << std::endl;
    std::cout<<"size of zFilteredSize: " <<zFilteredSize.size()<<std::endl;

    float tempX1; float tempY1; float tempZ1; float tempX2; float tempY2; float tempZ2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inspectingPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr viewPoints (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> point2ClusterMapping;
    std::vector<double> tempRes;
    std::vector<float> tempOrientation;
    std::vector<double> tempRes1;
    std::vector<float> tempOrientation1;
    std::vector<float> orientationOfCPs;
    int numOfCluster = 0;
    myfile1<<"===="<<loopNumber<<"===="<<std::endl;
    for(int j = 0; j<cloudOccTrimmed->size();j++){
      
      tempX1 = cloudOccTrimmed->points[j].x;
      tempY1 = cloudOccTrimmed->points[j].y;
      tempZ1 = cloudOccTrimmed->points[j].z;

      tempPoints->clear();
      tempPoints1->clear();
      findViewPoints(tempX1, tempY1, tempZ1, minRadius, maxRadius, tempPoints,&tempOrientation);

      //findIfVoxelCanBeSeen(tempX1,tempY1,tempZ1,zFilteredSize.at(j),cloudFreeFull,freeSizeFull,minRadius,maxRadius,sizeOfUAV,tempPoints1,&tempRes1,&tempOrientation1);
      //removeVoxelsTooClose(tempPoints1,cloudOccFull,sizeOfUAV+(zFilteredSize.at(j)/2),tempPoints,&tempRes1,&tempRes,&tempOrientation1,&tempOrientation);
        myfile1<<j<<". view point: ";
        myfile1<<cloudOccTrimmed->points[j]<<std::endl;
      if(tempPoints->size()>0){
        viewPoints->push_back(cloudOccTrimmed->points[j]);
        numOfCluster++;
        // myfile1<<"size (CP,Res,ori): "<<"("<<clusteredPoints->size()<<","<<tempRes.size()<<","<<tempOrientation.size()<<")"<<std::endl;
        myfile1<<"clustered points: ";
        myFileInspectableVoxels << runningVisitedVoxels->size() + j << "," 
                                << cloudOccTrimmed->points[j].x << "," << cloudOccTrimmed->points[j].y 
                                << "," << cloudOccTrimmed->points[j].z << ",No,Can be Inspected\n";
      }
      else {
        myFileInspectableVoxels << runningVisitedVoxels->size() + j << "," 
                                << cloudOccTrimmed->points[j].x << "," << cloudOccTrimmed->points[j].y 
                                << "," << cloudOccTrimmed->points[j].z << ",No,Uninspectable\n";
      }

      updateClusters = ros::Time::now();
      myfileClusters << updateClusters - beginT << "," << numOfCluster << "," << viewPoints->size() << std::endl;
      //ROS_INFO("Size of tempPoints: %d", tempPoints->size());
      //ROS_INFO("Size of tempOrientation: %d", tempOrientation.size());
      for(int i = 0; i<tempPoints->size();i++){
        clusteredPoints->push_back(tempPoints->points[i]);
        inspectingPoints->push_back(pcl::PointXYZ(tempX1, tempY1, tempZ1));
        point2ClusterMapping.push_back(numOfCluster);
        orientationOfCPs.push_back(tempOrientation[i]);
        myfileTestClusters << i << "," << tempX1 << "," << tempY1 << "," << tempZ1 << ","
                           << tempPoints->points[i].x << "," << tempPoints->points[i].y << "," << tempPoints->points[i].z << "," << tempOrientation[i] << "\n";
        myfile1<<tempPoints->points[i]<<" "<<tempOrientation[i]<<", ";
      }
      myfile1<<std::endl;
    }
    std::cout << "cloud free size: " << cloudFreeFull->size() << std::endl;
    compT_endAlgo = ros::Time::now();

    if(numOfCluster<2 && structureCovered == 0)
    { // the structure has been covered
      ROS_INFO("Number of frontiers flown to: %d", totalFrontierPlans);
      if(totalFrontierPlans >= 0)
      {
        ROS_INFO("No clusters, already travelled to %d frontiers. Terminating.", totalFrontierPlans);
        structureCovered = 1;
      }

    } 
    else if(structureCovered == 0) { // sending usable point cloud to Naik's code
      ROS_INFO("Sending usable graph to gtsp solver");
      gtsp::GTSPData gtsp_data;
      pcl::PCLPointCloud2 pcl_pc;
      pcl::toPCLPointCloud2(*clusteredPoints, pcl_pc);
      sensor_msgs::PointCloud2 usablePC2;
      pcl_conversions::fromPCL(pcl_pc, usablePC2);

      gtsp_data.points = usablePC2;
      gtsp_data.numClusters = numOfCluster;
      gtsp_data.pointClusterMapping = point2ClusterMapping;
      std::cout<<"Number of clusters: "<<numOfCluster<<"\nClustered points size: "<<clusteredPoints->size()<<"\nCluster mapping size: "<<point2ClusterMapping.size()<<std::endl;

      gtspData_pub.publish(gtsp_data);
      while(!tour_ready){
        ros::spinOnce();
        poll_rate.sleep();
      }
      tour_ready = false;
      ROS_INFO("Tour ready");
    }
    compT_endGTSP = ros::Time::now();

// publishing voxel arrays
    occArrayTrimmed_pub.publish(occArrayTrimmed);
    freeArrayTrimmed_pub.publish(freeArrayTrimmed);
    occArrayFull_pub.publish(occArrayFull);
    freeArrayFull_pub.publish(freeArrayFull);
 //   inspectedVoxels_pub.publish(inspectedVoxels);

// executes tour based on GTSP output
    if(structureCovered == 1){ // going back to origin because no new viewpoints
      ROS_INFO("Code done");
      //occArrayTrimmed_pub.publish(occArrayTrimmed);
      //freeArrayTrimmed_pub.publish(freeArrayTrimmed);
      //occArrayFull_pub.publish(occArrayFull);
      //freeArrayFull_pub.publish(freeArrayFull);
      break;
    } else{ //executing tour
      ros::Time startTime = ros::Time::now();
      ros::Duration elapsed = ros::Time::now()-startTime;
      pcl::PointCloud<pcl::PointXYZ>::Ptr tourPoints (new pcl::PointCloud<pcl::PointXYZ>);
      for(int iterator = 0; iterator < tour.size(); iterator++)
      {
        int pointIndex = tour[iterator];
        pointIndex = pointIndex - 1;
        tourPoints->push_back(clusteredPoints->points[pointIndex]);
      }

      pcl::KdTree<pcl::PointXYZ>::Ptr gtspTree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      gtspTree->setInputCloud(tourPoints);
      std::vector<int> nn_indices (1);
      std::vector<float> nn_dists (1);
      gtspTree->nearestKSearch(pcl::PointXYZ(currentPose.position.x, currentPose.position.y, currentPose.position.z), 1, nn_indices, nn_dists);
      std::cout<<std::endl;

      int currentPoint = -1;
      for(int iterator=0;iterator<clusteredPoints->size();iterator++){
        if(double_equals(clusteredPoints->at(iterator).x, tourPoints->at(nn_indices[0]).x) &&
           double_equals(clusteredPoints->at(iterator).y, tourPoints->at(nn_indices[0]).y) &&
           double_equals(clusteredPoints->at(iterator).z, tourPoints->at(nn_indices[0]).z))
        {
          currentPoint = iterator;
          break;
        }

      }
      currentPoint = currentPoint + 1;

      int currentPointNumber = -1;
      for(int j=0;j<tour.size();j++){
        if(tour[j] == currentPoint)
        {
          currentPointNumber = j;
          break;
        }
      }
      /*
      int currentPoint = 0;
      while(true){
        if(std::count(tour.begin(),tour.end(),nn_indices[currentPoint])){
          break;
        } else{
          currentPoint++;
        }
      }
      */
      int countMoveit = 0;
      tspDone = false;
      /*
      int currentPointNumber = -1;
      for(int j=0;j<tour.size();j++){
        if(tour.at(j)==nn_indices[currentPoint]){
          currentPointNumber = j;
          break;
        }
      }
      //currentPointNumber = 0;
      */
      ROS_INFO("\nStart index in tour: %d\nStart point in tour: %d",currentPointNumber,tour[currentPointNumber]);
      while(elapsed.sec<replanningTime && !tspDone)
      {
        updateT = ros::Time::now();
        myfileT << updateT-beginT << "," << runningVisitedVoxels->size() << "," << tempCloudOccTrimmed->size()-runningVisitedVoxels->size()<<","<< countOccTrimmed << "," << countOccFull << ","<< countFreeFull << "," << countUnknownFull << std::endl;
        myFileFlightDistance << updateT-beginT << "," << flight_Distance << std::endl;

      //while(elapsed.sec<replanningTime && !tspDone){
        ROS_INFO("UAV flight");
        resetFlag_msg.data=0;
        resetFlag_pub.publish(resetFlag_msg);
        
        float tempDistance = moveit_distance;
        
        length_ready = false;
        std::cout<<"Current index in tour: "<<currentPointNumber<<std::endl;
        std::cout<<"Current point in tour: "<<tour[currentPointNumber]<<std::endl;
        tf2::Quaternion UAVOrientation;
        ROS_INFO("Inspecting point: [%f, %f, %f]", inspectingPoints->points[tour[currentPointNumber]-1].x, inspectingPoints->points[tour[currentPointNumber]-1].y,inspectingPoints->points[tour[currentPointNumber]-1].z);
        double inspection_x = inspectingPoints->points[tour[currentPointNumber]-1].x;
        double inspection_y = inspectingPoints->points[tour[currentPointNumber]-1].y;

        double goal_x = clusteredPoints->points[tour[currentPointNumber]-1].x;
        double goal_y = clusteredPoints->points[tour[currentPointNumber]-1].y;
        double goal_z = clusteredPoints->points[tour[currentPointNumber]-1].z;

        double direction_y = inspection_y - goal_y;
        double direction_x = inspection_x - goal_x;
        double angle = atan2(direction_y, direction_x);

        //double tempYaw = orientationOfCPs[tour[currentPointNumber]-1];
        ROS_INFO("Goal yaw (degrees): %f", angle);
        //tempYaw = tempYaw * M_PI / 180;
        //UAVOrientation.setRPY(0,0,tempYaw);

        inspectionPath << goal_x << "," << goal_y << "," << goal_z << "," << angle << "\n";

        int clusterNumber = point2ClusterMapping.at(tour[currentPointNumber]-1);
        pcl::PointXYZ tempPoint(viewPoints->at(clusterNumber-1).x,viewPoints->at(clusterNumber-1).y,viewPoints->at(clusterNumber-1).z);
        runningVisitedVoxels->push_back(tempPoint);
        std::cout<<"moveit_distance: "<<moveit_distance<<std::endl;
        std::cout<<"tempDistance: " <<tempDistance<<std::endl;
        updateT = ros::Time::now();
        myfileDA << updateT-beginT << "," << moveit_distance << std::endl;
       // myfileDE << updateT-beginT << "," << sqrt(pow(goal.goal_pose.position.x-currentPose.position.x,2)+pow(goal.goal_pose.position.y-currentPose.position.y,2)+pow(goal.goal_pose.position.z-currentPose.position.z,2))<< std::endl;

        ROS_INFO("test1");
        if(currentPointNumber >= tour.size()-1){ // check if the current tour point is the last one and loop
          currentPointNumber = 0;
        } else{
          currentPointNumber++;
        }
        ROS_INFO("test2");
        if(tour.size()-1 == countMoveit){ // check if all tour points have been visited
          tspDone = true;
        } else{
          countMoveit++;
          elapsed = ros::Time::now()-startTime;
        }
        ROS_INFO("test3");
        ros::spinOnce();
        float realDistance = 0;
        bool found;
        for(int i=0;i<visitedPointsList->size();i++){
          for(int j=0;j<clusteredPoints->size();j++){
            if(checkIfDroneInVoxel(visitedPointsList->at(i), clusteredPoints->at(j), orientationOfCPs[j]))
            {
            //if(checkIfPointIsInVoxel(visitedPointsList->at(i), clusteredPoints->at(j), tempRes.at(j))){
              pcl::PointXYZ tempPoint = viewPoints->at(point2ClusterMapping.at(j)-1);
              found = false;
              for(int k=0;k<runningVisitedVoxels->size();k++){
                if(double_equals(runningVisitedVoxels->at(k).x,tempPoint.x) &&
                   double_equals(runningVisitedVoxels->at(k).y,tempPoint.y) && 
                   double_equals(runningVisitedVoxels->at(k).z,tempPoint.z)){
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
        ROS_INFO("test4");
        compT_endFlight = ros::Time::now();
        myfileCompTime<<loopNumber<<","<<compT_endAlgo-compT_begin<<","<<compT_endGTSP-compT_endAlgo<<","<<compT_endFlight-compT_endGTSP<<std::endl;
        ROS_INFO("test7");
        for(int i=0;i<visitedPointsList->size()-1;i++){
          ROS_INFO("test9");
          realDistance += sqrt(pow(visitedPointsList->at(i+1).x-visitedPointsList->at(i).x,2)+pow(visitedPointsList->at(i+1).y-visitedPointsList->at(i).y,2)+pow(visitedPointsList->at(i+1).z-visitedPointsList->at(i).z,2));
          ROS_INFO("test10");
        }
        ROS_INFO("test8");
        myfileDR<<updateT-beginT<<","<<realDistance<<std::endl;
        resetFlag_msg.data=1;
        resetFlag_pub.publish(resetFlag_msg);
        ROS_INFO("test6");
      }
      ROS_INFO("test5");
    }

    std::cout<<std::endl;
    ROS_INFO("Clearing all voxel maps stored");
    occArrayTrimmed.markers.clear();
    inspectedVoxels.markers.clear();
    freeArrayTrimmed.markers.clear();
    occArrayFull.markers.clear();
    freeArrayFull.markers.clear();
    impossibleArray.markers.clear();
    usableArray.markers.clear();

    std::cout<<"=============loop number: "<<loopNumber+1<<"========================"<<std::endl; // outputting loop number
    loopNumber++;
    r.sleep();
  }

  ros::Duration elapsedTotalTime = ros::Time::now()-totalRunTime;
  ROS_INFO("Total run time: %d seconds",elapsedTotalTime.sec);
  myfile.close();
  myfile1.close();
  myfileT.close();
  myfileDA.close();
  myfileDE.close();
  myfileCompTime.close();
  myfileDR.close();
  myfileTestClusters.close();
  myFileInspectableVoxels.close();
  myfileClusters.close();
  inspectionPath.close();
  return 0;
}
