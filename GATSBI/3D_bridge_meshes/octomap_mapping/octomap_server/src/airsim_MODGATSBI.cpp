#define _USE_MATH_DEFINES
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
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
#include <airsim_moveit_navigation/AirSim_NavigationAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
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
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include<unistd.h> 

#define MAX_TOUR_SIZE 25

// random comment here

// global variables
float thresholdDis = 0.1;
float viewingConeAngleThreshold = 10;
float viewingConeAngleBuffer = 0;
float minRadius = 5;
float maxRadius = 10;
int neighbor_viewpoints_check = 2;
int opp_neighbor_viewpoints_check = -1 * neighbor_viewpoints_check;

int neighbor_check = 1;
int opp_neighbor_check = -1 * neighbor_check;
float resolution = 1; // need to change this in the launch file to have an affect
octomap::OcTree* fullOcTree = NULL;
octomap::OcTree* trimmedOcTree = NULL;
int countCB = 0;
int fileSave = 0;
float markerSize = resolution;
float xData=0.0;
float yData=0.0;
float zData=0.0;
int positionCount = 0;
std::vector<int> tour;
double moveit_distance = -1;
bool tour_ready = false;
bool length_ready = false;
bool visited_points_ready = false;
bool visited_poses_ready = false;
bool trimmedOcReady = false;
bool fullOcReady = false;
bool rotate_for_multiple_poses_at_view_point = true;
bool top_down_enabled = false;
int message_wait;
float free_voxel_prob = 0.12;

geometry_msgs::Pose currentPose;
pcl::PointCloud<pcl::PointXYZ>::Ptr runningVisitedVoxels (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr moveitFailed (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<double> tempzFilteredSize;
std_msgs::Bool resetFlag_msg;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr visitedPointsList (new pcl::PointCloud<pcl::PointXYZRGBA>);
geometry_msgs::PoseArray visitedPoseList;

tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped base_link_to_world_ned;
std::mutex mutex_;
std::mutex mutex_trimmed;

float checkDistance = 25; // the allowed difference in the expected distance and actual distance
int replanningTime = 60; // replanning time

double flight_Distance = 0;
double previousX = 0;
double previousY = 0;
double previousZ = 0;

float xMin = -70;
float xMax = 70;
float yMin = -70;
float yMax = 70;
float zMin = 0.2;
float zMax = 25;

float sensor_range = 30;

bool checkIfDroneInVoxel(pcl::PointXYZRGBA POI, pcl::PointXYZ voxelPoint, float orientation){

  if((voxelPoint.x - 0.5) <= POI.x && POI.x <= (voxelPoint.x + 0.5) &&
     (voxelPoint.y - 0.5) <= POI.y && POI.y <= (voxelPoint.y + 0.5) &&
     (voxelPoint.z - 0.5) <= POI.z && POI.z <= (voxelPoint.z + 0.5))
  {
    return true;
  }
  return false;
  /*
  if(orientation > 350 && orientation < 10)
  {
    if(std::fabs(POI.x - voxelPoint.x) < 1 && std::fabs(POI.z - voxelPoint.z) < 1 && voxelPoint.y - POI.y < 10)
    {
      return true;
    }
  }else if(orientation > 80 && orientation < 100)
  {
    if(std::fabs(POI.y - voxelPoint.y) < 1 && std::fabs(POI.z - voxelPoint.z) < 1 && voxelPoint.x - POI.x < 10)
    {
      return true;
    }
  }else if(orientation > 170 && orientation < 190)
  {
    if(std::fabs(POI.x - voxelPoint.x) < 1 && std::fabs(POI.z - voxelPoint.z) < 1 && POI.y - voxelPoint.y < 10)
    {
      return true;
    }
  }else if(orientation > 260 && orientation < 280)
  {
    if(std::fabs(POI.y - voxelPoint.y) < 1 && std::fabs(POI.z - voxelPoint.z) < 1 && POI.x - voxelPoint.x < 10)
    {
      return true;
    }
  }
  return false;
  */
}


float dot(tf::Vector3 a, tf::Vector3 b)
{
  return ((a.x() * b.x()) + (a.y() * b.y()) + (a.z() * b.z()));
}


float mag(tf::Vector3 a)  //calculates magnitude of a
{
    return std::sqrt((a.x() * a.x()) + (a.y() * a.y()) + (a.z() * a.z()));
}

geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion_tf2;
  quaternion_tf2.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  return quaternion;
}

void countCells(int& occupiedCount, int& freeCount, int& unknownCount) {
    occupiedCount = 0;
    freeCount = 0;
    unknownCount = 0;

    std::unique_lock<std::mutex> lock(mutex_);
    for (octomap::OcTree::iterator it = trimmedOcTree->begin(), end = trimmedOcTree->end(); it != end; ++it) {
        if (trimmedOcTree->isNodeOccupied(*it)) {
            ++occupiedCount;
        } else {
            ++freeCount;
        }
    }
    ROS_INFO("Occupied Cells: [%d]", occupiedCount);
    ROS_INFO("Free Cells: [%d]", freeCount);
    int total_voxels = (int)(pow((sensor_range + sensor_range), 3) / resolution);
    unknownCount = total_voxels - (occupiedCount + freeCount);
    ROS_INFO("Unknown Cells: [%d]", unknownCount);
}

bool checkIfNeighborsFree(float candidate_x, float candidate_y, float candidate_z)
{
  octomap::OcTreeNode* result;
  for(int i = opp_neighbor_check; i <= neighbor_check; i++)
  {
    for(int j = opp_neighbor_check; j <= neighbor_check; j++)
    {
      for(int k = opp_neighbor_check; k <= neighbor_check; k++)
      {
        if(i == 0 && j == 0 && k == 0)
        {
          continue;
        }
        octomap::OcTreeKey neighbor_key = fullOcTree->coordToKey(candidate_x, candidate_y, candidate_z);
        neighbor_key[0] += i;
        neighbor_key[1] += j;
        neighbor_key[2] += k;
        result = fullOcTree->search(neighbor_key);
        if(!result)
        {
          return false;
        }
        else if(fullOcTree->isNodeOccupied(result))
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool checkIfViewUnobstructed(octomap::point3d origin, octomap::point3d direction, octomap::point3d POI, float maxRadius)
{
  octomap::point3d end_point(0, 0, -5000);
  bool rayOutput = fullOcTree->castRay(origin, direction, end_point, false, (maxRadius + resolution));
  if(rayOutput && (end_point == POI))
    return true;

  return false;
}

bool checkIfViewUnobstructed_ComputeRay(octomap::point3d origin, octomap::point3d end)
{
  octomap::KeyRay ray;
  if(fullOcTree->computeRayKeys(origin, end, ray))
  {
    octomap::OcTreeNode* result;
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      result = fullOcTree->search(*it);
      if(result)
      {
        if(fullOcTree->isNodeOccupied(result))
        {
          return false;
        }
      }
      else
      {
        return false;
      }
    }
    return true;
  }
  return false;
}

bool checkIfCandidateViewPointFree(octomap::OcTreeKey key, float POIX, float POIY, float POIZ, float minRadius, float maxRadius)
{
  octomap::point3d point = fullOcTree->keyToCoord(key);
  for(int k=0;k<moveitFailed->size();k++){
    if(double_equals(moveitFailed->at(k).x, point.x()) &&
       double_equals(moveitFailed->at(k).y, point.y()) && 
       double_equals(moveitFailed->at(k).z, point.z())){
      return false;
    }
  }

  octomap::OcTreeNode* result = fullOcTree->search(key);
  if(result)
  {
      if(result->getOccupancy() > free_voxel_prob)
      {
        return false;
      }
      
      
      double candidate_distance = sqrt(pow(point.x() - POIX,2) + 
                                        pow(point.y() - POIY,2) + 
                                        pow(point.z() - POIZ,2));    
      if(candidate_distance >= minRadius)
      {
        octomap::point3d direction((POIX - point.x()), (POIY - point.y()), (POIZ - point.z()));
        bool allNeighborsFree = checkIfNeighborsFree(point.x(), point.y(), point.z());
        octomap::point3d POI(POIX, POIY, POIZ);
        bool viewUnobstructed = checkIfViewUnobstructed(point, direction, POI, maxRadius);
        //bool viewUnobstructed = checkIfViewUnobstructed_ComputeRay(point, POI);
        if(allNeighborsFree && viewUnobstructed)
        {
          return true;
        }
      }
  }

  return false;
}

bool checkPOINeighbors(float POIX,float POIY,float POIZ, int direction)
{
  octomap::OcTreeKey key = fullOcTree->coordToKey(POIX, POIY, POIZ);
  if(direction == 0)
  {
    key[0] += 1;
    octomap::OcTreeNode* result = fullOcTree->search(key);
    if(result)
    {
      if(result->getOccupancy() <= free_voxel_prob)
      {
        return true;
      }
    }
  }
  else if(direction == 1)
  {
    key[0] -= 1;
    octomap::OcTreeNode* result = fullOcTree->search(key);
    if(result)
    {
      if(result->getOccupancy() <= free_voxel_prob)
      {
        return true;
      }
    }
  }
  else if(direction == 2)
  {
    key[1] += 1;
    octomap::OcTreeNode* result = fullOcTree->search(key);
    if(result)
    {
      if(result->getOccupancy() <= free_voxel_prob)
      {
        return true;
      }
    }
  }
  else if(direction == 3)
  {
    key[1] -= 1;
    octomap::OcTreeNode* result = fullOcTree->search(key);
    if(result)
    {
      if(result->getOccupancy() <= free_voxel_prob)
      {
        return true;
      }
    }
  }
  else if(direction == 4)
  {
    key[2] += 1;
    octomap::OcTreeNode* result = fullOcTree->search(key);
    if(result)
    {
      if(result->getOccupancy() <= free_voxel_prob)
      {
        return true;
      }
    }
  }
  return false;
}

void findViewPointsCastRay(float POIX,float POIY,float POIZ,float minRadius,float maxRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr outputPoints, std::vector<float>* outputOrientationYaw)
{
  /*
  ROS_INFO("Finding candidate view points for bridge voxel [%.2f, %.2f, %.2f]", POIX, POIY, POIZ);
  octomap::point3d plus_x_origin(POIX + 1, POIY, POIZ);
  octomap::point3d plus_x_direction(1, 0, 0);

  octomap::point3d minus_x_origin(POIX - 1, POIY, POIZ);
  octomap::point3d minus_x_direction(-1, 0, 0);

  octomap::point3d plus_y_origin(POIX, POIY + 1, POIZ);
  octomap::point3d plus_y_direction(0, 1, 0);

  octomap::point3d minus_y_origin(POIX, POIY - 1, POIZ);
  octomap::point3d minus_y_direction(0, -1, 0);

  octomap::point3d end_point(0, 0, 0);

  bool RayOutput;

  std::unique_lock<std::mutex> lock(mutex_);

  RayOutput = fullOcTree->castRay(plus_x_origin, plus_x_direction, end_point, false, maxRadius);
  ROS_INFO("Ray Casting for Positive X: %s", RayOutput ? "true" : "false");
  ROS_INFO("End Point [%.2f, %.2f, %.2f]", end_point.x(), end_point.y(), end_point.z());

  RayOutput = fullOcTree->castRay(minus_x_origin, minus_x_direction, end_point, false, maxRadius);
  ROS_INFO("Ray Casting for Negative X: %s", RayOutput ? "true" : "false");
  ROS_INFO("End Point [%.2f, %.2f, %.2f]", end_point.x(), end_point.y(), end_point.z());

  RayOutput = fullOcTree->castRay(plus_y_origin, plus_y_direction, end_point, false, maxRadius);
  ROS_INFO("Ray Casting for Positive Y: %s", RayOutput ? "true" : "false");
  ROS_INFO("End Point [%.2f, %.2f, %.2f]", end_point.x(), end_point.y(), end_point.z());

  RayOutput = fullOcTree->castRay(minus_y_origin, minus_y_direction, end_point, false, maxRadius);
  ROS_INFO("Ray Casting for Negative Y: %s", RayOutput ? "true" : "false");
  ROS_INFO("End Point [%.2f, %.2f, %.2f]", end_point.x(), end_point.y(), end_point.z());
  */
  ROS_INFO("Finding candidate view points for bridge voxel [%.2f, %.2f, %.2f]", POIX, POIY, POIZ);
  octomap::point3d end_point(POIX, POIY, POIZ);
  
  octomap::KeyRay ray;
  std::unique_lock<std::mutex> lock(mutex_);
  //if(checkPOINeighbors(POIX, POIY, POIZ, 0))
  //{
  octomap::point3d start_pos_x(POIX + maxRadius, POIY, POIZ);
  if(fullOcTree->computeRayKeys(start_pos_x, end_point, ray))
  {
    octomap::OcTreeNode* result;
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      octomap::point3d base_point = fullOcTree->keyToCoord(*it);
      for(int i = opp_neighbor_viewpoints_check; i <= neighbor_viewpoints_check; i++)
      {
        for(int j = opp_neighbor_viewpoints_check; j <= neighbor_viewpoints_check; j++)
        {
          octomap::OcTreeKey neighbor_key = *it;
          neighbor_key[1] += i;
          neighbor_key[2] += j;
          bool candidate_viewpoint = checkIfCandidateViewPointFree(neighbor_key, POIX, POIY, POIZ, minRadius, maxRadius);
          if(candidate_viewpoint)
          {
            octomap::point3d point = fullOcTree->keyToCoord(neighbor_key);
            tf::Vector3 D1(point.x() - POIX, point.y() - POIY, point.z() - POIZ);
            tf::Vector3 D2(base_point.x() - POIX, base_point.y() - POIY, base_point.z() - POIZ);
            float angle = std::acos(dot(D1,D2)/(mag(D1)*mag(D2)));
            angle = angle * 180 / M_PI;
            angle = std::abs(angle);
            //ROS_INFO("Angle between drone orientation and bridge: [%.2f]", angle);
            if(angle <= viewingConeAngleThreshold)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
          }
        }
      }      
    }
  }
  //}

  //if(checkPOINeighbors(POIX, POIY, POIZ, 1))
  //{
  octomap::point3d start_neg_x(POIX - maxRadius, POIY, POIZ);
  ray.reset();
  if(fullOcTree->computeRayKeys(start_neg_x, end_point, ray))
  {
    
    octomap::OcTreeNode* result;
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      octomap::point3d base_point = fullOcTree->keyToCoord(*it);
      for(int i = opp_neighbor_viewpoints_check; i <= neighbor_viewpoints_check; i++)
      {
        for(int j = opp_neighbor_viewpoints_check; j <= neighbor_viewpoints_check; j++)
        {
          octomap::OcTreeKey neighbor_key = *it;
          neighbor_key[1] += i;
          neighbor_key[2] += j;
          bool candidate_viewpoint = checkIfCandidateViewPointFree(neighbor_key, POIX, POIY, POIZ, minRadius, maxRadius);
          if(candidate_viewpoint)
          {
            octomap::point3d point = fullOcTree->keyToCoord(neighbor_key);
            tf::Vector3 D1(point.x() - POIX, point.y() - POIY, point.z() - POIZ);
            tf::Vector3 D2(base_point.x() - POIX, base_point.y() - POIY, base_point.z() - POIZ);
            float angle = std::acos(dot(D1,D2)/(mag(D1)*mag(D2)));
            angle = angle * 180 / M_PI;
            angle = std::abs(angle);
            //ROS_INFO("Angle between drone orientation and bridge: [%.2f]", angle);
            if(angle <= viewingConeAngleThreshold)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
          }
        }
      }      
    }
  }
  //}

  //if(checkPOINeighbors(POIX, POIY, POIZ, 2))
  //{
  octomap::point3d start_pos_y(POIX, POIY + maxRadius, POIZ);
  ray.reset();
  if(fullOcTree->computeRayKeys(start_pos_y, end_point, ray))
  {
    
    octomap::OcTreeNode* result;
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      octomap::point3d base_point = fullOcTree->keyToCoord(*it);
      for(int i = opp_neighbor_viewpoints_check; i <= neighbor_viewpoints_check; i++)
      {
        for(int j = opp_neighbor_viewpoints_check; j <= neighbor_viewpoints_check; j++)
        {
          octomap::OcTreeKey neighbor_key = *it;
          neighbor_key[0] += i;
          neighbor_key[2] += j;
          bool candidate_viewpoint = checkIfCandidateViewPointFree(neighbor_key, POIX, POIY, POIZ, minRadius, maxRadius);
          if(candidate_viewpoint)
          {
            octomap::point3d point = fullOcTree->keyToCoord(neighbor_key);
            tf::Vector3 D1(point.x() - POIX, point.y() - POIY, point.z() - POIZ);
            tf::Vector3 D2(base_point.x() - POIX, base_point.y() - POIY, base_point.z() - POIZ);
            float angle = std::acos(dot(D1,D2)/(mag(D1)*mag(D2)));
            angle = angle * 180 / M_PI;
            angle = std::abs(angle);
            //ROS_INFO("Angle between drone orientation and bridge: [%.2f]", angle);
            if(angle <= viewingConeAngleThreshold)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
          }
        }
      }      
    }
  }
  //}

  //if(checkPOINeighbors(POIX, POIY, POIZ, 3))
  //{
  octomap::point3d start_neg_y(POIX, POIY - maxRadius, POIZ);
  ray.reset();
  if(fullOcTree->computeRayKeys(start_neg_y, end_point, ray))
  {
    
    octomap::OcTreeNode* result;
    for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
    {
      octomap::point3d base_point = fullOcTree->keyToCoord(*it);
      for(int i = opp_neighbor_viewpoints_check; i <= neighbor_viewpoints_check; i++)
      {
        for(int j = opp_neighbor_viewpoints_check; j <= neighbor_viewpoints_check; j++)
        {
          octomap::OcTreeKey neighbor_key = *it;
          neighbor_key[0] += i;
          neighbor_key[2] += j;
          bool candidate_viewpoint = checkIfCandidateViewPointFree(neighbor_key, POIX, POIY, POIZ, minRadius, maxRadius);
          if(candidate_viewpoint)
          {
            octomap::point3d point = fullOcTree->keyToCoord(neighbor_key);
            tf::Vector3 D1(point.x() - POIX, point.y() - POIY, point.z() - POIZ);
            tf::Vector3 D2(base_point.x() - POIX, base_point.y() - POIY, base_point.z() - POIZ);
            float angle = std::acos(dot(D1,D2)/(mag(D1)*mag(D2)));
            angle = angle * 180 / M_PI;
            angle = std::abs(angle);
            //ROS_INFO("Angle between drone orientation and bridge: [%.2f]", angle);
            if(angle <= viewingConeAngleThreshold)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
          }
        }
      }      
    }
  }
  //}
  
  //if(top_down_enabled && checkPOINeighbors(POIX, POIY, POIZ, 4))
  if(top_down_enabled)
  {
    octomap::point3d start_pos_z(POIX, POIY, POIZ + maxRadius);
    ray.reset();
    if(fullOcTree->computeRayKeys(start_pos_z, end_point, ray))
    {
      
      octomap::OcTreeNode* result;
      for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
      {
        octomap::point3d base_point = fullOcTree->keyToCoord(*it);
        for(int i = opp_neighbor_viewpoints_check; i <= neighbor_viewpoints_check; i++)
        {
          for(int j = opp_neighbor_viewpoints_check; j <= neighbor_viewpoints_check; j++)
          {
            octomap::OcTreeKey neighbor_key = *it;
            neighbor_key[0] += i;
            neighbor_key[1] += j;
            
            bool candidate_viewpoint = checkIfCandidateViewPointFree(neighbor_key, POIX, POIY, POIZ, minRadius, maxRadius);
            if(candidate_viewpoint)
            {
              octomap::point3d point = fullOcTree->keyToCoord(neighbor_key);
              tf::Vector3 D1(point.x() - POIX, point.y() - POIY, point.z() - POIZ);
              tf::Vector3 D2(base_point.x() - POIX, base_point.y() - POIY, base_point.z() - POIZ);
              float angle = std::acos(dot(D1,D2)/(mag(D1)*mag(D2)));
              angle = angle * 180 / M_PI;
              angle = std::abs(angle);
              //ROS_INFO("Angle between drone orientation and bridge: [%.2f]", angle);
              if(angle <= viewingConeAngleThreshold)
              {
                outputPoints->push_back({point.x(), point.y(), point.z()});
                outputOrientationYaw->push_back(270);
              }
            }
          }
        }      
      }
    }
  }
  ROS_INFO("Candidate Viewpoints for this POI: [%lu]", outputPoints->size());
}

void findViewPoints(float POIX,float POIY,float POIZ,float minRadius,float maxRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr outputPoints, std::vector<float>* outputOrientationYaw)
{
  ROS_INFO("Finding candidate view points for bridge voxel [%.2f, %.2f, %.2f]", POIX, POIY, POIZ);
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
          if(result->getOccupancy() > free_voxel_prob)
          {
            break;
          }
          
          bool allNeighborsFree = true;
          if(point.x() >= POIX + minRadius)
          {
            for(int i = opp_neighbor_check; i <= neighbor_check; i++)
            {
              if(!allNeighborsFree)
                break;
              for(int j = opp_neighbor_check; j <= neighbor_check; j++)
              {
                if(!allNeighborsFree)
                  break;
                for(int k = opp_neighbor_check; k <= neighbor_check; k++)
                {
                  if(!allNeighborsFree)
                    break;
                  if(i == 0 && j == 0 && k == 0)
                  {
                    continue;
                  }
                  octomap::OcTreeKey neighbor_key = *it;
                  neighbor_key[0] += i;
                  neighbor_key[1] += j;
                  neighbor_key[2] += k;
                  result = fullOcTree->search(*it);
                  if(result && result->getOccupancy() > free_voxel_prob)
                  {
                    allNeighborsFree = false;
                  }
                  else if(!result)
                  {
                    allNeighborsFree = false;
                  }
                }
              }
            }
            if(allNeighborsFree)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
            //outputOrientationYaw->push_back(270);ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 270", POIX, POIY, POIZ, point.x(), point.y(), point.z());
          }
      }
      else
        break;
      /*
      else {
        if(point.x() >= POIX + minRadius)
        {
          outputPoints->push_back({point.x(), point.y(), point.z()});
          outputOrientationYaw->push_back(270);
        }
      }
      */
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
          if(result->getOccupancy() > free_voxel_prob)
          {
            break;
          }
          
          bool allNeighborsFree = true;
          if(point.x() <= POIX - minRadius)
          {
            for(int i = opp_neighbor_check; i <= neighbor_check; i++)
            {
              if(!allNeighborsFree)
                break;
              for(int j = opp_neighbor_check; j <= neighbor_check; j++)
              {
                if(!allNeighborsFree)
                  break;
                for(int k = opp_neighbor_check; k <= neighbor_check; k++)
                {
                  if(!allNeighborsFree)
                    break;
                  if(i == 0 && j == 0 && k == 0)
                  {
                    continue;
                  }
                  octomap::OcTreeKey neighbor_key = *it;
                  neighbor_key[0] += i;
                  neighbor_key[1] += j;
                  neighbor_key[2] += k;
                  result = fullOcTree->search(*it);
                  if(result && result->getOccupancy() > free_voxel_prob)
                  {
                    allNeighborsFree = false;
                  }
                  else if(!result)
                  {
                    allNeighborsFree = false;
                  }
                }
              }
            }
            if(allNeighborsFree)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
            //outputOrientationYaw->push_back(270);ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 270", POIX, POIY, POIZ, point.x(), point.y(), point.z());
          }
      }
      else
        break;

      /*
      else {
        if(point.x() <= POIX - minRadius)
        {
          outputPoints->push_back({point.x(), point.y(), point.z()});
          outputOrientationYaw->push_back(90);
        }
      }
      */
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
          if(result->getOccupancy() > free_voxel_prob)
          {
            break;
          }
          
          bool allNeighborsFree = true;
          if(point.y() >= POIY + minRadius)
          {
            for(int i = opp_neighbor_check; i <= neighbor_check; i++)
            {
              if(!allNeighborsFree)
                break;
              for(int j = opp_neighbor_check; j <= neighbor_check; j++)
              {
                if(!allNeighborsFree)
                  break;
                for(int k = opp_neighbor_check; k <= neighbor_check; k++)
                {
                  if(!allNeighborsFree)
                    break;
                  if(i == 0 && j == 0 && k == 0)
                  {
                    continue;
                  }
                  octomap::OcTreeKey neighbor_key = *it;
                  neighbor_key[0] += i;
                  neighbor_key[1] += j;
                  neighbor_key[2] += k;
                  result = fullOcTree->search(*it);
                  if(result && result->getOccupancy() > free_voxel_prob)
                  {
                    allNeighborsFree = false;
                  }
                  else if(!result)
                  {
                    allNeighborsFree = false;
                  }
                }
              }
            }
            if(allNeighborsFree)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
            //outputOrientationYaw->push_back(270);ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 270", POIX, POIY, POIZ, point.x(), point.y(), point.z());
          }
      }
      else
        break;

      /*
      else {
        if(point.y() >= POIY + minRadius)
        {
          outputPoints->push_back({point.x(), point.y(), point.z()});
          outputOrientationYaw->push_back(180);
        }
      }
      */
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
          if(result->getOccupancy() > free_voxel_prob)
          {
            break;
          }
          
          bool allNeighborsFree = true;
          if(point.y() <= POIY - minRadius)
          {
            for(int i = -1; i <= 1; i++)
            {
              if(!allNeighborsFree)
                break;
              for(int j = -1; j <= 1; j++)
              {
                if(!allNeighborsFree)
                  break;
                for(int k = -1; k <= 1; k++)
                {
                  if(!allNeighborsFree)
                    break;
                  if(i == 0 && j == 0 && k == 0)
                  {
                    continue;
                  }
                  octomap::OcTreeKey neighbor_key = *it;
                  neighbor_key[0] += i;
                  neighbor_key[1] += j;
                  neighbor_key[2] += k;
                  result = fullOcTree->search(*it);
                  if(result && result->getOccupancy() > free_voxel_prob)
                  {
                    allNeighborsFree = false;
                  }
                  else if(!result)
                  {
                    allNeighborsFree = false;
                  }
                }
              }
            }
            if(allNeighborsFree)
            {
              outputPoints->push_back({point.x(), point.y(), point.z()});
              outputOrientationYaw->push_back(270);
            }
            //outputOrientationYaw->push_back(270);ROS_INFO("Viewpoint: [%f, %f, %f]\nClusterpoint: [%f, %f, %f]\nwith orientation: 270", POIX, POIY, POIZ, point.x(), point.y(), point.z());
          }
      }
      else
        break;

      /*
      else {
        if(point.y() <= POIY - minRadius)
        {
          outputPoints->push_back({point.x(), point.y(), point.z()});
          outputOrientationYaw->push_back(0);
        }
      }
      */
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
  //ROS_INFO("Got full octomap");
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);

  std::unique_lock<std::mutex> lock(mutex_);

  fullOcTree = dynamic_cast<octomap::OcTree*>(absTree);

  fullOcReady = true;
  // countCB++;
}


void trimmed_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
  //ROS_INFO("Got trimmed octomap");
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
  std::unique_lock<std::mutex> lock_trimmed(mutex_trimmed);

  trimmedOcTree = dynamic_cast<octomap::OcTree*>(absTree);

  trimmedOcReady = true;
  // countCB++;
}



void imu_cb(const nav_msgs::Odometry::ConstPtr msg){ // imu
  //ROS_INFO("Got odometry");
  geometry_msgs::Pose odometry_information = transformPose(msg->pose.pose, "world_enu", "world_ned");
  /*
  odometry_information.orientation.x = msg->pose.pose.orientation.x;
  odometry_information.orientation.y = msg->pose.pose.orientation.y;
  odometry_information.orientation.z = msg->pose.pose.orientation.z;
  odometry_information.orientation.w = msg->pose.pose.orientation.w;
  */
  //pcl::PointXYZ tempPoint(odometry_information.position.x,odometry_information.position.y,odometry_information.position.z);
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

  /*
  tf::Quaternion q(odometry_information.orientation.x, odometry_information.orientation.y, odometry_information.orientation.z, odometry_information.orientation.w);
  tf::Vector3 p(odometry_information.position.x, odometry_information.position.y, odometry_information.position.z);
  tf::Transform myPose(q, p);
  tf::Vector3 x_axis(1, 0, 0);
  tf::Vector3 D = myPose.getBasis() * x_axis;

  int count = 0;
  for(int j=0;j<tempCloudOccTrimmed->size();j++)
  {
    double distToBridgeVoxel = sqrt(pow(previousX - tempCloudOccTrimmed->at(j).x,2) + 
                                    pow(previousY - tempCloudOccTrimmed->at(j).y,2) + 
                                    pow(previousZ - tempCloudOccTrimmed->at(j).z,2));
    if(distToBridgeVoxel > maxRadius || distToBridgeVoxel < minRadius)
      continue;

    tf::Vector3 odomToBridgeVoxelDirection(odometry_information.position.x - tempCloudOccTrimmed->at(j).x, odometry_information.position.y - tempCloudOccTrimmed->at(j).y, odometry_information.position.z - tempCloudOccTrimmed->at(j).z);
    float angle = std::acos(dot(D,odomToBridgeVoxelDirection)/(mag(D)*mag(odomToBridgeVoxelDirection)));
    if(angle <= viewingConeAngleThreshold)
    {
      count += 1;
    }
  }
  ROS_INFO("Voxels Inpsectable from Current Position: [%d]", count);
  */
}

void zFiltered_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
  //ROS_INFO("Got z filtered point cloud");
  tempCloudOccTrimmed->clear();
  BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points){
    tempCloudOccTrimmed->points.push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
  }
}

void zFilteredSize_cb(const std_msgs::Float64MultiArray& msg){
  //ROS_INFO("Got z filtered point cloud size");
  tempzFilteredSize.clear();
  for(int i = 0;i<msg.data.size();i++){
    tempzFilteredSize.push_back(msg.data.at(i));
  }
}

void visitedPointList_cb(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& msg){
  //ROS_INFO("Got visited points list");
  visitedPointsList->clear();
  BOOST_FOREACH(const pcl::PointXYZRGBA& pt, msg->points){
    visitedPointsList->points.push_back(pt);
  }
  visited_points_ready = true;
}

void visitedPoseList_cb(const geometry_msgs::PoseArray::ConstPtr& msg){
  //ROS_INFO("Got visited points list");
  visitedPoseList.poses.clear();
  BOOST_FOREACH(const geometry_msgs::Pose& pose, msg->poses){
    visitedPoseList.poses.push_back(pose);
  }
  visited_poses_ready = true;
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

  std::ofstream journalData;
  journalData.open("/home/user/bridgeInspection/journalData.csv");

  journalData << "Time,Inspectable Voxels,Inspected Voxels,Occupied Voxels,Free Voxels,Unknown Voxels,Flight Distance\n";
  myfileClusters << "Time,Inspectable Voxels,Candidate Viewpoints\n";
  myfileTestClusters << "Number," << "Bridge X," << "Bridge Y," << "Bridge Z," << "View X," << "View Y," << "View Z," << "Orientation\n";
  myFileInspectableVoxels << "Number," << "X," << "Y," << "Z," << "Inspected?," << "Notes\n";
  myFileFlightDistance << "Time,Flight Distance, Replanning Time: " << replanningTime << "\n";

// initializing ROS everything
  ros::init(argc, argv, "GATSBI");
  actionlib::SimpleActionClient<airsim_moveit_navigation::AirSim_NavigationAction> ac("airsim_navigator", true);
  ros::NodeHandle n;

  n.param<float>("/GATSBI/resolution", resolution, 1.0);
  n.param<float>("/GATSBI/viewing_cone_angle_threshold", viewingConeAngleThreshold, 10.0);
  n.param<float>("/GATSBI/viewing_cone_angle_buffer", viewingConeAngleBuffer, 0.0);
  n.param<float>("/GATSBI/min_radius", minRadius, 5.0);
  n.param<float>("/GATSBI/max_radius", maxRadius, 10.0);
  n.param<int>("/GATSBI/neighbor_viewpoints_check", neighbor_viewpoints_check, 2);
  n.param<int>("/GATSBI/neighbor_check", neighbor_check, 1);
  n.param<int>("/GATSBI/replanning_time", replanningTime, 60);
  n.param<bool>("/GATSBI/rotate_for_multiple_poses_at_view_point", rotate_for_multiple_poses_at_view_point, true);
  n.param<float>("/GATSBI/x_min", xMin, -70);
  n.param<float>("/GATSBI/x_max", xMax, 70);
  n.param<float>("/GATSBI/y_min", yMin, -70);
  n.param<float>("/GATSBI/y_max", yMax, 70);
  n.param<float>("/GATSBI/z_min", zMin, 0);
  n.param<float>("/GATSBI/z_max", zMax, 25);
  n.param<float>("/octomap_server/sensor_model/max_range", sensor_range, 30);
  n.param<bool>("/GATSBI/top_down_enabled", top_down_enabled, false);
  n.param<int>("/GATSBI/message_wait", message_wait, 5);
  n.param<float>("/GATSBI/free_voxel_prob", free_voxel_prob, 0.12);

  ROS_INFO("Resolution: %.2f", resolution);
  ROS_INFO("Viewing Cone Angle: %.2f", viewingConeAngleThreshold);
  ROS_INFO("Viewing Cone Radius: [%.2f, %.2f]", minRadius, maxRadius);
  ROS_INFO("Viewing Cone Neighbors: %d", neighbor_viewpoints_check);
  ROS_INFO("Free Space Neighbors: %d", neighbor_check);
  ROS_INFO("Replanning Time: %d", replanningTime);
  ROS_INFO("Rotate for Multiple Poses at Same View Point: %s", rotate_for_multiple_poses_at_view_point ? "True" : "False");
  ROS_INFO("World Size: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", xMin, yMin, zMin, xMax, yMax, zMax);
  ROS_INFO("Sensor Range: [%.2f]", sensor_range);
  ROS_INFO("Top Down Enabled: %s", top_down_enabled ? "True" : "False");

  ros::Rate r(0.05); // less than 1 is slower
  ros::Rate distanceSleep(10);
  ros::Rate waitSleep(10);
  
  ros::Time beginT = ros::Time::now();
  ros::Time updateT = ros::Time::now();
  ros::Time journalT = ros::Time::now();
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
  ros::Publisher candArray_pub = n.advertise<visualization_msgs::MarkerArray>("/candArray_pub",1,true);
  ros::Publisher inspectingVoxels_pub = n.advertise<visualization_msgs::MarkerArray>("/inspecting_voxels",1,true);
  ros::Publisher cur_focus_pub = n.advertise<visualization_msgs::MarkerArray>("/current_focus_voxels",1,true);

  ros::Publisher point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/gtsp_point_cloud", 1);
  ros::Publisher goal_distance_publisher = n.advertise<geometry_msgs::Point>("/compute_path/point", 1);
  ros::Publisher gtspData_pub = n.advertise<gtsp::GTSPData>("/gtsp_data", 1);
  ros::Publisher resetFlag_pub = n.advertise<std_msgs::Bool>("/resetFlag",1);
  ros::Publisher rotate_pub = n.advertise<geometry_msgs::Point>("/rotate_to_POI",1);
  ros::Publisher gatsbi_done_pub = n.advertise<std_msgs::Bool>("/gatsbi_done",1,true);
  ros::Publisher gtsp_path_pub = n.advertise<nav_msgs::Path>("/gtsp_path",1,true);

  ros::Subscriber fullTree_sub = n.subscribe("/octomap_full",1,full_cb);
  ros::Subscriber trimmedTree_sub = n.subscribe("/octomap_full_trimmed",1,trimmed_cb);
  ros::Subscriber uavIMU_sub = n.subscribe("/airsim_node/drone_1/odom_local_ned",1,imu_cb);
  ros::Subscriber tour_sub = n.subscribe("/gtsp_tour_list", 1, tourCallback);
  ros::Subscriber distance_sub = n.subscribe("/compute_path/length", 1, lengthCallback);
  ros::Subscriber zFiltered_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/zFiltered",1,zFiltered_cb);
  ros::Subscriber zFilteredSize_sub = n.subscribe("/zFilteredSize", 1, zFilteredSize_cb);
  ros::Subscriber visitedPointList_sub = n.subscribe("/visited_point_list", 1, visitedPointList_cb);
  ros::Subscriber visitedPoseList_sub = n.subscribe("/visited_poses_list", 1, visitedPoseList_cb);

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
  visualization_msgs::MarkerArray candidateViewPoints;
  visualization_msgs::MarkerArray delMarkers;
  visualization_msgs::MarkerArray inspectingVoxels;
  visualization_msgs::MarkerArray focusMarkers;

// initializing variables and markers
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int loopNumber = 0;
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
  
  bool visited_goal = false;

  int occupiedCount = 0;
  int freeCount = 0;
  int unknownCount = 0;
  int totalFrontierPlans = 0;

  std::vector<geometry_msgs::Pose> explored;

  visualization_msgs::Marker fullOccupiedMarkers; fullOccupiedMarkers.header.frame_id = "/world_enu"; fullOccupiedMarkers.header.stamp = ros::Time::now(); fullOccupiedMarkers.ns = "occupied_markers_full";
  fullOccupiedMarkers.type = shape; fullOccupiedMarkers.action = visualization_msgs::Marker::ADD;
  fullOccupiedMarkers.color.r = 1.0f; fullOccupiedMarkers.color.g = 0.0f; fullOccupiedMarkers.color.b = 0.0f; fullOccupiedMarkers.color.a = 1.0;
  visualization_msgs::Marker fullFreeMarkers; fullFreeMarkers.header.frame_id = "/world_enu"; fullFreeMarkers.header.stamp = ros::Time::now(); fullFreeMarkers.ns = "free_markers_full";
  fullFreeMarkers.type = shape; fullFreeMarkers.action = visualization_msgs::Marker::ADD;
  fullFreeMarkers.color.r = 0.0f; fullFreeMarkers.color.g = 1.0f; fullFreeMarkers.color.b = 0.0f; fullFreeMarkers.color.a = 1.0;
  visualization_msgs::Marker fullUnknownMarkers; fullUnknownMarkers.header.frame_id = "/world_enu"; fullUnknownMarkers.header.stamp = ros::Time::now(); fullUnknownMarkers.ns = "unknown_markers_full";
  fullUnknownMarkers.type = shape; fullUnknownMarkers.action = visualization_msgs::Marker::ADD;
  fullUnknownMarkers.color.r = 1.0f; fullUnknownMarkers.color.g = 1.0f; fullUnknownMarkers.color.b = 0.0f; fullUnknownMarkers.color.a = 1.0;
  visualization_msgs::Marker impossibleMarkers; impossibleMarkers.header.frame_id = "/world_enu"; impossibleMarkers.header.stamp = ros::Time::now(); impossibleMarkers.ns = "impossible_markers";
  impossibleMarkers.type = shape; impossibleMarkers.action = visualization_msgs::Marker::ADD;
  impossibleMarkers.color.r = 0.4f; impossibleMarkers.color.g = 0.0f; impossibleMarkers.color.b = 0.8f; impossibleMarkers.color.a = 1.0;
  visualization_msgs::Marker usableMarkers; usableMarkers.header.frame_id = "/world_enu"; usableMarkers.header.stamp = ros::Time::now(); usableMarkers.ns = "usable_markers";
  usableMarkers.type = shape; usableMarkers.action = visualization_msgs::Marker::ADD;
  usableMarkers.color.r = 0.0f; usableMarkers.color.g = 0.0f; usableMarkers.color.b = 1.0f; usableMarkers.color.a = 0.4;
  visualization_msgs::Marker trimmedOccupiedMarkers; trimmedOccupiedMarkers.header.frame_id = "/world_enu"; trimmedOccupiedMarkers.header.stamp = ros::Time::now(); trimmedOccupiedMarkers.ns = "occupied_markers_trimmed";
  trimmedOccupiedMarkers.type = shape; trimmedOccupiedMarkers.action = visualization_msgs::Marker::ADD;
  trimmedOccupiedMarkers.color.r = 1.0f; trimmedOccupiedMarkers.color.g = 0.0f; trimmedOccupiedMarkers.color.b = 0.0f; trimmedOccupiedMarkers.color.a = 1.0;
  visualization_msgs::Marker trimmedFreeMarkers; trimmedFreeMarkers.header.frame_id = "/world_enu"; trimmedFreeMarkers.header.stamp = ros::Time::now(); trimmedFreeMarkers.ns = "free_markers_trimmed";
  trimmedFreeMarkers.type = shape; trimmedFreeMarkers.action = visualization_msgs::Marker::ADD;
  trimmedFreeMarkers.color.r = 0.0f; trimmedFreeMarkers.color.g = 1.0f; trimmedFreeMarkers.color.b = 0.0f; trimmedFreeMarkers.color.a = 1.0;
  visualization_msgs::Marker trimmedUnknownMarkers; trimmedUnknownMarkers.header.frame_id = "/world_enu"; trimmedUnknownMarkers.header.stamp = ros::Time::now(); trimmedUnknownMarkers.ns = "unknown_markers_trimmed";
  trimmedUnknownMarkers.type = shape; trimmedUnknownMarkers.action = visualization_msgs::Marker::ADD;
  trimmedUnknownMarkers.color.r = 1.0f; trimmedUnknownMarkers.color.g = 1.0f; trimmedUnknownMarkers.color.b = 0.0f; trimmedUnknownMarkers.color.a = 1.0;

  visualization_msgs::Marker inspectedVoxelMarkers; inspectedVoxelMarkers.header.frame_id = "/world_enu"; inspectedVoxelMarkers.header.stamp = ros::Time::now(); inspectedVoxelMarkers.ns = "inspected_voxel_markers";
  inspectedVoxelMarkers.type = shape; inspectedVoxelMarkers.action = visualization_msgs::Marker::ADD;
  inspectedVoxelMarkers.color.r = 1.0f; inspectedVoxelMarkers.color.g = 0.0f; inspectedVoxelMarkers.color.b = 0.0f; inspectedVoxelMarkers.color.a = 1.0;
  

  visualization_msgs::Marker candidateViewPointMarkers; candidateViewPointMarkers.header.frame_id = "/world_enu"; candidateViewPointMarkers.header.stamp = ros::Time::now(); candidateViewPointMarkers.ns = "candidate_markers_trimmed";
  candidateViewPointMarkers.type = visualization_msgs::Marker::SPHERE; candidateViewPointMarkers.action = visualization_msgs::Marker::ADD;
  candidateViewPointMarkers.color.r = 1.0f; candidateViewPointMarkers.color.g = 1.0f; candidateViewPointMarkers.color.b = 0.0f; candidateViewPointMarkers.color.a = 1.0;

  visualization_msgs::Marker delMarker; delMarker.id = 0; delMarker.ns = "delete_markers"; delMarker.action = visualization_msgs::Marker::DELETEALL;
  delMarkers.markers.push_back(delMarker);


  visualization_msgs::Marker currentInspectingVoxels; currentInspectingVoxels.header.frame_id = "/world_enu"; currentInspectingVoxels.header.stamp = ros::Time::now(); currentInspectingVoxels.ns = "current_inspecting_voxel_markers";
  currentInspectingVoxels.type = shape; currentInspectingVoxels.action = visualization_msgs::Marker::ADD;
  currentInspectingVoxels.color.r = 1.0f; currentInspectingVoxels.color.g = 0.0f; currentInspectingVoxels.color.b = 0.0f; currentInspectingVoxels.color.a = 0.6;

  visualization_msgs::Marker focusVoxel; focusVoxel.header.frame_id = "/world_enu"; focusVoxel.header.stamp = ros::Time::now(); focusVoxel.ns = "current_focus_voxel_markers";
  focusVoxel.type = shape; focusVoxel.action = visualization_msgs::Marker::ADD;
  focusVoxel.color.r = 1.0f; focusVoxel.color.g = 1.0f; focusVoxel.color.b = 1.0f; focusVoxel.color.a = 1.0;

  ros::Time totalRunTime = ros::Time::now();

  airsim_moveit_navigation::AirSim_NavigationGoal goal;
  ROS_INFO("Waiting for action server to start");
  ac.waitForServer();
  ROS_INFO("Action server started");

  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = -1;
  goal.goal_pose.position.z = 3;
  /*
  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = -0.7071068;
  goal.goal_pose.orientation.w = 0.7071068;
  */
  goal.goal_pose.orientation.x = -5;
  goal.goal_pose.orientation.y = -1;
  goal.goal_pose.orientation.z = 3;
  ac.sendGoal(goal);
  ac.waitForResult();

  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = -1;
  goal.goal_pose.position.z = 5;
  /*
  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = -0.7071068;
  goal.goal_pose.orientation.w = 0.7071068;
  */

  goal.goal_pose.orientation.x = -5;
  goal.goal_pose.orientation.y = -1;
  goal.goal_pose.orientation.z = 5;

  ac.sendGoal(goal);
  ac.waitForResult();

  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = -1;
  goal.goal_pose.position.z = 7;
  /*
  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = -0.7071068;
  goal.goal_pose.orientation.w = 0.7071068;
  */

  goal.goal_pose.orientation.x = -5;
  goal.goal_pose.orientation.y = -1;
  goal.goal_pose.orientation.z = 7;

  ac.sendGoal(goal);
  ac.waitForResult();

  ROS_INFO("Finished all takeoff maneuvers");
  ROS_INFO("Waiting for gtsp_solver and distance_publisher to subscribe");
  ros::Rate poll_rate(100);
  while(point_cloud_publisher.getNumSubscribers() == 0)
    poll_rate.sleep();

  while(goal_distance_publisher.getNumSubscribers() == 0)
    poll_rate.sleep();

  ROS_INFO("Finished");
  
  journalT = ros::Time::now();
  journalData << journalT - beginT << "," << tempCloudOccTrimmed->size() << "," << runningVisitedVoxels->size() << "," << occupiedCount << "," << freeCount << "," << unknownCount << "," << flight_Distance << "\n";


  while (ros::ok()){ //Main while loop
    compT_begin = ros::Time::now();
    resetFlag_msg.data = true;
    resetFlag_pub.publish(resetFlag_msg);
    int message_count = 0;
    while(message_count < message_wait)
    {
      trimmedOcReady = false;
      fullOcReady = false;
      while(!fullOcReady || !trimmedOcReady)
      {
        poll_rate.sleep();
        ros::spinOnce();
      }
      message_count++;
    }
    ROS_INFO("Received updated full and trimmed octree");
    countCells(occupiedCount, freeCount, unknownCount);
    journalT = ros::Time::now();
    journalData << journalT - beginT << "," << tempCloudOccTrimmed->size() << "," << runningVisitedVoxels->size() << "," << occupiedCount << "," << freeCount << "," << unknownCount << "," << flight_Distance << "\n";

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

    ROS_INFO("Done with full octomap marker array");
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

    ROS_INFO("Done with trimmed octomap marker array");

    /*
    markerSize = resolution;
    inspectedVoxelMarkers.header.stamp = ros::Time::now();
    for(int markerIt = 0; markerIt < runningVisitedVoxels->size(); markerIt++)
    {
      inspectedVoxelMarkers.pose.position.x = runningVisitedVoxels->at(markerIt).x; inspectedVoxelMarkers.pose.position.y = runningVisitedVoxels->at(markerIt).y; inspectedVoxelMarkers.pose.position.z = runningVisitedVoxels->at(markerIt).z;
      inspectedVoxelMarkers.id = id4Markers;
      inspectedVoxelMarkers.scale.x = markerSize; inspectedVoxelMarkers.scale.y = markerSize; inspectedVoxelMarkers.scale.z = markerSize;
      inspectedVoxels.markers.push_back(inspectedVoxelMarkers);
      id4Markers = id4Markers + 1;
    }
    
    inspectedVoxels_pub.publish(delMarkers);
    ros::spinOnce();
    inspectedVoxels_pub.publish(inspectedVoxels);
    ros::spinOnce();
    
    ROS_INFO("Done with inspected voxels marker array");
    */

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

    std::ofstream inspectableFile;
    std::string inspectable_filename = "/home/user/bridgeInspection/inspectable_voxels_data_loop_" + std::to_string(loopNumber) + ".csv";
    inspectableFile.open(inspectable_filename);
    inspectableFile << "Pos_X,Pos_Y,Pos_Z\n";

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
      inspectableFile << tempCloudOccTrimmed->at(j).x << "," << tempCloudOccTrimmed->at(j).y << "," << tempCloudOccTrimmed->at(j).z << "\n";
    }

    inspectableFile.close();

    std::ofstream inspectedFile;
    std::string inspected_filename = "/home/user/bridgeInspection/inspected_voxels_data_loop_" + std::to_string(loopNumber) + ".csv";
    inspectedFile.open(inspected_filename);
    inspectedFile << "Pos_X,Pos_Y,Pos_Z\n";
    for(int j=0;j<runningVisitedVoxels->size();j++){
        myFileInspectableVoxels << j << "," << runningVisitedVoxels->at(j).x << ","
                                << runningVisitedVoxels->at(j).y << "," << runningVisitedVoxels->at(j).z
                                << ",Yes\n";

        inspectedFile << runningVisitedVoxels->at(j).x << "," << runningVisitedVoxels->at(j).y << "," << runningVisitedVoxels->at(j).z << "\n";
    }

    inspectedFile.close();
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
    id4Markers = 0;
    candidateViewPoints.markers.clear();
    for(int j = 0; j<cloudOccTrimmed->size();j++){
      tempX1 = cloudOccTrimmed->points[j].x;
      tempY1 = cloudOccTrimmed->points[j].y;
      tempZ1 = cloudOccTrimmed->points[j].z;
      tempPoints->clear();
      tempPoints1->clear();
      findViewPointsCastRay(tempX1, tempY1, tempZ1, minRadius, maxRadius, tempPoints,&tempOrientation);
      //findViewPoints(tempX1, tempY1, tempZ1, minRadius, maxRadius, tempPoints,&tempOrientation);
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
        /*
        candidateViewPointMarkers.pose.position.x = tempPoints->points[i].x; candidateViewPointMarkers.pose.position.y = tempPoints->points[i].y; candidateViewPointMarkers.pose.position.z = tempPoints->points[i].z;
        candidateViewPointMarkers.id = id4Markers;
        candidateViewPointMarkers.scale.x = 0.8, candidateViewPointMarkers.scale.y = 0.8; candidateViewPointMarkers.scale.z = 0.8;
        candidateViewPointMarkers.header.stamp = ros::Time::now();
        candidateViewPoints.markers.push_back(candidateViewPointMarkers);
        id4Markers++;
        */
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
    ROS_INFO("Total Number of Candidate View Points [%lu]", clusteredPoints->size());
    /*
    id4Markers = 0;
    candidateViewPoints.markers.clear();
    for(int j=0;j<clusteredPoints->size();j++)
    {
      //ROS_INFO("Candidate View Point [%.2f, %.2f, %.2f]", clusteredPoints->at(j).x, clusteredPoints->at(j).y, clusteredPoints->at(j).z);
      candidateViewPointMarkers.pose.position.x = clusteredPoints->at(j).x; candidateViewPointMarkers.pose.position.y = clusteredPoints->at(j).y; candidateViewPointMarkers.pose.position.z = clusteredPoints->at(j).z;
      candidateViewPointMarkers.id = id4Markers;
      candidateViewPointMarkers.scale.x = 0.8, candidateViewPointMarkers.scale.y = 0.8; candidateViewPointMarkers.scale.z = 0.8;
      candidateViewPointMarkers.header.stamp = ros::Time::now();
      candidateViewPoints.markers.push_back(candidateViewPointMarkers);
    }
    
    candArray_pub.publish(candidateViewPoints);
    freeArrayTrimmed_pub.publish(freeArrayTrimmed);
    ros::spinOnce();
    */
    //return 0;
    
    std::cout << "cloud free size: " << cloudFreeFull->size() << std::endl;
    compT_endAlgo = ros::Time::now();

    if(numOfCluster<2 && structureCovered == 0)
    { // the structure has been covered
      ROS_INFO("Number of clusters: %d", numOfCluster);
      ROS_INFO("Number of frontiers flown to: %d", totalFrontierPlans);
      if(totalFrontierPlans >= 0)
      {
        ROS_INFO("No clusters, already travelled to %d frontiers. Terminating.", totalFrontierPlans);
        structureCovered = 1;
      }

      /*
      ROS_INFO("No clusters, finding frontiers");
      double frontier_resolution = fullOcTree->getResolution();

      bool found_frontier = false;
      geometry_msgs::Pose closest_frontier;
      closest_frontier.orientation.x = 0;
      closest_frontier.orientation.y = 0;
      closest_frontier.orientation.z = 0;
      closest_frontier.orientation.w = 1;

      std::vector<std::pair<double, geometry_msgs::Pose> > candidate_frontiers;
      for(octomap::OcTree::leaf_iterator n = fullOcTree->begin_leafs(fullOcTree->getTreeDepth()); n != fullOcTree->end_leafs(); ++n)
      {
          if(!fullOcTree->isNodeOccupied(*n))
          {
              double x_cur = n.getX();
              double y_cur = n.getY();
              double z_cur = n.getZ();

              bool frontier = false;

              // Check whether voxel is within moveit environment restrictions
              if(x_cur < xMin + frontier_resolution || x_cur > xMax - frontier_resolution
              || y_cur < yMin + frontier_resolution || y_cur > yMax - frontier_resolution
              || z_cur < zMin + frontier_resolution || z_cur > zMax - frontier_resolution) continue;

              // Check whether very close point is discovered previously
              bool already_explored = false;
              for(auto a : explored){
                  if(fabs(x_cur - a.position.x) < 0.5 && fabs(y_cur - a.position.y) < 0.5 && fabs(z_cur - a.position.z) < 0.5){
                      already_explored = true;
                      break;
                  }
              }

              if(already_explored)
                  continue;

              for (double x_cur_buf = x_cur - frontier_resolution; x_cur_buf < x_cur + frontier_resolution; x_cur_buf += frontier_resolution)
              {
                  for (double y_cur_buf = y_cur - frontier_resolution; y_cur_buf < y_cur + frontier_resolution; y_cur_buf += frontier_resolution)
                  {

                      octomap::OcTreeNode *n_cur_frontier = fullOcTree->search(x_cur_buf, y_cur_buf, z_cur);
                      if(!n_cur_frontier)
                      {
                          frontier = true;
                          found_frontier = true;
                      }
                  }
              }
              if(frontier){
                  
                  // Find nearest bridge voxel to frontier
                  pcl::KdTree<pcl::PointXYZ>::Ptr bridgeTree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
                  bridgeTree->setInputCloud(tempCloudOccTrimmed);
                  std::vector<int> num_indices (1);
                  std::vector<float> num_dists (1);
                  bridgeTree->nearestKSearch(pcl::PointXYZ(x_cur, y_cur, z_cur), 1, num_indices, num_dists);
                  
                  double closest_bridge_x;
                  double closest_bridge_y;
                  double closest_bridge_z;

                  if(num_indices[0] >= 0) {
                    closest_bridge_x = tempCloudOccTrimmed->at(num_indices[0]).x;
                    closest_bridge_y = tempCloudOccTrimmed->at(num_indices[0]).y;
                    closest_bridge_z = tempCloudOccTrimmed->at(num_indices[0]).z;
                  }
                  else {
                    ROS_INFO("Shouldn't get here, unable to find closest bridge point to frontier");
                    found_frontier = false;
                    break;
                  }
                  double dist = sqrt(pow(x_cur - closest_bridge_x,2) + pow(y_cur - closest_bridge_y,2) + pow(z_cur - closest_bridge_z,2));
                  //ROS_INFO("Distance from frontier to closest bridge voxel: %f", dist);
                  closest_frontier.position.x = x_cur;
                  closest_frontier.position.y = y_cur;
                  closest_frontier.position.z = z_cur;
                  candidate_frontiers.push_back({dist,closest_frontier});
              }
          }
      }
      if(!found_frontier) {
        ROS_INFO("No frontiers found. Terminating.");
        structureCovered = 1;
      }
      else {
        double smallest_dist = 10000;
        int index = 0;
        for(int i = 0; i < candidate_frontiers.size(); i++)
        {
          if(candidate_frontiers[i].first < smallest_dist)
          {
            index = i;
            smallest_dist = candidate_frontiers[i].first;
          }
        }
        totalFrontierPlans++;
        ROS_INFO("Found frontier, attempting to navigate to it.");
        ROS_INFO("Moving to point: (%f,%f,%f)", candidate_frontiers[index].second.position.x,candidate_frontiers[index].second.position.y,candidate_frontiers[index].second.position.z);
        ROS_INFO("Sending goal");
        goal.goal_pose.position.x = candidate_frontiers[index].second.position.x;
        goal.goal_pose.position.y = candidate_frontiers[index].second.position.y;
        goal.goal_pose.position.z = candidate_frontiers[index].second.position.z;
        goal.goal_pose.orientation.x = 0;
        goal.goal_pose.orientation.y = 0;
        goal.goal_pose.orientation.z = 0;
        goal.goal_pose.orientation.w = 1;
        ac.sendGoal(goal);
        ac.waitForResult();
        explored.push_back(goal.goal_pose);
        continue;
      }
      */
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
      visited_goal = false;
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
      std_msgs::Bool gatsbi_done;
      gatsbi_done.data = true;
      goal.goal_pose.position.x = 0;
      goal.goal_pose.position.y = 0;
      goal.goal_pose.position.z = 1;
      ROS_INFO("Moving to point: (%f,%f,%f)", goal.goal_pose.position.x,goal.goal_pose.position.y,goal.goal_pose.position.z);
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      ac.waitForResult();
      gatsbi_done_pub.publish(gatsbi_done);
      ROS_INFO("Code done");
      break;
    } else{ //executing tour
      ros::Time startTime = ros::Time::now();
      ros::Duration elapsed = ros::Time::now()-startTime;
      //pcl::PointCloud<pcl::PointXYZ>::Ptr tourPoints (new pcl::PointCloud<pcl::PointXYZ>);

      double minDistance = 100000;
      int currentPointNumber = -1;
      candidateViewPoints.markers.clear();

      nav_msgs::Path gtsp_path;
      gtsp_path.header.seq = 1;
      gtsp_path.header.frame_id = "/world_enu";
      std::ofstream gtspFile;
      std::string gtsp_filename = "/home/user/bridgeInspection/gtsp_data_loop_" + std::to_string(loopNumber) + ".csv";
      gtspFile.open(gtsp_filename);
      gtspFile << "Pos_X,Pos_Y,Pos_Z,Ori_X,Ori_Y,Ori_Z,Ori_W\n";

      id4Markers = 0;
      for(int iterator = 0; iterator < tour.size(); iterator++)
      {
        int pointIndex = tour[iterator];
        pointIndex = pointIndex - 1;
        double UAVToTourPointDistance = sqrt(pow(clusteredPoints->at(pointIndex).x-currentPose.position.x,2)+pow(clusteredPoints->at(pointIndex).y-currentPose.position.y,2)+pow(clusteredPoints->at(pointIndex).z-currentPose.position.z,2));
        if(UAVToTourPointDistance < minDistance)
        {
          minDistance = UAVToTourPointDistance;
          currentPointNumber = iterator;
        }

        //tourPoints->push_back(clusteredPoints->points[pointIndex]);
        candidateViewPointMarkers.pose.position.x = clusteredPoints->at(pointIndex).x; candidateViewPointMarkers.pose.position.y = clusteredPoints->at(pointIndex).y; candidateViewPointMarkers.pose.position.z = clusteredPoints->at(pointIndex).z;
        candidateViewPointMarkers.id = id4Markers;
        candidateViewPointMarkers.scale.x = 0.5 * resolution, candidateViewPointMarkers.scale.y = 0.5 * resolution; candidateViewPointMarkers.scale.z = 0.5 * resolution;
        candidateViewPointMarkers.header.stamp = ros::Time::now();
        candidateViewPoints.markers.push_back(candidateViewPointMarkers);
        

        currentInspectingVoxels.header.stamp = ros::Time::now();
        currentInspectingVoxels.pose.position.x = inspectingPoints->points[pointIndex].x; currentInspectingVoxels.pose.position.y = inspectingPoints->points[pointIndex].y; currentInspectingVoxels.pose.position.z = inspectingPoints->points[pointIndex].z;
        currentInspectingVoxels.id = id4Markers;
        currentInspectingVoxels.scale.x = 0.8 * resolution; currentInspectingVoxels.scale.y = 0.8 * resolution; currentInspectingVoxels.scale.z = 0.8 * resolution;
        inspectingVoxels.markers.push_back(currentInspectingVoxels);

        double inspection_x = inspectingPoints->points[pointIndex].x;
        double inspection_y = inspectingPoints->points[pointIndex].y;
        double inspection_z = inspectingPoints->points[pointIndex].z;

        double goal_x = clusteredPoints->at(pointIndex).x;
        double goal_y = clusteredPoints->at(pointIndex).y;
        double goal_z = clusteredPoints->at(pointIndex).z;

        double direction_z = inspection_z - goal_z;
        double direction_y = inspection_y - goal_y;
        double direction_x = inspection_x - goal_x;

        double magnitude = sqrt((direction_x * direction_x) + (direction_y * direction_y));
        double yaw_angle = atan2(direction_y, direction_x);
        double pitch_angle = atan2(-direction_z, magnitude);

        geometry_msgs::Point gtsp_pos;
        gtsp_pos.x = goal_x;
        gtsp_pos.y = goal_y;
        gtsp_pos.z = goal_z;
        geometry_msgs::Quaternion gtsp_ori = quaternion_from_rpy(0, pitch_angle, yaw_angle);

        geometry_msgs::PoseStamped gtsp_pose;
        gtsp_pose.header.stamp = ros::Time::now();
        gtsp_pose.header.seq = id4Markers + 1;
        gtsp_pose.header.frame_id = "/world_enu";
        gtsp_pose.pose.position = gtsp_pos;
        gtsp_pose.pose.orientation = gtsp_ori;

        gtsp_path.poses.push_back(gtsp_pose);

        gtspFile << goal_x << "," << goal_y << "," << goal_z << "," << gtsp_ori.x << "," << gtsp_ori.y << "," << gtsp_ori.z << "," << gtsp_ori.w << "\n";

        id4Markers++;
      }

      gtspFile.close();
      gtsp_path.header.stamp = ros::Time::now();
      gtsp_path_pub.publish(gtsp_path);

      ROS_INFO("UAV currently at [%.2f, %.2f, %.2f]", currentPose.position.x, currentPose.position.y, currentPose.position.z);
      if(currentPointNumber != -1)
      {
        ROS_INFO("Closest GTSP Tour Point at [%.2f, %.2f, %.2f] with distance of [%.2f]", clusteredPoints->points[tour[currentPointNumber]-1].x, clusteredPoints->points[tour[currentPointNumber]-1].y, clusteredPoints->points[tour[currentPointNumber]-1].z, minDistance);
      }
      else
      {
        ROS_ERROR("Could not find close point");
        return -1;
      }
    
      candArray_pub.publish(delMarkers);
      inspectingVoxels_pub.publish(delMarkers);
      poll_rate.sleep();
      //ros::spinOnce();
      candArray_pub.publish(candidateViewPoints);
      inspectingVoxels_pub.publish(inspectingVoxels);
      poll_rate.sleep();
      //ros::spinOnce();
      //return 0;

      /*
      pcl::KdTree<pcl::PointXYZ>::Ptr gtspTree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      gtspTree->setInputCloud(tourPoints);
      std::vector<int> nn_indices (1);
      std::vector<float> nn_dists (1);
      gtspTree->nearestKSearch(pcl::PointXYZ(currentPose.position.x, currentPose.position.y, currentPose.position.z), 1, nn_indices, nn_dists);
      ROS_INFO("UAV currently at [%.2f, %.2f, %.2f]", currentPose.position.x, currentPose.position.y, currentPose.position.z);
      ROS_INFO("Closest GTSP Tour Point at [%.2f, %.2f, %.2f] with distance of [%.2f]", tourPoints->at(nn_indices[0]).x, tourPoints->at(nn_indices[0]).y, tourPoints->at(nn_indices[0]).z, nn_dists[0]);
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
      ROS_INFO("Found GTSP Tour Point in Clustered Points at Iterator [%d]", currentPoint);
      ROS_INFO("Point [%.2f, %.2f, %.2f]", clusteredPoints->at(currentPoint).x, clusteredPoints->at(currentPoint).y, clusteredPoints->at(currentPoint).z);
      currentPoint = currentPoint + 1;
      

      int currentPointNumber = -1;
      for(int j=0;j<tour.size();j++){
        if(tour[j] == currentPoint)
        {
          currentPointNumber = j;
          break;
        }
      }
      */
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
      ROS_INFO("Start index in tour: %d\nStart point in tour: %d",currentPointNumber,tour[currentPointNumber]);
      while(elapsed.sec<replanningTime && !tspDone)
      {
        updateT = ros::Time::now();
        myfileT << updateT-beginT << "," << runningVisitedVoxels->size() << "," << tempCloudOccTrimmed->size()-runningVisitedVoxels->size()<<","<< countOccTrimmed << "," << countOccFull << ","<< countFreeFull << "," << countUnknownFull << std::endl;
        myFileFlightDistance << updateT-beginT << "," << flight_Distance << std::endl;

      //while(elapsed.sec<replanningTime && !tspDone){
        ROS_INFO("UAV flight");
        resetFlag_msg.data = false;
        resetFlag_pub.publish(resetFlag_msg);
        // goal point for moveit
        goal.goal_pose.position.x = clusteredPoints->points[tour[currentPointNumber]-1].x;
        goal.goal_pose.position.y = clusteredPoints->points[tour[currentPointNumber]-1].y;
        goal.goal_pose.position.z = clusteredPoints->points[tour[currentPointNumber]-1].z;
        
        //goal_distance_publisher.publish(goal.goal_pose.position);
        //while(!length_ready){
        //  ros::spinOnce();
        //  distanceSleep.sleep();
        //}
        
        float tempDistance = moveit_distance;
        //if(moveit_distance == -1){ // if moveit couldn't find a path replan
        //  resetFlag_msg.data=1;
        //  resetFlag_pub.publish(resetFlag_msg);
        //  break;
        //}
        //if(moveit_distance-sqrt(pow(goal.goal_pose.position.x-currentPose.position.x,2)+pow(goal.goal_pose.position.y-currentPose.position.y,2)+pow(goal.goal_pose.position.z-currentPose.position.z,2))>checkDistance){ // if the distances are too different replan
        //  resetFlag_msg.data=1;
        //  resetFlag_pub.publish(resetFlag_msg);
        //  ROS_INFO("moveit distance was too different from euclidean differance.");
        //  break;
        //}
        length_ready = false;
        std::cout<<"Current index in tour: "<<currentPointNumber<<std::endl;
        std::cout<<"Current point in tour: "<<tour[currentPointNumber]<<std::endl;
        std::cout<<"Moving to point: ("<<goal.goal_pose.position.x<<","<<goal.goal_pose.position.y<<","<<goal.goal_pose.position.z<<")"<<std::endl;
        
        ROS_INFO("Inspecting point: [%f, %f, %f]", inspectingPoints->points[tour[currentPointNumber]-1].x, inspectingPoints->points[tour[currentPointNumber]-1].y,inspectingPoints->points[tour[currentPointNumber]-1].z);
        goal.goal_pose.orientation.x = inspectingPoints->points[tour[currentPointNumber]-1].x;
        goal.goal_pose.orientation.y = inspectingPoints->points[tour[currentPointNumber]-1].y;
        goal.goal_pose.orientation.z = inspectingPoints->points[tour[currentPointNumber]-1].z;
        goal.goal_pose.orientation.w = 0;
        /*
        double tempYaw = orientationOfCPs[tour[currentPointNumber]-1];
        ROS_INFO("Goal yaw (degrees): %f", tempYaw);
        tempYaw = tempYaw * M_PI / 180;
        tf2::Quaternion UAVOrientation;
        UAVOrientation.setRPY(0,0,tempYaw);
        goal.goal_pose.orientation.x = UAVOrientation.x();
        goal.goal_pose.orientation.y = UAVOrientation.y();
        goal.goal_pose.orientation.z = UAVOrientation.z();
        goal.goal_pose.orientation.w = UAVOrientation.w();
        */
        ROS_INFO("Goal orientation: (%lf,%lf,%lf,%lf)",goal.goal_pose.orientation.x,goal.goal_pose.orientation.y,goal.goal_pose.orientation.z,goal.goal_pose.orientation.w);

        focusVoxel.header.stamp = ros::Time::now();
        focusVoxel.pose.position.x = inspectingPoints->points[tour[currentPointNumber]-1].x; focusVoxel.pose.position.y = inspectingPoints->points[tour[currentPointNumber]-1].y; focusVoxel.pose.position.z = inspectingPoints->points[tour[currentPointNumber]-1].z;
        focusVoxel.id = 1;
        focusVoxel.scale.x = markerSize; focusVoxel.scale.y = markerSize; focusVoxel.scale.z = markerSize;
        focusVoxel.color.r = 1.0f; focusVoxel.color.g = 1.0f; focusVoxel.color.b = 1.0f; focusVoxel.color.a = 1.0;
        focusMarkers.markers.push_back(focusVoxel);
        focusVoxel.pose.position.x = clusteredPoints->points[tour[currentPointNumber]-1].x; focusVoxel.pose.position.y = clusteredPoints->points[tour[currentPointNumber]-1].y; focusVoxel.pose.position.z = clusteredPoints->points[tour[currentPointNumber]-1].z;
        focusVoxel.id = 2;
        focusVoxel.scale.x = markerSize; focusVoxel.scale.y = markerSize; focusVoxel.scale.z = markerSize;
        focusVoxel.color.r = 1.0f; focusVoxel.color.g = 0.0f; focusVoxel.color.b = 1.0f; focusVoxel.color.a = 1.0;
        focusMarkers.markers.push_back(focusVoxel);

        cur_focus_pub.publish(delMarkers);
        poll_rate.sleep();
        cur_focus_pub.publish(focusMarkers);
        poll_rate.sleep();

        // ROS_INFO("Sending goal");
        if(rotate_for_multiple_poses_at_view_point)
        {
          int prev_iterator = currentPointNumber - 1;
          if(currentPointNumber == 0)
          {
            prev_iterator = tour.size() - 1;
          }
          if(double_equals(goal.goal_pose.position.x,clusteredPoints->points[tour[prev_iterator]-1].x) &&
             double_equals(goal.goal_pose.position.y,clusteredPoints->points[tour[prev_iterator]-1].y) && 
             double_equals(goal.goal_pose.position.z,clusteredPoints->points[tour[prev_iterator]-1].z))
          {
            if(visited_goal)
            {
              bool found = false;
              pcl::PointXYZ tempPoint(inspectingPoints->points[tour[currentPointNumber]-1].x, inspectingPoints->points[tour[currentPointNumber]-1].y, inspectingPoints->points[tour[currentPointNumber]-1].z);
              for(int k=0;k<runningVisitedVoxels->size();k++){
                if(double_equals(runningVisitedVoxels->at(k).x, tempPoint.x) &&
                   double_equals(runningVisitedVoxels->at(k).y, tempPoint.y) && 
                   double_equals(runningVisitedVoxels->at(k).z, tempPoint.z)){
                  found = true;
                  break;
                }
              }
              if(!found)
              {
                ROS_INFO("Rotating drone");
                geometry_msgs::Point POI;
                POI.x = goal.goal_pose.orientation.x;
                POI.y = goal.goal_pose.orientation.y;
                POI.z = goal.goal_pose.orientation.z;
                //rotate_pub.publish(POI);
                //ros::Duration(2).sleep();
              }
            }
          }
          else {
            ac.sendGoal(goal);
            ac.waitForResult();
          }
        }
        else {
          int prev_iterator = currentPointNumber - 1;
          if(currentPointNumber == 0)
          {
            prev_iterator = tour.size() - 1;
          }
          if(double_equals(goal.goal_pose.position.x,clusteredPoints->points[tour[prev_iterator]-1].x) &&
             double_equals(goal.goal_pose.position.y,clusteredPoints->points[tour[prev_iterator]-1].y) && 
             double_equals(goal.goal_pose.position.z,clusteredPoints->points[tour[prev_iterator]-1].z) &&
             (visited_goal))
          {
            ROS_INFO("Skipping different orientation at same view point");
          }
          else {
            ac.sendGoal(goal);
            ac.waitForResult();
          }
        }
        actionlib::SimpleClientGoalState state = ac.getState();
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          visited_goal = true;
          pcl::PointXYZ tempPoint(goal.goal_pose.orientation.x, goal.goal_pose.orientation.y, goal.goal_pose.orientation.z);
          runningVisitedVoxels->push_back(tempPoint);
        }
        else if(state == actionlib::SimpleClientGoalState::ABORTED){
          pcl::PointXYZ tempPoint(goal.goal_pose.position.x, goal.goal_pose.position.y, goal.goal_pose.position.z);
          moveitFailed->push_back(tempPoint);
        }
        else {
          visited_goal = false;
        }
        int clusterNumber = point2ClusterMapping.at(tour[currentPointNumber]-1);
        pcl::PointXYZ tempPoint(viewPoints->at(clusterNumber-1).x,viewPoints->at(clusterNumber-1).y,viewPoints->at(clusterNumber-1).z);
        //runningVisitedVoxels->push_back(tempPoint);
        ROS_INFO("Action finished: %s", state.toString().c_str());
        std::cout<<"moveit_distance: "<<moveit_distance<<std::endl;
        std::cout<<"tempDistance: " <<tempDistance<<std::endl;
        updateT = ros::Time::now();
        myfileDA << updateT-beginT << "," << moveit_distance << std::endl;
        myfileDE << updateT-beginT << "," << sqrt(pow(goal.goal_pose.position.x-currentPose.position.x,2)+pow(goal.goal_pose.position.y-currentPose.position.y,2)+pow(goal.goal_pose.position.z-currentPose.position.z,2))<< std::endl;

        //ROS_INFO("test1");
        if(currentPointNumber >= tour.size()-1){ // check if the current tour point is the last one and loop
          currentPointNumber = 0;
        } else{
          currentPointNumber++;
        }
        //ROS_INFO("test2");
        if(tour.size()-1 == countMoveit){ // check if all tour points have been visited
          tspDone = true;
        } else{
          countMoveit++;
          elapsed = ros::Time::now()-startTime;
        }

        if(visited_goal)
        {
          //ROS_INFO("test3");
          ROS_INFO("Size of visited poses list before: [%lu]", visitedPoseList.poses.size());
          visited_poses_ready = false;
          fullOcReady = false;
          trimmedOcReady = false;
          while(!visited_poses_ready || !fullOcReady || !trimmedOcReady){
            ros::spinOnce();
            poll_rate.sleep();
          }
          ROS_INFO("Size of visited poses list after: [%lu]", visitedPoseList.poses.size());
          //ros::spinOnce();
          float realDistance = 0;
          /*
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
          */
          int previousChecked = 0;
          for(int i=0;i<visitedPoseList.poses.size();i++)
          {
            
            geometry_msgs::Pose curVisitedPose = visitedPoseList.poses[i];
            /*
            if(i > 0)
            {
              pcl::PointXYZRGBA prevVisitedPoint = visitedPointsList->at(previousChecked);
              double visDistance = sqrt(pow(curVisitedPoint.x-prevVisitedPoint.x,2)+pow(curVisitedPoint.y-prevVisitedPoint.y,2)+pow(curVisitedPoint.z-prevVisitedPoint.z,2));
              if(visDistance > thresholdDis)
              {
                continue;
              }
              previousChecked = i;
            }
            */

            
            tf::Quaternion q(curVisitedPose.orientation.x, curVisitedPose.orientation.y, curVisitedPose.orientation.z, curVisitedPose.orientation.w);
            tf::Vector3 p(curVisitedPose.position.x, curVisitedPose.position.y, curVisitedPose.position.z);
            //ROS_INFO("Start Point Vector [%.2f, %.2f, %.2f]", p.x(), p.y(), p.z());
            //ROS_INFO("Orientation Vector [%.2f, %.2f, %.2f, %f]", q.x(), q.y(), q.z(), q.w());
            tf::Transform myPose(q, p);
            tf::Vector3 x_axis(1, 0, 0);
            tf::Vector3 D = myPose.getBasis() * x_axis;

            tf::Quaternion top_down(0, 0.707, 0, 0.707);
            tf::Transform myPoseTopDown(top_down, p);
            tf::Vector3 D_top_down = myPoseTopDown.getBasis() * x_axis;

            //ROS_INFO("Orientation Direction Vector [%.2f, %.2f, %.2f]", D.x(), D.y(), D.z());
            int count = 0;
            for(int j=0;j<tempCloudOccTrimmed->size();j++)
            {
              bool found = false;
              pcl::PointXYZ tempPoint(tempCloudOccTrimmed->at(j).x, tempCloudOccTrimmed->at(j).y, tempCloudOccTrimmed->at(j).z);
              for(int k=0;k<runningVisitedVoxels->size();k++){
                if(double_equals(runningVisitedVoxels->at(k).x,tempPoint.x) &&
                   double_equals(runningVisitedVoxels->at(k).y,tempPoint.y) && 
                   double_equals(runningVisitedVoxels->at(k).z,tempPoint.z)){
                  found = true;
                  break;
                }
              }
              if(found){
                continue;
              }

              double distToBridgeVoxel = sqrt(pow(curVisitedPose.position.x - tempCloudOccTrimmed->at(j).x,2) + 
                                              pow(curVisitedPose.position.y - tempCloudOccTrimmed->at(j).y,2) + 
                                              pow(curVisitedPose.position.z - tempCloudOccTrimmed->at(j).z,2));
              //ROS_INFO("Visited Point Position [%.2f, %.2f, %.2f]", curVisitedPoint.x, curVisitedPoint.y, curVisitedPoint.z);
              //ROS_INFO("Distance: [%.2f]", distToBridgeVoxel);
              if(distToBridgeVoxel > maxRadius)
                continue;

              tf::Vector3 odomToBridgeVoxelDirection(tempCloudOccTrimmed->at(j).x - curVisitedPose.position.x, tempCloudOccTrimmed->at(j).y - curVisitedPose.position.y, tempCloudOccTrimmed->at(j).z - curVisitedPose.position.z);
              float angle = std::acos(dot(D,odomToBridgeVoxelDirection)/(mag(D)*mag(odomToBridgeVoxelDirection)));
              angle = angle * 180 / M_PI;
              angle = std::abs(angle);

              float top_down_angle = std::acos(dot(D_top_down,odomToBridgeVoxelDirection)/(mag(D_top_down)*mag(odomToBridgeVoxelDirection)));
              top_down_angle = top_down_angle * 180 / M_PI;
              top_down_angle = std::abs(top_down_angle);

              //ROS_INFO("Angle between drone orientation and bridge: [%.2f]", angle);
              if((angle <= (viewingConeAngleThreshold + viewingConeAngleBuffer)) || (top_down_enabled && (top_down_angle <= (viewingConeAngleThreshold + viewingConeAngleBuffer))))
              {
                /*
                octomap::point3d end_point(0, 0, -5000);
                octomap::point3d direction(odomToBridgeVoxelDirection.x(), odomToBridgeVoxelDirection.y(), odomToBridgeVoxelDirection.z());
                octomap::point3d origin(curVisitedPose.position.x, curVisitedPose.position.y, curVisitedPose.position.z);
                octomap::point3d POI(tempCloudOccTrimmed->at(j).x, tempCloudOccTrimmed->at(j).y, tempCloudOccTrimmed->at(j).z);

                bool rayOutput = fullOcTree->castRay(origin, direction, end_point, false, (maxRadius));
                if(rayOutput && (end_point == POI))
                */

                octomap::KeyRay ray;
                octomap::point3d origin(curVisitedPose.position.x, curVisitedPose.position.y, curVisitedPose.position.z);
                octomap::point3d end(tempCloudOccTrimmed->at(j).x, tempCloudOccTrimmed->at(j).y, tempCloudOccTrimmed->at(j).z);

                if(fullOcTree->computeRayKeys(origin, end, ray))
                {
                  bool inspectable = true;
                  octomap::OcTreeNode* result;
                  for(octomap::KeyRay::reverse_iterator it = ray.rbegin(); it != ray.rend(); ++it)
                  {
                    result = fullOcTree->search(*it);
                    if(result)
                    {
                      if(fullOcTree->isNodeOccupied(result))
                      {
                        inspectable = false;
                        break;
                      }
                    }
                    else
                    {
                      inspectable = false;
                      break;
                    }
                  }
                  if(inspectable)
                  {
                    count +=1;
                    runningVisitedVoxels->push_back(tempPoint);
                  }
                } 
                //else
                //  continue;
              }
            }
            ROS_INFO("Voxels Inspectable from Current Position: [%d]", count);
          }
        
          ROS_INFO("Check if any newly observed voxels can be inspected from current location.");
          for(int j=0;j<tempCloudOccTrimmed->size();j++)
          {
            bool found = false;
            pcl::PointXYZ tempPoint(tempCloudOccTrimmed->at(j).x, tempCloudOccTrimmed->at(j).y, tempCloudOccTrimmed->at(j).z);
            for(int k=0;k<runningVisitedVoxels->size();k++){
              if(double_equals(runningVisitedVoxels->at(k).x,tempPoint.x) &&
                 double_equals(runningVisitedVoxels->at(k).y,tempPoint.y) && 
                 double_equals(runningVisitedVoxels->at(k).z,tempPoint.z)){
                found = true;
                break;
              }
            }
            if(found){
              continue;
            }

            double distToBridgeVoxel = sqrt(pow(currentPose.position.x - tempCloudOccTrimmed->at(j).x,2) + 
                                              pow(currentPose.position.y - tempCloudOccTrimmed->at(j).y,2) + 
                                              pow(currentPose.position.z - tempCloudOccTrimmed->at(j).z,2));

            if(distToBridgeVoxel > maxRadius)
              continue;

            octomap::point3d point(currentPose.position.x, currentPose.position.y, currentPose.position.z);
            octomap::point3d direction((tempCloudOccTrimmed->at(j).x - point.x()), (tempCloudOccTrimmed->at(j).y - point.y()), (tempCloudOccTrimmed->at(j).z - point.z()));
            octomap::point3d POI(tempCloudOccTrimmed->at(j).x, tempCloudOccTrimmed->at(j).y, tempCloudOccTrimmed->at(j).z);
            bool viewUnobstructed = checkIfViewUnobstructed(point, direction, POI, maxRadius);
            if(viewUnobstructed)
            {
              ROS_INFO("Rotating drone");
              geometry_msgs::Point POI_voxel;
              POI_voxel.x = POI.x();
              POI_voxel.y = POI.y();
              POI_voxel.z = POI.z();
              //rotate_pub.publish(POI_voxel);
              focusVoxel.header.stamp = ros::Time::now();
              focusVoxel.pose.position.x = POI.x(); focusVoxel.pose.position.y = POI.y(); focusVoxel.pose.position.z = POI.z();
              focusVoxel.id = 1;
              focusVoxel.scale.x = markerSize; focusVoxel.scale.y = markerSize; focusVoxel.scale.z = markerSize;
              focusVoxel.color.r = 1.0f; focusVoxel.color.g = 1.0f; focusVoxel.color.b = 1.0f; focusVoxel.color.a = 1.0;
              focusMarkers.markers.push_back(focusVoxel);

              cur_focus_pub.publish(delMarkers);
              poll_rate.sleep();
              cur_focus_pub.publish(focusMarkers);
              poll_rate.sleep();

              ros::Duration(0.5).sleep();
              runningVisitedVoxels->push_back(tempPoint);
            }
          }
        
          inspectedVoxelMarkers.header.stamp = ros::Time::now();
          id4Markers = 0;
          for(int markerIt = 0; markerIt < runningVisitedVoxels->size(); markerIt++)
          {
            inspectedVoxelMarkers.pose.position.x = runningVisitedVoxels->at(markerIt).x; inspectedVoxelMarkers.pose.position.y = runningVisitedVoxels->at(markerIt).y; inspectedVoxelMarkers.pose.position.z = runningVisitedVoxels->at(markerIt).z;
            inspectedVoxelMarkers.id = id4Markers;
            inspectedVoxelMarkers.scale.x = markerSize * 0.98; inspectedVoxelMarkers.scale.y = markerSize * 0.98; inspectedVoxelMarkers.scale.z = markerSize * 0.98;
            inspectedVoxels.markers.push_back(inspectedVoxelMarkers);
            id4Markers = id4Markers + 1;
          }
          inspectedVoxels_pub.publish(delMarkers);
          poll_rate.sleep();
          //ros::spinOnce();
          inspectedVoxels_pub.publish(inspectedVoxels);
          //ros::spinOnce();
          poll_rate.sleep();

          countCells(occupiedCount, freeCount, unknownCount);
          journalT = ros::Time::now();
          journalData << journalT - beginT << "," << tempCloudOccTrimmed->size() << "," << runningVisitedVoxels->size() << "," << occupiedCount << "," << freeCount << "," << unknownCount << "," << flight_Distance << "\n";

          //return 0;
          
          //ROS_INFO("test4");
          compT_endFlight = ros::Time::now();
          myfileCompTime<<loopNumber<<","<<compT_endAlgo-compT_begin<<","<<compT_endGTSP-compT_endAlgo<<","<<compT_endFlight-compT_endGTSP<<std::endl;
          //ROS_INFO("test7");
          for(int i=0;i<visitedPointsList->size()-1;i++){
            //ROS_INFO("test9");
            realDistance += sqrt(pow(visitedPointsList->at(i+1).x-visitedPointsList->at(i).x,2)+pow(visitedPointsList->at(i+1).y-visitedPointsList->at(i).y,2)+pow(visitedPointsList->at(i+1).z-visitedPointsList->at(i).z,2));
            //ROS_INFO("test10");
          }
          //ROS_INFO("test8");
          myfileDR<<updateT-beginT<<","<<realDistance<<std::endl;
          resetFlag_msg.data = true;
          resetFlag_pub.publish(resetFlag_msg);
          sleep(3);
          //ROS_INFO("test6");
        }
      }
      //ROS_INFO("test5");
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
    candidateViewPoints.markers.clear();

    std::cout<<"=============loop number: "<<loopNumber+1<<"========================"<<std::endl; // outputting loop number
    loopNumber++;
    poll_rate.sleep();
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
  journalData.close();
  return 0;
}
