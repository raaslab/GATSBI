#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <airsim_moveit_navigation/AirSim_NavigationAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <math.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <limits>
#include <mutex>
#include<unistd.h> 

tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped base_link_to_world_ned;
geometry_msgs::Pose currentPose;
geometry_msgs::PoseArray visitedPoseList;

pcl::PointCloud<pcl::PointXYZ>::Ptr runningVisitedVoxels (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);

octomap::OcTree* fullOcTree = NULL;
octomap::OcTree* trimmedOcTree = NULL;

double flight_Distance = 0;
double previousX = 0;
double previousY = 0;
double previousZ = 0;

float resolution = 1;

bool got_odom = false;
bool visited_poses_ready = false;
bool fullOcReady = false;
bool trimmedOcReady = false;

float thresholdDis = 0.1;
float viewingConeAngleThreshold = 10;
float viewingConeAngleBuffer = 0;
float minRadius = 5;
float maxRadius = 10;

float sensor_range = 30;

std_msgs::Bool resetFlag_msg;

std::mutex mutex_;
std::mutex mutex_trimmed;

#define EPSILON 1e-4

bool double_equals(double a, double b)
{
    return (fabs(a - b)<EPSILON);
}

float dot(tf::Vector3 a, tf::Vector3 b)
{
  return ((a.x() * b.x()) + (a.y() * b.y()) + (a.z() * b.z()));
}


float mag(tf::Vector3 a)  //calculates magnitude of a
{
    return std::sqrt((a.x() * a.x()) + (a.y() * a.y()) + (a.z() * a.z()));
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

void imu_cb(const nav_msgs::Odometry::ConstPtr msg){ // imu
  //ROS_INFO("Got odometry");
  geometry_msgs::Pose odometry_information = transformPose(msg->pose.pose, "world_enu", "world_ned");
  
  currentPose = odometry_information;
  got_odom = true;

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

void visitedPoseList_cb(const geometry_msgs::PoseArray::ConstPtr& msg){
  //ROS_INFO("Got visited points list");
  visitedPoseList.poses.clear();
  BOOST_FOREACH(const geometry_msgs::Pose& pose, msg->poses){
    visitedPoseList.poses.push_back(pose);
  }
  visited_poses_ready = true;
}

void zFiltered_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
  //ROS_INFO("Got z filtered point cloud");
  tempCloudOccTrimmed->clear();
  BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points){
    tempCloudOccTrimmed->points.push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
  }
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

double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    return sqrt(pow(pose1.position.x - pose2.position.x,2) + 
                pow(pose1.position.y - pose2.position.y,2) + 
                pow(pose1.position.z - pose2.position.z,2));
}

int main (int argc, char **argv)
{

  std::ofstream journalData;
  journalData.open("/home/user/bridgeInspection/journalData_baseline.csv");

  journalData << "Time,Inspectable Voxels,Inspected Voxels,Occupied Voxels,Free Voxels,Unknown Voxels,Flight Distance\n";

  ros::init(argc, argv, "baseline_gatsbi_client");
  ros::NodeHandle n;

  ros::Subscriber uavIMU_sub = n.subscribe("/airsim_node/drone_1/odom_local_ned",1,imu_cb);
  ros::Subscriber zFiltered_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/zFiltered",1,zFiltered_cb);
  ros::Subscriber fullTree_sub = n.subscribe("/octomap_full",1,full_cb);
  ros::Subscriber trimmedTree_sub = n.subscribe("/octomap_full_trimmed",1,trimmed_cb);
  ros::Subscriber visitedPoseList_sub = n.subscribe("/visited_poses_list", 1, visitedPoseList_cb);

  ros::Publisher inspectedVoxels_pub = n.advertise<visualization_msgs::MarkerArray>("/inspected_voxels",1,true);
  ros::Publisher resetFlag_pub = n.advertise<std_msgs::Bool>("/resetFlag",1);
  ros::Publisher baseline_done_pub = n.advertise<std_msgs::Bool>("/baseline_done",1,true);

  ros::Rate poll_rate(100);

  std::string path_file;
  n.param<std::string>("/baseline_gatsbi_client/path_file", path_file, "path.csv");
  n.param<float>("/baseline_gatsbi_client/resolution", resolution, 1);
  n.param<float>("/baseline_gatsbi_client/viewing_cone_angle_threshold", viewingConeAngleThreshold, 10.0);
  n.param<float>("/baseline_gatsbi_client/viewing_cone_angle_buffer", viewingConeAngleBuffer, 0.0);
  n.param<float>("/baseline_gatsbi_client/min_radius", minRadius, 5.0);
  n.param<float>("/baseline_gatsbi_client/max_radius", maxRadius, 10.0);
  n.param<float>("/octomap_server/sensor_model/max_range", sensor_range, 30);

  ROS_INFO("Path File: %s", path_file.c_str());
  ROS_INFO("Resolution: %.2f", resolution);
  ROS_INFO("Viewing Cone Angle: %.2f", viewingConeAngleThreshold);
  ROS_INFO("Viewing Cone Radius: [%.2f, %.2f]", minRadius, maxRadius);
  ROS_INFO("Sensor Range: [%.2f]", sensor_range);
  

  int occupiedCount = 0;
  int freeCount = 0;
  int unknownCount = 0;
  float markerSize = resolution;

  std::ifstream file(path_file.c_str());
  if (!file.is_open()) {
      ROS_ERROR("Failed to open file '%s'", path_file.c_str());
      return 1;
  }

  std::string line;
  std::vector<geometry_msgs::Pose> poses;
  
  // Skip the header line
  std::getline(file, line);
  
  // Read the CSV file line by line
  while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string token;
      geometry_msgs::Pose pose;
      
      // Skip the timestamp
      std::getline(ss, token, ',');

      // Read the position
      std::getline(ss, token, ',');
      pose.position.x = std::stod(token);
      std::getline(ss, token, ',');
      pose.position.y = std::stod(token);
      std::getline(ss, token, ',');
      pose.position.z = std::stod(token);

      // Read the orientation
      std::getline(ss, token, ',');
      pose.orientation.x = std::stod(token);
      std::getline(ss, token, ',');
      pose.orientation.y = std::stod(token);
      std::getline(ss, token, ',');
      pose.orientation.z = std::stod(token);
      std::getline(ss, token, ',');
      pose.orientation.w = std::stod(token);

      geometry_msgs::Pose pose_transformed = transformPose(pose, "world_enu", "kopt_frame");
      poses.push_back(pose_transformed);
  }
  file.close();

  if (poses.empty()) {
      ROS_ERROR("No poses found in '%s'", path_file.c_str());
      return 1;
  }

  actionlib::SimpleActionClient<airsim_moveit_navigation::AirSim_NavigationAction> ac("airsim_navigator", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Connected to Action Server.");
  
  ros::Time beginT = ros::Time::now();
  ros::Time journalT = ros::Time::now();

  ROS_INFO("Waiting for Odom.");
  got_odom = false;
  while(!got_odom)
  {
    poll_rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Got Odom.");
  /*
  ROS_INFO("Got Odom, shifting poses so first is closest to UAV.");

  // Find the pose closest to the origin
  
  double min_distance = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < poses.size(); ++i) {
      double distance = calculateDistance(poses[i], currentPose);
      if (distance < min_distance) {
          min_distance = distance;
          min_index = i;
      }
  }

  // Shift the poses so that the closest one is first
  std::rotate(poses.begin(), poses.begin() + min_index, poses.end());
  */

  airsim_moveit_navigation::AirSim_NavigationGoal goal;

  visualization_msgs::MarkerArray inspectedVoxels;
  visualization_msgs::MarkerArray delMarkers;
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker inspectedVoxelMarkers; inspectedVoxelMarkers.header.frame_id = "/world_enu"; inspectedVoxelMarkers.header.stamp = ros::Time::now(); inspectedVoxelMarkers.ns = "inspected_voxel_markers";
  inspectedVoxelMarkers.type = shape; inspectedVoxelMarkers.action = visualization_msgs::Marker::ADD;
  inspectedVoxelMarkers.color.r = 1.0f; inspectedVoxelMarkers.color.g = 0.0f; inspectedVoxelMarkers.color.b = 0.0f; inspectedVoxelMarkers.color.a = 1.0;

  visualization_msgs::Marker delMarker; delMarker.id = 0; delMarker.ns = "delete_markers"; delMarker.action = visualization_msgs::Marker::DELETEALL;

  for(int i = 0; i < poses.size(); i++)
  {
    if(i > 0)
    {
      if(double_equals(poses[i].position.x, poses[i-1].position.x) &&
         double_equals(poses[i].position.y, poses[i-1].position.y) &&
         double_equals(poses[i].position.z, poses[i-1].position.z) &&
         double_equals(poses[i].orientation.x, poses[i-1].orientation.x) &&
         double_equals(poses[i].orientation.y, poses[i-1].orientation.y) &&
         double_equals(poses[i].orientation.z, poses[i-1].orientation.z) &&
         double_equals(poses[i].orientation.w, poses[i-1].orientation.w))
      {
        continue;
      }
    }
    ROS_INFO("Moving to Pose[%d]: [%.2f, %.2f, %.2f]", (i+1), poses[i].position.x, poses[i].position.y, poses[i].position.z);
    ROS_INFO("with Orientation: [%.2f, %.2f, %.2f, %.2f]", poses[i].orientation.w, poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z);
    goal.goal_pose = poses[i];
    resetFlag_msg.data=false;
    resetFlag_pub.publish(resetFlag_msg);

    ROS_INFO("Send action goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    actionlib::SimpleClientGoalState state = ac.getState();
    

    fullOcReady = false;
    trimmedOcReady = false;
    got_odom = false;
    visited_poses_ready = false;
    while(!fullOcReady && !trimmedOcReady && !got_odom && !visited_poses_ready)
    {
      poll_rate.sleep();
      ros::spinOnce();
    }

    ROS_INFO("Received updated full and trimmed octree, odometry, and visited poses.");
    countCells(occupiedCount, freeCount, unknownCount);
    journalT = ros::Time::now();
    journalData << journalT - beginT << "," << tempCloudOccTrimmed->size() << "," << runningVisitedVoxels->size() << "," << occupiedCount << "," << freeCount << "," << unknownCount << "," << flight_Distance << "\n";

    if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      continue;
    }

    int previousChecked = 0;
    for(int l=0;l<visitedPoseList.poses.size();l++)
    {
      
      geometry_msgs::Pose curVisitedPose = visitedPoseList.poses[l];
      tf::Quaternion q(curVisitedPose.orientation.x, curVisitedPose.orientation.y, curVisitedPose.orientation.z, curVisitedPose.orientation.w);
      tf::Vector3 p(curVisitedPose.position.x, curVisitedPose.position.y, curVisitedPose.position.z);
      //ROS_INFO("Start Point Vector [%.2f, %.2f, %.2f]", p.x(), p.y(), p.z());
      //ROS_INFO("Orientation Vector [%.2f, %.2f, %.2f, %f]", q.x(), q.y(), q.z(), q.w());
      tf::Transform myPose(q, p);
      tf::Vector3 x_axis(1, 0, 0);
      tf::Vector3 D = myPose.getBasis() * x_axis;
      //ROS_INFO("Orientation Direction Vector [%.2f, %.2f, %.2f]", D.x(), D.y(), D.z());
      int count = 0;
      for(int j=0;j<tempCloudOccTrimmed->size();j++)
      {
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
        if(angle <= (viewingConeAngleThreshold + viewingConeAngleBuffer))
        {
          octomap::KeyRay ray;
          octomap::point3d origin(curVisitedPose.position.x, curVisitedPose.position.y, curVisitedPose.position.z);
          octomap::point3d end(tempCloudOccTrimmed->at(j).x, tempCloudOccTrimmed->at(j).y, tempCloudOccTrimmed->at(j).z);
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
                  break;
                }
                count += 1;
                pcl::PointXYZ tempPoint(tempCloudOccTrimmed->at(j).x, tempCloudOccTrimmed->at(j).y, tempCloudOccTrimmed->at(j).z);
                bool found = false;
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
              }
              else
              {
                break;
              }
            }
           } 
          //else
          //  continue;
        } 
      }
      ROS_INFO("Voxels Inpsectable from Current Position: [%d]", count);
    }

    resetFlag_msg.data=true;
    resetFlag_pub.publish(resetFlag_msg);
    ROS_INFO("Resetting visited poses.");
    sleep(3);

    markerSize = resolution;
    inspectedVoxelMarkers.header.stamp = ros::Time::now();
    int id4Markers = 0;
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
  
    countCells(occupiedCount, freeCount, unknownCount);
    journalT = ros::Time::now();
    journalData << journalT - beginT << "," << tempCloudOccTrimmed->size() << "," << runningVisitedVoxels->size() << "," << occupiedCount << "," << freeCount << "," << unknownCount << "," << flight_Distance << "\n";

  }

  std_msgs::Bool baseline_done;
  baseline_done.data = true;
  baseline_done_pub.publish(baseline_done);

  sleep(10);
  /*
  airsim_moveit_navigation::AirSim_NavigationGoal goal;

  goal.goal_pose.position.x = 1;
  goal.goal_pose.position.y = 1;
  goal.goal_pose.position.z = 4;

  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = 0;
  goal.goal_pose.orientation.w = 1;

  ROS_INFO("Send action goal");
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult();

  if(finished_before_timeout)
  {
  	actionlib::SimpleClientGoalState state = ac.getState();
  	ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  	ROS_INFO("Action did not finish before the time out.");

  goal.goal_pose.position.x = -2;
  goal.goal_pose.position.y = -2;
  goal.goal_pose.position.z = 6;

  ac.sendGoal(goal);

  finished_before_timeout = ac.waitForResult();

  if(finished_before_timeout)
  {
  	actionlib::SimpleClientGoalState state = ac.getState();
  	ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  	ROS_INFO("Action did not finish before the time out.");
  */

  return 0;
}