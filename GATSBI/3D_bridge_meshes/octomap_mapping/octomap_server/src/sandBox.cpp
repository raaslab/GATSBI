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
// #include "sensor_msgs/msg/NavSatFix"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

int main(int argc, char** argv){
	float num1 = 2.234;
	float num2 = 2.245;
	float answer1 = abs(num1-num2);
	float answer2 = abs(num2-num1);
	std::vector<float> v;
	pcl::PointCloud<pcl::PointXYZ>::Ptr returnPoints (new pcl::PointCloud<pcl::PointXYZ>);
	returnPoints->push_back({2,3,4});
	v = {2,2,0,4,4.009,6,8,10,-1,12,12};
	// v.erase(v.begin()+1);
	// removeIfTwoValues(v);
	// std::vector<float>::iterator result = std::min_element(v.begin(),v.end());
	// std::cout << "min element: " << v.at(std::distance(v.begin(),result))-1.234554<<std::endl;
	// for (auto it = v.cbegin(); it != v.cend(); ++it){
	// 	std::cout << *it << ' ';
	// }
	// std::cout<<std::endl;
	// std::cout<<checkIfFloatsAreTheSame(num1, num2, 100)<<std::endl;
	std::vector<float> testVariable;
	std::vector<float> inputVariable;
	inputVariable.push_back(7);
	// inputVariable.push_back(5);
	// inputVariable.push_back(2.3);
	// inputVariable.push_back(3);
	// inputVariable.push_back(9);
	// inputVariable.push_back(-7);
	// inputVariable.push_back(-5);
	// inputVariable.push_back(-2.3);
	// inputVariable.push_back(-3);
	// inputVariable.push_back(-9);
	// testCode(&testVariable);
	float minRadius = 1;
	float maxRadius = 8;
	std::cout<<*max_element(inputVariable.begin(),inputVariable.end())<<std::endl;
	for(int i = 0;i<minRadius;i++){
		std::cout<<i<<std::endl;
	}
	

}