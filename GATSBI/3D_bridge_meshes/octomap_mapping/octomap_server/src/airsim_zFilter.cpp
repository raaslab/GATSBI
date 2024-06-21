#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
// #include <vector>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// ros::Publisher occArrayTrimmed_pub;
float thresholdOcc = 1.0;
float resolution = 1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccFull (new pcl::PointCloud<pcl::PointXYZ>);
visualization_msgs::MarkerArray zFiltered;
uint32_t shape = visualization_msgs::Marker::CUBE;
float trimmedHeight = 3; // 3 is standard
std::vector<double> outputVector;
std_msgs::Float64MultiArray outputSize;


void trimmed_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
	octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
	octomap::OcTree* trimmedOcTree = new octomap::OcTree(resolution);
	trimmedOcTree = dynamic_cast<octomap::OcTree*>(absTree);
	octomap::OcTree::leaf_iterator it;
	octomap::OcTree::leaf_iterator endLeaf;
	visualization_msgs::Marker zFilteredMarkers; zFilteredMarkers.header.frame_id = "/world_enu"; zFilteredMarkers.header.stamp = ros::Time::now(); zFilteredMarkers.ns = "free_markers_trimmed";
	zFilteredMarkers.type = shape; zFilteredMarkers.action = visualization_msgs::Marker::ADD;
	int id4Markers = 0; float markerSize = resolution; int sizeOfCloud = 0;

	for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
		if(it->getValue()>thresholdOcc){
			if(it.getZ()>trimmedHeight){
				sizeOfCloud++;
			}
		}
	}

	cloudOccFull->clear();
	cloudOccFull->header.frame_id = "/world_enu";
	//cloudOccFull->width = sizeOfCloud; cloudOccFull->height = 1; cloudOccFull->points.resize (cloudOccFull->width * cloudOccFull->height);
	for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
		if(it->getValue()>thresholdOcc){
			if(it.getZ()>trimmedHeight){
				//cloudOccFull->points[id4Markers].x = it.getX(); cloudOccFull->points[id4Markers].y = it.getY(); cloudOccFull->points[id4Markers].z = it.getZ();
				cloudOccFull->points.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
				outputVector.push_back(it.getSize());
				zFilteredMarkers.pose.position.x = it.getX(); zFilteredMarkers.pose.position.y = it.getY(); zFilteredMarkers.pose.position.z = it.getZ();
				zFilteredMarkers.id = id4Markers;
				if(it.getSize() > resolution)
				{
					ROS_INFO("Voxel %d Resolution: [%.2f]", id4Markers, it.getSize());
				}
				zFilteredMarkers.scale.x = it.getSize(); zFilteredMarkers.scale.y = it.getSize(); zFilteredMarkers.scale.z = it.getSize();
				if(it.getZ()>4){
					zFilteredMarkers.color.r = 1.0f; zFilteredMarkers.color.g = 0.0f; zFilteredMarkers.color.b = 0.0f; zFilteredMarkers.color.a = 1.0;
				}
				else if(it.getZ()>1.9){
					zFilteredMarkers.color.r = 0.0f; zFilteredMarkers.color.g = 1.0f; zFilteredMarkers.color.b = 0.0f; zFilteredMarkers.color.a = 1.0;
				}
				else{
					zFilteredMarkers.color.r = 0.0f; zFilteredMarkers.color.g = 0.0f; zFilteredMarkers.color.b = 1.0f; zFilteredMarkers.color.a = 1.0;
				}
				zFiltered.markers.push_back(zFilteredMarkers);
				id4Markers++;
			}
		}
	}
	ROS_INFO("Size of cloud: %zu", cloudOccFull->size());
}

int main(int argc, char** argv){
	ros::init(argc, argv, "zFilter");
	ros::NodeHandle n;
	ros::Publisher output_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/zFiltered",1,true);
	ros::Publisher outputSize_pub = n.advertise<std_msgs::Float64MultiArray> ("/zFilteredSize",1,true);
	ros::Publisher zFiltered_pub = n.advertise<visualization_msgs::MarkerArray>("/zFiltered_Markers",1,true);

	ros::Subscriber trimmedTree_sub = n.subscribe("octomap_full_trimmed",1,trimmed_cb);

	n.param<float>("/airsim_zFilter/resolution", resolution, 1.0);
	n.param<float>("/airsim_zFilter/trimmed_height", trimmedHeight, 3.0);

	visualization_msgs::MarkerArray delMarkers;
	visualization_msgs::Marker delMarker; delMarker.id = 0; delMarker.ns = "delete_markers"; delMarker.action = visualization_msgs::Marker::DELETEALL;
  	delMarkers.markers.push_back(delMarker);

	ros::Rate loop_rate(1);
	ROS_INFO("Finished initializing all parts of ROS");
	// int loopNumber = 0;

	bool del_marker_array = false;

	while (ros::ok()){
		// ROS_INFO("While loop");
		ros::spinOnce();
		pcl_conversions::toPCL(ros::Time::now(), cloudOccFull->header.stamp);
		outputSize.data.resize(outputVector.size());
		for(int i = 0;i<outputVector.size();i++){
			outputSize.data[i] = outputVector.at(i);
		}
		ROS_INFO("New size: %zu",outputSize.data.size());
		if(cloudOccFull->size()){
			output_pub.publish(cloudOccFull);
			outputSize_pub.publish(outputSize);
		}
		if(del_marker_array)
		{
			zFiltered_pub.publish(delMarker);
			del_marker_array = false;
		}
		else {
			if(zFiltered.markers.size()>2){
				zFiltered_pub.publish(zFiltered);
				del_marker_array = true;
			}
		}
		loop_rate.sleep();
		zFiltered.markers.clear();
		cloudOccFull->clear();
		outputSize.data.clear();
		outputVector.clear();
	}
}
