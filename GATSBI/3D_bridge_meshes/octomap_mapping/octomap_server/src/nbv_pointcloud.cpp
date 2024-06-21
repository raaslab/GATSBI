#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>


std::mutex mutex_;
octomap::OcTree* fullOcTree = NULL;

void full_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
	// std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
	octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);

	std::unique_lock<std::mutex> lock(mutex_);

	fullOcTree = dynamic_cast<octomap::OcTree*>(absTree);

	ROS_INFO("Got pointcloud");
	// countCB++;
}


int main(int argc, char** argv){

	ros::init(argc, argv, "nbv_pointcloud");
	ros::NodeHandle n;

	ros::Subscriber fullTree_sub = n.subscribe("/octomap_full",1,full_cb);
	ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/octomap_points",100);
	float thresholdOcc = 1.0;
  	float thresholdFree = 0.0;

  	octomap::OcTree::leaf_iterator it;
  	octomap::OcTree::leaf_iterator endLeaf;
	ros::Rate r(1);

	ros::spinOnce();
	r.sleep();
	ros::spinOnce();
	r.sleep();

	int pub_count = 1;
	while(ros::ok())
	{

		int total = fullOcTree->getNumLeafNodes();

		ROS_INFO("Octomap Size: %d", total);

		pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
    	pointcloud->width = total; 
    	pointcloud->height = 1; 
    	pointcloud->points.resize (pointcloud->width * pointcloud->height);
    
    
    	total = 0;

	    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it)
	    {
			if(it->getValue()>thresholdOcc)
			{
				pointcloud->points[total].x = it.getX(); 
				pointcloud->points[total].y = it.getY(); 
				pointcloud->points[total].z = it.getZ();
				pointcloud->points[total].intensity = 1;
			}
			else if(it->getValue()<thresholdFree)
			{
				pointcloud->points[total].x = it.getX(); 
				pointcloud->points[total].y = it.getY(); 
				pointcloud->points[total].z = it.getZ();
				pointcloud->points[total].intensity = -1;
			}
			else
			{
				pointcloud->points[total].x = it.getX(); 
				pointcloud->points[total].y = it.getY(); 
				pointcloud->points[total].z = it.getZ();
				pointcloud->points[total].intensity = 0;
			}
			total++;
		}

		sensor_msgs::PointCloud2 ros_pointcloud;
		pcl::toROSMsg(*pointcloud.get(),ros_pointcloud );

		ros_pointcloud.header.seq = pub_count;
		ros_pointcloud.header.stamp = ros::Time::now();
		ros_pointcloud.header.frame_id = "world_enu";

		pointcloud_pub.publish(ros_pointcloud);
		ros::spinOnce();
		r.sleep();
		
	}
	return 0;
}