// Copyright 2019, RAAS lab

#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <pcl/common/transforms.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Publisher pub;


void cloud_pub_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud) {
    pcl::PCLPointCloud2 pcl_input;
    pcl_conversions::toPCL(*input_cloud, pcl_input);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::fromPCLPointCloud2(pcl_input, *temp_cloud);
    tf::StampedTransform transformer;
    Eigen::Affine3d trans_eigen;
    tf::Vector3 origin;
    tf::Quaternion rot;
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform( "/world", "/velodyne",  ros::Time(), ros::Duration(1.0));
    try{
      tf_listener.lookupTransform("/world", "/velodyne",  
                                  ros::Time(0), transformer);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

    origin = transformer.getOrigin();
    rot = transformer.getRotation();
    tf::transformTFToEigen(transformer, trans_eigen);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Resize the about to be transformed point cloud to the input point cloud;
    transform_cloud->points.resize(temp_cloud->size());

    pcl::transformPointCloud(*temp_cloud, *transform_cloud, trans_eigen);

    sensor_msgs::PointCloud2 ros_output_cloud;
    pcl::toROSMsg(*transform_cloud, ros_output_cloud);
    pub.publish(ros_output_cloud);
}
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "pub_robust_pcl");
    ros::NodeHandle handler;

    ros::Subscriber sub = handler.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, cloud_pub_callback);
    
    pub = handler.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
    ros::spin();
}
