// Copyright 2019, RAAS lab

#include <iostream>
#include <sstream>
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/tran>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
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
    // tf_listener.waitForTransform( "world", "laser0", ros::Time(), ros::Duration(1.0));
    // try {
    //     tf_listener.lookupTransform("world", "laser0" ,
    tf_listener.waitForTransform( "world", "laser0_frame", ros::Time(), ros::Duration(1.0));
    try {
        tf_listener.lookupTransform("world", "laser0_frame",
                                        ros::Time(0), transformer);
    }
    catch (tf::TransformException exception) {
        ROS_ERROR("%s", exception.what());
        return;
    }

    origin = transformer.getOrigin();
    rot = transformer.getRotation();
    rot.normalize();
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
    
    ros::init(argc, argv, "transform_cloud_velo_pub");
    ros::NodeHandle handler;

    ros::Subscriber sub = handler.subscribe("/velodyne_points", 1, cloud_pub_callback);
    
    pub = handler.advertise<sensor_msgs::PointCloud2>("transformed_cloud_velo", 1);
    ros::spin();
}
