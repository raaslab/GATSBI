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
#include <pcl/filters/passthrough.h>
//#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/search/impl/search.hpp>
#include <ctime>

ros::Publisher pub;

void reg_multiple_callback(const sensor_msgs::PointCloud2ConstPtr &input, 
                           pcl::PointCloud<pcl::PointXYZRGB> &concat ) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl_input;
    pcl_conversions::toPCL(*input, pcl_input);
    pcl::fromPCLPointCloud2(pcl_input, *temp);
    // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // icp.setInputSource(temp);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr temp_concat = concat.makeShared() ;
    // icp.setInputTarget(temp_concat);
    // pcl::PointCloud<pcl::PointXYZI> final;
    // icp.setMaxCorrespondenceDistance (0.05);
    // // Set the maximum number of iterations (criterion 1)
    // icp.setMaximumIterations (100);
    // // Set the transformation epsilon (criterion 2)
    // icp.setTransformationEpsilon (1e-8);
    // // Set the euclidean distance difference epsilon (criterion 3)
    // icp.setEuclideanFitnessEpsilon (0.1);
    // icp.align(final);
    concat += *temp;           
    
    // pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    // pcl::toPCLPointCloud2(concat, *cloud);
    // pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud (cloud);
    // sor.setLeafSize (0.05f, 0.05f, 0.05f);
    // sor.filter (*cloud_filtered);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_concat(new pcl::PointCloud<pcl::PointXYZRGB>);

    // pcl::fromPCLPointCloud2(*cloud_filtered, *output_concat);
    sensor_msgs::PointCloud2 output_pcl;
    pcl::toROSMsg(concat, output_pcl);
    output_pcl.header.frame_id = "world";

    pub.publish(output_pcl);
}

int main(int argc, char **argv) {
    
    
    ros::init(argc, argv, "register_multiple");
    // ros::Rate rate(10);
    ros::NodeHandle node_handler;
    sensor_msgs::PointCloud2 output_pcl;
    // ros::Subscriber sub = node_handler.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, boost::bind(&pcl_callback, _1, output_pcl));
    pcl::PointCloud<pcl::PointXYZRGB> concat;
    boost::shared_ptr<const sensor_msgs::PointCloud2> first_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("transformed_cloud");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl_input;
    pcl_conversions::toPCL(*first_msg, pcl_input);
    pcl::fromPCLPointCloud2(pcl_input, concat);
    ros::Subscriber sub = node_handler.subscribe<sensor_msgs::PointCloud2>("transformed_cloud", 0.5 , 
                                            boost::bind(&reg_multiple_callback, _1, concat));

    pub = node_handler.advertise<sensor_msgs::PointCloud2>("register_multiple", 0.5);
    ros::spin();
}