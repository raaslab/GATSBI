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
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

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
    tf_listener.waitForTransform("camera_init", "world", ros::Time(), ros::Duration(1.0));
    try {
        tf_listener.lookupTransform("camera_init", "world", 
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
    
    ros::init(argc, argv, "loam_to_world");
    ros::NodeHandle handler;

    ros::Subscriber sub = handler.subscribe("laser_cloud_surround", 1, cloud_pub_callback);
    
    pub = handler.advertise<sensor_msgs::PointCloud2>("loam_cloud_world", 1);
    ros::spin();
}
