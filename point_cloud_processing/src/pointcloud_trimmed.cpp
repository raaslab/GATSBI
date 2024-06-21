#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
tf::TransformListener *listener = NULL;  

double filter_limit_low_x, filter_limit_high_x, filter_limit_low_y, filter_limit_high_y, filter_limit_low_z, filter_limit_high_z, filter_rotation_r, filter_rotation_p, filter_rotation_y;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
      
    /*tf::StampedTransform transform;
    listener->waitForTransform("/world", "/velodyne", input->header.stamp, ros::Duration(10.0) );
    listener->lookupTransform("/world", "/velodyne", input->header.stamp, transform);
    ROS_INFO("Got transform");

    
    sensor_msgs::PointCloud2 transformed_cloud;
    pcl_ros::transformPointCloud("world", *input, transformed_cloud, *listener);
    */


    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(temp_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(filter_limit_low_x, filter_limit_high_x);
    pass_x.filter(*cloud_filtered_x);
    

   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_filtered_x);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(filter_limit_low_y, filter_limit_high_y);
    pass_y.filter(*cloud_filtered_y);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_y.setInputCloud(cloud_filtered_y);
    pass_y.setFilterFieldName("z");
    pass_y.setFilterLimits(filter_limit_low_z, filter_limit_high_z);
    pass_y.filter(*cloud_filtered_z);
    */

    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    //boxFilter.setTranslation(Eigen::Vector3f(4, 1.5, 0));
    boxFilter.setMin(Eigen::Vector4f(filter_limit_low_x, filter_limit_low_y, filter_limit_low_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(filter_limit_high_x, filter_limit_high_y, filter_limit_high_z, 1.0));
    //boxFilter.setMin(Eigen::Vector4f(-1, -3, -100, 1.0));
    //boxFilter.setMax(Eigen::Vector4f(1, 3, 100, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(filter_rotation_r, filter_rotation_p, filter_rotation_y));
    boxFilter.setInputCloud(temp_cloud);
    boxFilter.filter(*bodyFiltered);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*bodyFiltered.get(), output);
    pub.publish(output);
    
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "pointcloud_trimmed");
    ros::NodeHandle nh;

    listener = new(tf::TransformListener);

    std::string input_topic;
    std::string output_topic;

    nh.param<std::string>("/pointcloud_trimmed/input_topic", input_topic, "velodyne_points");
    nh.param<std::string>("/pointcloud_trimmed/output_topic", output_topic, "velodyne_points_trimmed");
    nh.param<double>("/pointcloud_trimmed/filter_limit_low_x", filter_limit_low_x, -50.0);
    nh.param<double>("/pointcloud_trimmed/filter_limit_high_x", filter_limit_high_x, 50.0);
    nh.param<double>("/pointcloud_trimmed/filter_limit_low_y", filter_limit_low_y, -50.0);
    nh.param<double>("/pointcloud_trimmed/filter_limit_high_y", filter_limit_high_y, 50.0);
    nh.param<double>("/pointcloud_trimmed/filter_limit_low_z", filter_limit_low_z, -50.0);
    nh.param<double>("/pointcloud_trimmed/filter_limit_high_z", filter_limit_high_z, 50.0);
    nh.param<double>("/pointcloud_trimmed/filter_rotation_r", filter_rotation_r, 0.0);
    nh.param<double>("/pointcloud_trimmed/filter_rotation_p", filter_rotation_p, 0.0);
    nh.param<double>("/pointcloud_trimmed/filter_rotation_y", filter_rotation_y, 0.0);

    ROS_INFO("Input Topic: %s", input_topic.c_str());
    ROS_INFO("Output Topic: %s", input_topic.c_str());
    ROS_INFO("Filter Field Limits X: [%.2f, %.2f]", filter_limit_low_x, filter_limit_high_x);
    ROS_INFO("Filter Field Limits Y: [%.2f, %.2f]", filter_limit_low_y, filter_limit_high_y);
    ROS_INFO("Filter Field Limits Z: [%.2f, %.2f]", filter_limit_low_z, filter_limit_high_z);
    ros::Subscriber sub = nh.subscribe(input_topic, 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);

    ros::spin();
}
