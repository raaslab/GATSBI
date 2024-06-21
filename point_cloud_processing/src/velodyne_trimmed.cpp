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
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
tf::TransformListener *listener = NULL;  

std::string filter_field_name;
double filter_limit_low, filter_limit_high;

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

    /*   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(temp_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-5.0, 5.0);
    pass_x.filter(*cloud_filtered_x);
    */

   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(temp_cloud);
    pass_y.setFilterFieldName(filter_field_name);
    pass_y.setFilterLimits(filter_limit_low, filter_limit_high);
    pass_y.filter(*cloud_filtered_y);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered_y.get(), output);
    pub.publish(output);
    
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "velodyne_trimmed");
    ros::NodeHandle nh;

    listener = new(tf::TransformListener);

    std::string input_topic;
    std::string output_topic;

    nh.param<std::string>("input_topic", input_topic, "velodyne_points");
    nh.param<std::string>("output_topic", output_topic, "velodyne_points_trimmed");
    nh.param<std::string>("filter_field_name", filter_field_name, "y");
    nh.param<double>("filter_limit_low", filter_limit_low, 0.0);
    nh.param<double>("filter_limit_high", filter_limit_high, 5.0);

    ros::Subscriber sub = nh.subscribe(input_topic, 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);

    ros::spin();
}
