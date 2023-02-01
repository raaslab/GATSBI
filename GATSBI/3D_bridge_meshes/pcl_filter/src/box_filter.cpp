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
#include <ctime>

ros::Publisher pub;

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input_pcl, std::string topic) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input_pcl, pcl_pc2);
   	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    int z_min, z_max, x_min, x_max, y_min, y_max;
    std::string axis;
   	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZI>);
    std::cout << topic << "\n";
    if (topic == "laser_cloud_surround") {
        z_min = 5;
        z_max = 10;
        x_min = -10;
        x_max = 0;
        // z_min = -10;
        // z_max = +10;
        y_min = -15;
        y_max = 10;
        // x_min = -10;
        // x_max = +10;
    } else {
        z_min = -1;
        z_max = 3;
        x_min = -20;
        x_max = 3;
        axis = "z";
    }

    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud(temp_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min, x_max);
    pass_x.filter(*cloud_filtered_y);
    
    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pass_z.setInputCloud(cloud_filtered_y);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min, z_max);
    pass_z.filter(*cloud_filtered_y);


    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(cloud_filtered_y);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min, y_max);
    pass_y.filter(*cloud_filtered_y);
    
    // pcl::io::savePCDFile("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd",
    //                      *cloud_filtered_y);
//  pcl::io::savePCDFile("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd",
//                          *temp_cloud);
    sensor_msgs::PointCloud2 ros_output_cloud;
    pcl::toROSMsg(*cloud_filtered_y, ros_output_cloud);
    // pcl::toROSMsg(*temp_cloud, ros_output_cloud);
    
    pub.publish(ros_output_cloud);

}

int main(int argc, char** argv) {
    ros::init (argc, argv, "pcl_filter");
    ros::NodeHandle node_handler;
    sensor_msgs::PointCloud2 output_pcl;
    std::string topic = argv[1];
    std::cout << "the topic being subscribed is " << topic << "\n";
    // ros::Subscriber sub = node_handler.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, boost::bind(&pcl_callback, _1, output_pcl));
    ros::Subscriber sub = node_handler.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(pcl_callback, _1, topic));

    pub = node_handler.advertise<sensor_msgs::PointCloud2>("filtered", 1);
    ros::spin();
}