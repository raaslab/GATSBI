// Copyright 2019, RAAS lab

#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>


ros::Publisher pose_pub;
int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle handler;

    // Initialize an odometry topic broadcaster
    pose_pub = handler.advertise<geometry_msgs::PoseStamped>("pose", 10);
    // Brodcast odom after lisening to them
    tf::TransformListener tf_listener;
    tf::StampedTransform transformer;
    geometry_msgs::PoseStamped pose_stamped;
    tf::Vector3 origin;
    tf::Quaternion rot;
    ros::Rate freq(10);
    while (ros::ok) {
        tf_listener.waitForTransform("velodyne", "world", ros::Time(), ros::Duration(1.0));
        try {
            tf_listener.lookupTransform("velodyne", "world",
                                            ros::Time(0), transformer);
        }
        catch (tf::TransformException exception) {
            ROS_ERROR("%s", exception.what());
            return -1;
        }

        origin = transformer.getOrigin();
        rot = transformer.getRotation();
        geometry_msgs::Pose pose;
        pose.position.x = origin.getX();
        pose.position.y = origin.getY();
        pose.position.z = origin.getZ();
        pose.orientation.w = rot.getW();
        pose.orientation.x = rot.getX();
        pose.orientation.y = rot.getY();
        pose.orientation.z = rot.getZ();
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.seq = 1;
        pose_stamped.pose = pose;
        pose_pub.publish(pose_stamped);
    }
    ros::spin();
    return 0;
}

