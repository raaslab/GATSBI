// Copyright 2019, RAAS lab

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
// #include <Matrix3x3.h>
#include <iostream>

void intel_pose_callback(const nav_msgs::Odometry::ConstPtr &intel_pose,
                        const sensor_msgs::PointCloud2::ConstPtr &laser_scan) {
        static tf::TransformBroadcaster bcast;
        tf::Transform tf;
        tf::Quaternion rot;
        tf.setOrigin(tf::Vector3(intel_pose->pose.pose.position.y, -intel_pose->pose.pose.position.x, 
                                    -intel_pose->pose.pose.position.z));
        tf.setRotation(tf::Quaternion(intel_pose->pose.pose.orientation.y, intel_pose->pose.pose.orientation.x, 
                                    intel_pose->pose.pose.orientation.z, intel_pose->pose.pose.orientation.w));
        // tfScalar yaw, pitch, roll;
        // tf::Matrix3x3 mat(tf::Quaternion(intel_pose->pose.pose.orientation.x, intel_pose->pose.pose.orientation.y, 
        //                             intel_pose->pose.pose.orientation.z, intel_pose->pose.pose.orientation.w));
        // // mat.getEulerYPR(yaw, pitch, roll);                
        // tf::Quaternion qrot;
        // qrot.setEulerZYX(roll, yaw, pitch);
        // tf.setRotation(qrot);
        // qrot.g

        bcast.sendTransform(tf::StampedTransform(tf, ros::Time::now(),  "world", "cam_pose"));
        
    }

int main(int argc, char** argv) {
    
    
    // Initialize topic and broadcaster.
    ros::init(argc, argv, "intel_tf_pub");

    ros::NodeHandle handler;
    
    // Pass the position and attitude topics through a filter,
    // so that they can by synced together.
    message_filters::Subscriber<nav_msgs::Odometry> odom_intel
                                    (handler, "/camera/odom/sample", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> velo_scans
                                    (handler, "/velodyne_points", 1);
    // Object call for devising a policy or parameters on how to 
    // synchronize the topics
    typedef message_filters::sync_policies::ApproximateTime
                <nav_msgs::Odometry, sensor_msgs::PointCloud2> 
                SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync_params(SyncPolicy(5), 
                                              odom_intel, velo_scans);
    // Using these parameters of the sync policy, get the messages, which are synced
    sync_params.registerCallback(boost::bind(&intel_pose_callback, _1, _2));
    
    ros::spin();

    // ros::Subscriber sub = handler.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, intel_pose_callback
    
    // ros::spin();
    return 0;
}