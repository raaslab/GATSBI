// Copyright 2019, RAAS lab

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>

void dji_tf_callback(const geometry_msgs::PointStampedConstPtr &position_dji,
                    const geometry_msgs::QuaternionStampedConstPtr &attitude_dji) {
        static tf::TransformBroadcaster bcast;
        tf::Transform tf;
        tf::Quaternion dji_rot;

        tf.setOrigin(tf::Vector3(position_dji->point.x, position_dji->point.y, 
                                    position_dji->point.z));
        tf.setRotation(tf::Quaternion(attitude_dji->quaternion.x, attitude_dji->quaternion.y, 
                                    attitude_dji->quaternion.z, attitude_dji->quaternion.w));
        bcast.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", "dji"));
        
    }

int main(int argc, char** argv) {
    // Initialize topic and broadcaster.
    ros::init(argc, argv, "dji_tf_pub");

    ros::NodeHandle handler;

    // Pass the position and attitude topics through a filter,
    // so that they can by synced together.
    message_filters::Subscriber<geometry_msgs::PointStamped> position_dji
                                    (handler, "/dji_sdk/local_position", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_dji
                                    (handler, "/dji_sdk/attitude", 1);
    // Object call for devising a policy or parameters on how to 
    // synchronize the topics
    typedef message_filters::sync_policies::ApproximateTime
                <geometry_msgs::PointStamped, geometry_msgs::QuaternionStamped> 
                SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync_params(SyncPolicy(10), 
                                              position_dji, attitude_dji);
    // Using these parameters of the sync policy, get the messages, which are synced
    sync_params.registerCallback(boost::bind(&dji_tf_callback, _1, _2));
    
    ros::spin();
    return 0;
}