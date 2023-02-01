//#include <pluginlib/class_loader.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
//#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

string nbv_file = "/home/user/nbv_position.txt";
msr::airlib::MultirotorRpcLibClient client;

geometry_msgs::Pose transformPose(geometry_msgs::Pose in, std::string target_frame, std::string source_frame)
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::PoseStamped in_stamped;
    geometry_msgs::PoseStamped transformed_stamped;

    in_stamped.pose.position.x = in.position.x;
    in_stamped.pose.position.y = in.position.y;
    in_stamped.pose.position.z = in.position.z;
    in_stamped.pose.orientation.x = in.orientation.x;
    in_stamped.pose.orientation.y = in.orientation.y;
    in_stamped.pose.orientation.z = in.orientation.z;
    in_stamped.pose.orientation.w = in.orientation.w;

    in_stamped.header.seq = 1;
    in_stamped.header.frame_id = source_frame;
    
    geometry_msgs::TransformStamped base_link_to_world_ned = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(5.0) );
    tf2::doTransform(in_stamped, transformed_stamped, base_link_to_world_ned);
    geometry_msgs::Pose out;
    out = transformed_stamped.pose;

    return out;
}

void nbv_callback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Got NBV ready message: [%d]", msg->data);
    if(msg->data)
    {
        std::ifstream in(nbv_file);
        if(!in)
        {
            ROS_INFO("Couldn't open NBV position file");
            return;
        }

        double pos_x = -1;
        double pos_y = -1;
        double yaw = -1;
        in >> pos_y >> pos_x >> yaw;

        ROS_INFO("Got NBV position: [%f, %f] and yaw: [%f]", pos_y, pos_x, yaw);
        
        pos_x = -1 * pos_x;

        yaw = yaw - 90;
        if(yaw < -180)
        {
            yaw += 360;
        }

        
        ROS_INFO("After coordinate frame transformation from NBV to AirSim");
        ROS_INFO("NBV target position: [%f, %f] and yaw: [%f]", pos_x, pos_y, yaw);

        auto position = client.getMultirotorState().getPosition();
        auto pose = client.simGetVehiclePose();
        ROS_INFO("Current position in world_ned frame: [%f, %f, %f]", pose.position.x(), pose.position.y(), pose.position.z());
        ROS_INFO("Current orientation in world_ned frame: [%f, %f, %f, %f]", pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());
        
        geometry_msgs::Pose current;
        current.position.x = position.x();
        current.position.y = position.y();
        current.position.z = position.z();
        current.orientation.x = pose.orientation.x();
        current.orientation.y = pose.orientation.y();
        current.orientation.z = pose.orientation.z();
        current.orientation.w = pose.orientation.w();
        geometry_msgs::Pose current_transformed = transformPose(current, "world_enu", "world_ned");

        ROS_INFO("Current transformed position in world_enu frame: [%f, %f, %f]", current_transformed.position.x, current_transformed.position.y, current_transformed.position.z);

        geometry_msgs::Pose input;
        input.position.x = pos_x;
        input.position.y = pos_y;
        input.position.z = current_transformed.position.z;
        input.orientation.x = 0;
        input.orientation.y = 0;
        input.orientation.z = 0;
        input.orientation.w = 1;

        ROS_INFO("Input target position in world_ned frame: [%f, %f, %f]", input.position.x, input.position.y, input.position.z);

        geometry_msgs::Pose transformed = transformPose(input, "world_ned", "world_enu");

        ROS_INFO("Transformed target position in world_enu frame: [%f, %f, %f]", transformed.position.x, transformed.position.y, transformed.position.z);

        ROS_INFO("Moving to target position");
        double waiting_time = sqrt((transformed.position.x * transformed.position.x) + (transformed.position.y * transformed.position.y)) + 1;

        client.moveToPositionAsync(transformed.position.x, transformed.position.y, transformed.position.z, 1, 3);ros::Duration(4).sleep();
        ros::Duration(waiting_time).sleep();
        client.rotateToYawAsync(yaw, 3);
        ros::Duration(4).sleep();
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "airsim_nbv_planner");    

    ros::NodeHandle node_handle;

    ros::Subscriber sub = node_handle.subscribe("/nbv_position_ready", 1000, nbv_callback);
    
    client.enableApiControl(true);
    client.armDisarm(true);
    
    
    client.takeoffAsync(5);
    ros::Duration(5.0).sleep();
    client.rotateToYawAsync(-90, 3);
    ros::Duration(4).sleep();

    auto position = client.getMultirotorState().getPosition();
    ROS_INFO("Current drone position in world_ned frame: [%f, %f, %f]", position.x(), position.y(), position.z());
    ros::spin();

    return 0;
}