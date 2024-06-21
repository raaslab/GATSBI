#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#define EPSILON 1e-4
#define _USE_MATH_DEFINES
#include <cmath>

msr::airlib::MultirotorRpcLibClient client;
DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom;
const YawMode yaw_mode(true, 0);
tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped base_link_to_world_ned;
ros::Publisher drone_1_done, drone_2_done;
double velocity = 1.0;

double collisionTime1 = 0.0;
double collisionTime2 = 0.0;


bool double_equals(double a, double b)
{
    return (fabs(a - b)<EPSILON);
}

geometry_msgs::Pose transformPose(geometry_msgs::Pose in, std::string target_frame, std::string source_frame)
{
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
    
    base_link_to_world_ned = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(5.0) );
    tf2::doTransform(in_stamped, transformed_stamped, base_link_to_world_ned);
    geometry_msgs::Pose out;
    out = transformed_stamped.pose;

    return out;
}


void moveDrone1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Commands need to be in "world_ned" frame for AirSim API
    ROS_INFO("I heard: [%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    geometry_msgs::Pose goal_transformed = transformPose(msg->pose, "world_ned", msg->header.frame_id);
    ROS_INFO("Transformed position: [%f, %f, %f]", goal_transformed.position.x, goal_transformed.position.y, goal_transformed.position.z);

    geometry_msgs::Pose center;
    center.position.x = msg->pose.orientation.x;
    center.position.y = msg->pose.orientation.y;
    center.position.z = msg->pose.orientation.z;

    geometry_msgs::Pose center_transformed = transformPose(center, "world_ned", "pointr");

    auto position = client.getMultirotorState("drone_1").getPosition();
    double dist = sqrt(pow(goal_transformed.position.x - position.x(),2) + pow(goal_transformed.position.y - position.y(),2) + pow(goal_transformed.position.z - position.z(),2));
    double wait_Time = dist / velocity; 
    wait_Time += 1.5;
    ROS_INFO("Wait time: %f", wait_Time);

    client.moveToPositionAsync(goal_transformed.position.x, goal_transformed.position.y, goal_transformed.position.z, 1.5, wait_Time, drivetrain, yaw_mode, -1, 1, "drone_1");
    ros::Duration(wait_Time + 1.1).sleep();

    
    msr::airlib::CollisionInfo collision = client.simGetCollisionInfo("drone_1");
    ROS_INFO("Drone 1 Collision Information: %d", collision.has_collided);
    if(collision.has_collided)
    {
        ROS_INFO("Robot has collided");
        ROS_INFO("Previous Collision Time: %f", collisionTime1);
        ROS_INFO("Current Collision Time: %lu", collision.time_stamp);
        
        if(collision.time_stamp > collisionTime1)
        {
            ROS_INFO("Robot just collided, correction position");
            collisionTime1 = collision.time_stamp;
            position = client.getMultirotorState("drone_1").getPosition();

            geometry_msgs::Pose freePosition;

            freePosition.position.x = 3 * (collision.impact_point[0] - position.x());
            freePosition.position.y = 3 * (collision.impact_point[1] - position.y());
            freePosition.position.z = 3 * (collision.impact_point[2] - position.z());

            freePosition.position.x = position.x() - freePosition.position.x;
            freePosition.position.y = position.y() - freePosition.position.y;
            freePosition.position.z = position.z() - freePosition.position.z;

            dist = sqrt(pow((freePosition.position.x) - position.x(),2) + pow(freePosition.position.y - position.y(),2) + pow(freePosition.position.z - position.z(),2));
            wait_Time = dist / velocity; 
            wait_Time += 1.5;
            geometry_msgs::Pose freePosition_transformed = transformPose(freePosition, "world_enu", "world_ned");
            ROS_INFO("Drone 1 moving to collision free position: [%.2f, %.2f, %.2f].", freePosition_transformed.position.x, freePosition_transformed.position.y, freePosition_transformed.position.z);
            client.moveToPositionAsync(freePosition.position.x, freePosition.position.y, freePosition.position.z, 1.5, wait_Time, drivetrain, yaw_mode, -1, 1, "drone_1");
            ros::Duration(wait_Time + 1.1).sleep();
        }
    }
    
    double direction_y = center_transformed.position.y - goal_transformed.position.y;
    double direction_x = center_transformed.position.x - goal_transformed.position.x;
    double angle = atan2(direction_y, direction_x);

    angle = angle * 180 / M_PI;

    ROS_INFO("Target step orientation yaw: %f", angle);
    client.rotateToYawAsync(angle, 3, 5, "drone_1");
    sleep(3.5);

    std_msgs::Bool message;
    message.data = true;
    drone_1_done.publish(message);
}

void moveDrone2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // Commands need to be in "world_ned" frame for AirSim API
    geometry_msgs::Pose goal_transformed = transformPose(msg->pose, "world_ned", msg->header.frame_id);

    //Drone 2 starting position is -4 in X compared to Drone 1. Subtract 4 to X value.
    goal_transformed.position.x -= 4;
    ROS_INFO("Transformed position: [%f, %f, %f]", goal_transformed.position.x, goal_transformed.position.y, goal_transformed.position.z);

    geometry_msgs::Pose center;
    center.position.x = msg->pose.orientation.x;
    center.position.y = msg->pose.orientation.y;
    center.position.z = msg->pose.orientation.z;

    geometry_msgs::Pose center_transformed = transformPose(center, "world_ned", "pointr");

    //Drone 2 starting position is -4 in X compared to Drone 1. Subtract 4 to X value.
    center_transformed.position.x -= 4;

    auto position = client.getMultirotorState("drone_2").getPosition();
    double dist = sqrt(pow((goal_transformed.position.x) - position.x(),2) + pow(goal_transformed.position.y - position.y(),2) + pow(goal_transformed.position.z - position.z(),2));
    double wait_Time = dist / velocity; 
    wait_Time += 1.5;
    ROS_INFO("Wait time: %f", wait_Time);

    client.moveToPositionAsync(goal_transformed.position.x, goal_transformed.position.y, goal_transformed.position.z, 1.5, wait_Time, drivetrain, yaw_mode, -1, 1, "drone_2");
    ros::Duration(wait_Time + 1.1).sleep();

    
    msr::airlib::CollisionInfo collision = client.simGetCollisionInfo("drone_2");
    ROS_INFO("Drone 2 Collision Information: %d", collision.has_collided);
    if(collision.has_collided)
    {
        ROS_INFO("Robot has collided");
        ROS_INFO("Previous Collision Time: %f", collisionTime2);
        ROS_INFO("Current Collision Time: %lu", collision.time_stamp);
        
        if(collision.time_stamp > collisionTime2)
        {
            ROS_INFO("Robot just collided, correction position");
            collisionTime2 = collision.time_stamp;
            
            position = client.getMultirotorState("drone_2").getPosition();
            
            geometry_msgs::Pose freePosition;

            freePosition.position.x = 3 * (collision.impact_point[0] - position.x());
            freePosition.position.y = 3 * (collision.impact_point[1] - position.y());
            freePosition.position.z = 3 * (collision.impact_point[2] - position.z());

            freePosition.position.x = position.x() - freePosition.position.x;
            freePosition.position.y = position.y() - freePosition.position.y;
            freePosition.position.z = position.z() - freePosition.position.z;

            geometry_msgs::Pose freePosition_transformed = transformPose(freePosition, "world_enu", "world_ned");
            ROS_INFO("Drone 2 moving to collision free position: [%.2f, %.2f, %.2f].", freePosition_transformed.position.x, freePosition_transformed.position.y, freePosition_transformed.position.z);
            dist = sqrt(pow((freePosition.position.x) - position.x(),2) + pow(freePosition.position.y - position.y(),2) + pow(freePosition.position.z - position.z(),2));
            wait_Time = dist / velocity; 
            wait_Time += 1.5;
            client.moveToPositionAsync(freePosition.position.x, freePosition.position.y, freePosition.position.z, 1.5, wait_Time, drivetrain, yaw_mode, -1, 1, "drone_2");
            ros::Duration(wait_Time + 1.1).sleep();
        }
    }
    
    
    double direction_y = center_transformed.position.y - goal_transformed.position.y;
    double direction_x = center_transformed.position.x - goal_transformed.position.x;
    double angle = atan2(direction_y, direction_x);

    angle = angle * 180 / M_PI;

    ROS_INFO("Target step orientation yaw: %f", angle);
    client.rotateToYawAsync(angle, 3, 5, "drone_2");
    sleep(3.5);

    std_msgs::Bool message;
    message.data = true;
    drone_2_done.publish(message);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "airsim_multiagent_client");    
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Subscriber drone_1_sub = nh.subscribe("/drone_1_command", 1000, moveDrone1Callback);
    ros::Subscriber drone_2_sub = nh.subscribe("/drone_2_command", 1000, moveDrone2Callback);

    drone_1_done = nh.advertise<std_msgs::Bool>("/drone_1_arrived", 1000);
    drone_2_done = nh.advertise<std_msgs::Bool>("/drone_2_arrived", 1000);

    client.enableApiControl(true, "drone_1");
    client.enableApiControl(true, "drone_2");
    client.armDisarm(true, "drone_1");
    client.armDisarm(true, "drone_2");
    
    client.takeoffAsync(5, "drone_1");
    client.takeoffAsync(5, "drone_2");
    ros::Duration(5.0).sleep();

    client.moveToPositionAsync(0, 0, -5, 1, 4, drivetrain, yaw_mode, -1, 1, "drone_1");
    client.moveToPositionAsync(0, 0, -5, 1, 4, drivetrain, yaw_mode, -1, 1, "drone_2");
    //MultirotorRpcLibClient* moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec = Utils::max<float>(),
    //                                                DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(),
    //                                                float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
        
    ros::Duration(4).sleep();
    client.rotateToYawAsync(-90, 3, 5, "drone_1");
    client.rotateToYawAsync(-90, 3, 5, "drone_2");
    ros::Duration(4).sleep();

    ROS_INFO("Takeoff successful");

    ros::waitForShutdown();
    //ros::spin();

    return 0;
}