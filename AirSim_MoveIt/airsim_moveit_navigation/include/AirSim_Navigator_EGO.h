#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap_msgs/conversions.h>
#include "quadrotor_msgs/PositionCommand.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <airsim_moveit_navigation/AirSim_NavigationAction.h>

#include <octomap/OcTree.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>
#include <unistd.h>
#define XMIN -70
#define XMAX 70
#define YMIN -70
#define YMAX 70
#define ZMIN 0.2
#define ZMAX 25

#define EPSILON 1e-4

#include <iostream>
#include <chrono>

using namespace std;
using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;

#include <omp.h>
class Quadrotor{
    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<airsim_moveit_navigation::AirSim_NavigationAction> as_;
        std::string action_name_;

        airsim_moveit_navigation::AirSim_NavigationFeedback feedback_;
        airsim_moveit_navigation::AirSim_NavigationResult result_;

        bool odom_received, odom_2_received,trajectory_received;
        bool isPathValid;
        bool collision;

        tf2_ros::Buffer tf_buffer;

        geometry_msgs::TransformStamped base_link_to_world_ned;

        geometry_msgs::Pose odometry_information, odometry_2_information, odometry_information_ned;
        std::vector<geometry_msgs::Pose> trajectory;
        
        ros::Subscriber base_sub, plan_sub,goal_sub,distance_sub,path_sub,nbv_path_sub,multiagent_nbv_path_sub, position_command_sub;
        ros::Publisher distance_pub;
        ros::Publisher path_pub, path_2_pub;
        ros::Publisher rrt_pub;
        ros::Publisher uav_pause_pub;
        ros::Publisher goal_pub;
        
        ros::ServiceClient planning_scene_service;
    
        octomap::OcTree* current_map;

        double previousX;
        double previousY;
        double previousZ;

        double previous2X;
        double previous2Y;
        double previous2Z;

        double goal_x;
        double goal_y;
        double goal_z;

        nav_msgs::Path path, path2;
        
        bool pos_com_received;

        bool traversing;

        double velocity;

        double collisionDistance;

        double collisionTime;

        int pause_interval;

        msr::airlib::MultirotorRpcLibClient client;
        
        const std::string PLANNING_GROUP = "DroneBody";

        geometry_msgs::Pose transformPose(geometry_msgs::Pose in, std::string target_frame, std::string source_frame);

        bool double_equals(double a, double b);

        bool checkCollision(geometry_msgs::Pose pose);

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg);

        void executeCB(const airsim_moveit_navigation::AirSim_NavigationGoalConstPtr &goal);
        
        void planCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);

        void moveFromObstacle(void);
    
    public:
        Quadrotor(ros::NodeHandle& nh, std::string name);
        void takeoff();
        void run();
        
};