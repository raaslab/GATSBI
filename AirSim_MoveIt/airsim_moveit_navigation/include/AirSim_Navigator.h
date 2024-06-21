#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap_msgs/conversions.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <airsim_moveit_navigation/AirSim_NavigationAction.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>
#include <unistd.h>

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
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        actionlib::SimpleActionServer<airsim_moveit_navigation::AirSim_NavigationAction> as_;
        std::unique_ptr<robot_state::RobotState> start_state;
        std::unique_ptr<planning_scene::PlanningScene> planning_scene;
        std::string action_name_;

        airsim_moveit_navigation::AirSim_NavigationFeedback feedback_;
        airsim_moveit_navigation::AirSim_NavigationResult result_;

        bool odom_received, odom_2_received,trajectory_received;
        bool isPathValid;
        bool collision;
         
        tf2_ros::Buffer tf_buffer;

        geometry_msgs::TransformStamped base_link_to_world_ned;

        geometry_msgs::Pose odometry_information, odometry_2_information;
        geometry_msgs::Pose odometry_information_enu;
        std::vector<geometry_msgs::Pose> trajectory;
        
        ros::Subscriber base_sub, base_2_sub, plan_sub,goal_sub,distance_sub,path_sub,nbv_path_sub,multiagent_nbv_path_sub,rotate_sub;
        ros::Publisher distance_pub;
        ros::Publisher path_pub, path_2_pub;
        ros::Publisher rrt_pub;
        ros::Publisher uav_pause_pub;
        ros::Publisher uav_moving_pub;

        ros::ServiceClient planning_scene_service;

        moveit_msgs::RobotState plan_start_state;
        moveit_msgs::RobotTrajectory plan_trajectory;
    
        octomap::OcTree* current_map;

        double previousX;
        double previousY;
        double previousZ;

        double previous2X;
        double previous2Y;
        double previous2Z;

        nav_msgs::Path path, path2;
        
        bool traversing;

        bool rotate_to_center;

        bool moveit_octomap_updated;

        bool last_move_free_space;

        std_msgs::Bool uav_moving;
        
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

        void pose2Callback(const nav_msgs::Odometry::ConstPtr & msg);

        void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

        void executeCB(const airsim_moveit_navigation::AirSim_NavigationGoalConstPtr &goal);

        void computePathLengthCB(const geometry_msgs::Point::ConstPtr &path);

        void computeRRTPathCB(const geometry_msgs::Point::ConstPtr &path);

        void computeNBVPathCB(const geometry_msgs::Point::ConstPtr &path);

        void rotateDroneCB(const geometry_msgs::Point::ConstPtr &POI);

        void computeMultiAgentNBVPathCB(const geometry_msgs::Pose::ConstPtr &path);
        
        void moveFromObstacle(void);

        void updateMoveitOctomap(void);

        geometry_msgs::Pose findFreeNeighbor(int neighbors);

        int numFreeNeighbors(octomap::OcTreeKey key);
    
    public:
        Quadrotor(ros::NodeHandle& nh, std::string name);
        void takeoff();
        void run();
        
};