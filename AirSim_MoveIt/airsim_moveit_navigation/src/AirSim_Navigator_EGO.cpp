#include <AirSim_Navigator_EGO.h>

Quadrotor::Quadrotor(ros::NodeHandle& nh, std::string name) :
    as_(nh_, name, boost::bind(&Quadrotor::executeCB, this, _1), false),
    action_name_(name)
{
    odom_received = false;
    odom_2_received = false;
    trajectory_received = false;
    collision = false;
    pos_com_received = false;

    current_map = NULL;

    as_.start();

    base_sub = nh.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned",10,&Quadrotor::poseCallback,this);    

    position_command_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1000, &Quadrotor::planCallback,this);

    distance_pub = nh.advertise<std_msgs::Float64>("/compute_path/length",1);
    path_pub = nh.advertise<nav_msgs::Path>("/path",1);
    path_2_pub = nh.advertise<nav_msgs::Path>("/path_2",1);

    rrt_pub = nh.advertise<geometry_msgs::PoseArray>("/rrt_path", 1);
    uav_pause_pub = nh.advertise<std_msgs::Bool>("/drone_trajectory_paused", 1);
    
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

    traversing = false;

    previousX = 0.0;
    previousY = 0.0;
    previousZ = 0.0;

    previous2X = 0.0;
    previous2Y = 4.0;
    previous2Z = 0.0;

    path.header.frame_id = "world_enu";
    path.header.seq = 0;

    path2.header.frame_id = "world_enu";
    path2.header.seq = 0;

    collisionDistance = 0.7;

    collisionTime = 0.0;

    velocity = 1.0;

    pause_interval = 4;


}

void Quadrotor::moveFromObstacle(void)
{

}

void Quadrotor::executeCB(const airsim_moveit_navigation::AirSim_NavigationGoalConstPtr &goal)
{
    int timeOut = 0;
    while(traversing && timeOut < 50){
        ROS_INFO("Waiting for compute distance check to finish");
        timeOut++;
        ros::Duration(0.2).sleep();
    }
    traversing = true;
    feedback_.feedback_pose = odometry_information;
     
    geometry_msgs::Pose center;
    center.position.x = goal->goal_pose.orientation.x;
    center.position.y = goal->goal_pose.orientation.y;
    center.position.z = goal->goal_pose.orientation.z;

    geometry_msgs::Pose goal_ned_transformed = transformPose(goal->goal_pose, "world_ned", "world_enu");
    geometry_msgs::Pose center_transformed = transformPose(center, "world_ned", "world_enu");

    this->collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf] with orientation [%lf, %lf, %lf, %lf]",odometry_information.position.x,odometry_information.position.y,odometry_information.position.z,odometry_information.orientation.x,odometry_information.orientation.y,odometry_information.orientation.z,odometry_information.orientation.w);
    ROS_INFO("Try to go to [%lf,%lf,%lf] with orientation [%lf, %lf, %lf, %lf]",goal->goal_pose.position.x, goal->goal_pose.position.y, goal->goal_pose.position.z,goal->goal_pose.orientation.x,goal->goal_pose.orientation.y,goal->goal_pose.orientation.z,goal->goal_pose.orientation.w);

    geometry_msgs::PoseStamped ego_goal;

    ego_goal.pose.position.x = goal->goal_pose.position.x;
    ego_goal.pose.position.y = goal->goal_pose.position.y;
    ego_goal.pose.position.z = goal->goal_pose.position.z;

    goal_pub.publish(ego_goal);

    double goal_dist = sqrt(pow(goal->goal_pose.position.x - odometry_information.position.x,2) + pow(goal->goal_pose.position.y - odometry_information.position.y,2) + pow(goal->goal_pose.position.z - odometry_information.position.z,2));

    while(goal_dist > 0.7)
    {  

        if(as_.isPreemptRequested())
        {
            as_.setPreempted();
            ROS_INFO("Trajectory cancelled");
            break;
        }
        
        ROS_INFO("Check for collision");
        geometry_msgs::Pose collisionCheck;
        collisionCheck.position.x = goal_x;
        collisionCheck.position.y = goal_y;
        collisionCheck.position.z = goal_z;
        geometry_msgs::Pose collisionCheck_enu = transformPose(collisionCheck, "world_enu", "world_ned");
        /*
        if(checkCollision(collisionCheck_enu))
        {
            ROS_INFO("Current trajectory step will send drone into collision. Abort trajectory and plan for next frontier");
            break;
        }
        */
        double dist = sqrt(pow(collisionCheck_enu.position.x - odometry_information.position.x,2) + pow(collisionCheck_enu.position.y - odometry_information.position.y,2) + pow(collisionCheck_enu.position.z - odometry_information.position.z,2));
        double wait_Time = dist * velocity;
        wait_Time += 1.5;
        /*
        if(wait_Time > 10)
        {
            ROS_INFO("Wait time too high, most likely in collision.");
            break;
        }
        */
        ROS_INFO("Wait time: %f", wait_Time);
        client.moveToPositionAsync(goal_x, goal_y, goal_z, 1, 3);
        client.waitOnLastTask();
        //ros::Duration(wait_Time + 1.1).sleep();

        msr::airlib::CollisionInfo collision = client.simGetCollisionInfo();
        if(collision.has_collided)
        {
            ROS_INFO("Robot has collided");
            ROS_INFO("Previous Collision Time: %d", collisionTime);
            ROS_INFO("Current Collision Time: %d", collision.time_stamp);
            
            if(collision.time_stamp > collisionTime)
            {
                ROS_INFO("Robot just collided, correction position");
                collisionTime = collision.time_stamp;
                geometry_msgs::Pose collisionPosition;
                collisionPosition.position.x = collision.impact_point[0];
                collisionPosition.position.y = collision.impact_point[1];
                collisionPosition.position.z = collision.impact_point[2];

                geometry_msgs::Pose collisionTransformed = transformPose(collisionPosition, "world_enu", "world_ned");

                geometry_msgs::Pose freePosition;
                freePosition.position.x = (2 * odometry_information.position.x) - collisionTransformed.position.x;
                freePosition.position.y = (2 * odometry_information.position.y) - collisionTransformed.position.y;
                freePosition.position.z = (2 * odometry_information.position.z) - collisionTransformed.position.z;
                freePosition.position.x = (2 * freePosition.position.x) - collisionTransformed.position.x;
                freePosition.position.y = (2 * freePosition.position.y) - collisionTransformed.position.y;
                freePosition.position.z = (2 * freePosition.position.z) - collisionTransformed.position.z;
                geometry_msgs::Pose transformed = transformPose(freePosition, "world_ned", "world_enu");
                client.moveToPositionAsync(transformed.position.x, transformed.position.y, transformed.position.z, 1, 3);
                client.waitOnLastTask();
                //ros::Duration(2).sleep();
            }
        }

        double direction_y = center_transformed.position.y - goal_y;
        double direction_x = center_transformed.position.x - goal_x;
        double angle = atan2(direction_y, direction_x);

        angle = angle * 180 / M_PI;

        client.rotateToYawAsync(angle, 3);
        sleep(2);

       
        this->trajectory_received = false;
        this->odom_received = false;
        goal_dist = sqrt(pow(goal->goal_pose.position.x - odometry_information.position.x,2) + pow(goal->goal_pose.position.y - odometry_information.position.y,2) + pow(goal->goal_pose.position.z - odometry_information.position.z,2));
    }
    ROS_INFO("Trajectory is traversed");
    result_.result_pose = feedback_.feedback_pose;
    as_.setSucceeded(result_);

    traversing = false;
}

void Quadrotor::planCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
  //ROS_INFO("Got command");
  geometry_msgs::Pose pose_enu;
  pose_enu.position.x = msg->position.x;
  pose_enu.position.y = msg->position.y;
  pose_enu.position.z = msg->position.z;

  geometry_msgs::Pose transformed = transformPose(pose_enu, "world_ned", "world_enu");

  //if(pos_com_received == false)
  //{
  //ROS_INFO("Drone command (enu): [%.2f, %.2f, %.2f]", pose_enu.position.x, pose_enu.position.y, pose_enu.position.z);
  goal_x = transformed.position.x;
  goal_y = transformed.position.y;
  goal_z = transformed.position.z;

  //client.moveToPositionAsync(com_x, com_y, com_z, 1, 3);
  pos_com_received = true;
  //}
}

void Quadrotor::poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    odometry_information_ned.position.x = msg->pose.pose.position.x;
    odometry_information_ned.position.y = msg->pose.pose.position.y;
    odometry_information_ned.position.z = msg->pose.pose.position.z;
    odometry_information_ned.orientation.x = msg->pose.pose.orientation.x;
    odometry_information_ned.orientation.y = msg->pose.pose.orientation.y;
    odometry_information_ned.orientation.z = msg->pose.pose.orientation.z;
    odometry_information_ned.orientation.w = msg->pose.pose.orientation.w;

    odometry_information = transformPose(msg->pose.pose, "world_enu", "world_ned");
    odometry_information.orientation.x = msg->pose.pose.orientation.x;
    odometry_information.orientation.y = msg->pose.pose.orientation.y;
    odometry_information.orientation.z = msg->pose.pose.orientation.z;
    odometry_information.orientation.w = msg->pose.pose.orientation.w;
    odom_received = true;

    if(double_equals(previousX, odometry_information.position.x) &&
       double_equals(previousY, odometry_information.position.y) &&
       double_equals(previousZ, odometry_information.position.z))
        return;

    previousX = odometry_information.position.x;
    previousY = odometry_information.position.y;
    previousZ = odometry_information.position.z;

    geometry_msgs::PoseStamped pose;
    pose.pose = odometry_information;
    path.header.seq = path.header.seq + 1;
    pose.header.seq = path.header.seq;
    path.header.stamp = ros::Time::now();
    pose.header.stamp = path.header.stamp;
    path.poses.push_back(pose);

    path_pub.publish(path);
}


bool Quadrotor::checkCollision(geometry_msgs::Pose pose)
{
    /*
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
    ros::spinOnce();
    if(planning_scene_service.call(srv)){
        this->planning_scene->setPlanningSceneDiffMsg(srv.response.scene);
        octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
        
        delete current_map;
        current_map = (octomap::OcTree*)octomap_msgs::msgToMap(octomap);
        
        for(octomap::OcTree::leaf_iterator n = current_map->begin_leafs(current_map->getTreeDepth()); n != current_map->end_leafs(); ++n)
        {
            if(current_map->isNodeOccupied(*n))
            {
                double dist = sqrt(pow(pose.position.x - n.getX(),2) + pow(pose.position.y - n.getY(),2) + pow(pose.position.z - n.getZ(),2));
                if(dist <= collisionDistance)
                    return true;
            }
        }
        
        
    }
    return false;
    */
}

bool Quadrotor::double_equals(double a, double b)
{
    return (fabs(a - b)<EPSILON);
}

geometry_msgs::Pose Quadrotor::transformPose(geometry_msgs::Pose in, std::string target_frame, std::string source_frame)
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


void Quadrotor::takeoff()
{

    client.enableApiControl(true);
    client.armDisarm(true);
    
    client.takeoffAsync(5);
    client.waitOnLastTask();
    //ros::Duration(5.0).sleep();
    client.moveToPositionAsync(0, 0, -5, 1, 3);
    client.waitOnLastTask();
    //ros::Duration(4).sleep();
    client.rotateToYawAsync(-90, 3);
    client.waitOnLastTask();
    //ros::Duration(4).sleep();

    ROS_INFO("Takeoff successful");
}


void Quadrotor::run()
{
    ros::Rate rate(2);
    ros::AsyncSpinner spinner(4);
    int count = 1;

    while(ros::ok()){
        if(!odom_received){
          spinner.stop();
          rate.sleep();
        }
        spinner.start();

        // ros::spinOnce();
        rate.sleep();
    }
}
