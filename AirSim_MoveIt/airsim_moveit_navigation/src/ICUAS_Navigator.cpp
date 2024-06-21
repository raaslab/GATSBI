#include <ICUAS_Navigator.h>

Quadrotor::Quadrotor(ros::NodeHandle& nh, std::string name) :
    as_(nh_, name, boost::bind(&Quadrotor::executeCB, this, _1), false),
    action_name_(name)
{
    odom_received = false;
    trajectory_received = false;
    collision = false;

    current_map = NULL;

    as_.start();

    base_sub = nh.subscribe<geometry_msgs::PoseStamped>("/red/pose",10,&Quadrotor::poseCallback,this);
    plan_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,&Quadrotor::planCallback,this);
    distance_sub = nh.subscribe<geometry_msgs::Point>("/compute_path/point",1,&Quadrotor::computePathLengthCB,this);
    path_sub = nh.subscribe<geometry_msgs::Point>("/compute_rrt_path",1,&Quadrotor::computeRRTPathCB,this);
    nbv_path_sub = nh.subscribe<geometry_msgs::Point>("/compute_nbv_path",1,&Quadrotor::computeNBVPathCB,this);

    distance_pub = nh.advertise<std_msgs::Float64>("/compute_path/length",1);
    icuas_pub = nh.advertise<geometry_msgs::PoseStamped>("/red/tracker/input_pose",1);
    path_pub = nh.advertise<nav_msgs::Path>("/path",1);
    rrt_pub = nh.advertise<geometry_msgs::PoseArray>("/rrt_path", 1);

    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();


    planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

    move_group->setPlannerId("RRTConnectkConfigDefault");
    move_group->setNumPlanningAttempts(10);
    move_group->setPlanningTime(10);
    move_group->setWorkspace(XMIN,YMIN,ZMIN,XMAX,YMAX,ZMAX);

    start_state.reset(new robot_state::RobotState(move_group->getRobotModel()));
    planning_scene.reset(new planning_scene::PlanningScene(kmodel));
    
    traversing = false;

    previousX = 0.0;
    previousY = 0.0;
    previousZ = 0.0;

    path.header.frame_id = "world";
    path.header.seq = 0;

    collisionDistance = 0.7;

    collisionTime = 0.0;

    velocity = 1.0;
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

    std::vector<double> target(7);
    target[0] = goal->goal_pose.position.x;
    target[1] = goal->goal_pose.position.y;
    target[2] = goal->goal_pose.position.z;
    target[3] = goal->goal_pose.orientation.x;
    target[4] = goal->goal_pose.orientation.y;
    target[5] = goal->goal_pose.orientation.z;
    target[6] = goal->goal_pose.orientation.w;

    std::vector<double> start_state_(7);
    start_state_[0] = odometry_information.position.x;
    start_state_[1] = odometry_information.position.y;
    start_state_[2] = odometry_information.position.z;
    //start_state_[0] = 10;
    //start_state_[1] = 2;
    //start_state_[2] = 0.5;
    start_state_[3] = odometry_information.orientation.x;
    start_state_[4] = odometry_information.orientation.y;
    start_state_[5] = odometry_information.orientation.z;
    start_state_[6] = odometry_information.orientation.w;

    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    this->collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf] with orienation [%lf, %lf, %lf, %lf]",odometry_information.position.x,odometry_information.position.y,odometry_information.position.z,odometry_information.orientation.x,odometry_information.orientation.y,odometry_information.orientation.z,odometry_information.orientation.w);
    ROS_INFO("Try to go to [%lf,%lf,%lf] with orienation [%lf, %lf, %lf, %lf]",goal->goal_pose.position.x,goal->goal_pose.position.y,goal->goal_pose.position.z,goal->goal_pose.orientation.x,goal->goal_pose.orientation.y,goal->goal_pose.orientation.z,goal->goal_pose.orientation.w);
    this->start_state->setVariablePositions(start_state_);
    this->move_group->setStartState(*start_state);

    moveit::planning_interface::MoveItErrorCode moveiterrorcode = move_group->plan(plan);
    this->isPathValid = (moveiterrorcode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //if(!this->isPathValid)
    //{
    //    ROS_INFO("Invalid Moveit Error Code: %d", moveiterrorcode.val);
    //    ROS_INFO("Attempting to move away from collision");
    //    moveFromObstacle();
    //}

    if(this->isPathValid){

        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            ros::Duration(0.2).sleep();
        }

        ROS_INFO("Moving through moveit planned trajectory");
        for(int i=1;i<trajectory.size();i++){
            if(as_.isPreemptRequested())
            {
                as_.setPreempted();
                ROS_INFO("Trajectory cancelled");
                break;
            }
            ROS_INFO("Trajectory step: [%d]", i);
            ROS_INFO("Check for collision");
            if(checkCollision(trajectory[i]))
            {
                ROS_INFO("Current trajectory step will send drone into collision. Abort trajectory and plan for next frontier");
                break;
            }
            double dist = sqrt(pow(trajectory[i].position.x - odometry_information.position.x,2) + pow(trajectory[i].position.y - odometry_information.position.y,2) + pow(trajectory[i].position.z - odometry_information.position.z,2));
            double wait_Time = dist * velocity; 
            wait_Time += 1.5;
            if(wait_Time > 10)
            {
                ROS_INFO("Wait time too high, most likely in collision.");
                break;
            }
            ROS_INFO("Wait time: %f", wait_Time);

            geometry_msgs::PoseStamped output;
            output.pose = trajectory[i];
            output.header.stamp = ros::Time::now();
            output.header.frame_id = "world"; 
            icuas_pub.publish(output);
            ros::Duration(wait_Time + 0.1).sleep();

        }



        ROS_INFO("Trajectory is traversed");
        this->trajectory_received = false;
        this->odom_received = false;
    }
    else
    {
        ROS_INFO("Moveit Error Code: %d", moveiterrorcode.val);
    }
    result_.result_pose = feedback_.feedback_pose;
    as_.setSucceeded(result_);

    traversing = false;
    //return this->isPathValid;
}

void Quadrotor::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{

    odometry_information.position.x = msg->pose.position.x;
    odometry_information.position.y = msg->pose.position.y;
    odometry_information.position.z = msg->pose.position.z;
    odometry_information.orientation.x = msg->pose.orientation.x;
    odometry_information.orientation.y = msg->pose.orientation.y;
    odometry_information.orientation.z = msg->pose.orientation.z;
    odometry_information.orientation.w = msg->pose.orientation.w;
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

void Quadrotor::planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    if(!odom_received) return;
    trajectory.clear();
    for(auto robot_traj: msg->trajectory){
        for(auto point : robot_traj.multi_dof_joint_trajectory.points){
            geometry_msgs::Pose waypoint;
            waypoint.position.x = point.transforms[0].translation.x;
            waypoint.position.y = point.transforms[0].translation.y;
            waypoint.position.z = point.transforms[0].translation.z;

            waypoint.orientation.x = point.transforms[0].rotation.x;
            waypoint.orientation.y = point.transforms[0].rotation.y;
            waypoint.orientation.z = point.transforms[0].rotation.z;
            waypoint.orientation.w = point.transforms[0].rotation.w;

            trajectory.push_back(waypoint);
        }
    }
    trajectory_received = true;
}

bool Quadrotor::checkCollision(geometry_msgs::Pose pose)
{
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
}


void Quadrotor::computeRRTPathCB(const geometry_msgs::Point::ConstPtr &point)
{
    int timeOut = 0;
    while(traversing && timeOut < 50){
        ROS_INFO("Waiting for drone traversal to finish before checking distance");
        timeOut++;
        ros::Duration(0.2).sleep();
    }
    traversing = true;
    this->trajectory_received = false;

    ROS_INFO("Computing MoveIt distance from (%f,%f,%f) to (%f,%f,%f)", odometry_information.position.x, odometry_information.position.y, odometry_information.position.z,
                                                                        point->x, point->y, point->z);
    std::vector<double> target(7);
    target[0] = point->x;
    target[1] = point->y;
    target[2] = point->z;
    target[3] = odometry_information.orientation.x;
    target[4] = odometry_information.orientation.y;
    target[5] = odometry_information.orientation.z;
    target[6] = odometry_information.orientation.w;

    std::vector<double> start_state_(7);
    start_state_[0] = odometry_information.position.x;
    start_state_[1] = odometry_information.position.y;
    start_state_[2] = odometry_information.position.z;
    start_state_[3] = odometry_information.orientation.x;
    start_state_[4] = odometry_information.orientation.y;
    start_state_[5] = odometry_information.orientation.z;
    start_state_[6] = odometry_information.orientation.w;

    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    this->collision = false;
    this->start_state->setVariablePositions(start_state_);
    this->move_group->setStartState(*start_state);

    moveit::planning_interface::MoveItErrorCode moveiterrorcode = move_group->plan(plan);
    this->isPathValid = (moveiterrorcode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Moveit Error Code: %d", moveiterrorcode.val);
    if(this->isPathValid){
        ROS_INFO("Path is valid");
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            //ros::Duration(0.2).sleep();
        }
        geometry_msgs::PoseArray rrt_path;
        rrt_path.header.stamp = ros::Time::now();
        rrt_path.header.frame_id = "world";
        for(int i=0;i<trajectory.size();i++){
            ROS_INFO("Trajectory #:%d, Point: (%f, %f, %f)", i+1, trajectory[i].position.x, trajectory[i].position.y, trajectory[i].position.z);
            geometry_msgs::Pose p;
            p.position.x = trajectory[i].position.x;
            p.position.y = trajectory[i].position.y;
            p.position.z = trajectory[i].position.z;

            rrt_path.poses.push_back(p);
        }
        rrt_pub.publish(rrt_path);
    }
    else
    {
        ROS_INFO("Path is NOT valid");
        ROS_INFO("Moveit Error Code: %d", moveiterrorcode.val);
        geometry_msgs::PoseArray rrt_path;
        rrt_path.header.stamp = ros::Time::now();
        rrt_path.header.frame_id = "world";
        geometry_msgs::Pose p;
        p.position.x = -99;
        p.position.y = -99;
        p.position.z = -99;
        rrt_path.poses.push_back(p);
        rrt_pub.publish(rrt_path);
        ROS_INFO("Sending pose of (-99, -99, -99) (invalid)");
    }

    traversing = false;
}

void Quadrotor::computeNBVPathCB(const geometry_msgs::Point::ConstPtr &point)
{
    int timeOut = 0;
    while(traversing && timeOut < 50){
        ROS_INFO("Waiting for drone traversal to finish before checking distance");
        timeOut++;
        ros::Duration(0.2).sleep();
    }
    traversing = true;
    this->trajectory_received = false;

    geometry_msgs::Pose input_pose;
    input_pose.position.x = point->x;
    input_pose.position.y = point->y;
    input_pose.position.z = point->z;

    ROS_INFO("Computing MoveIt distance from (%f,%f,%f) to (%f,%f,%f)", odometry_information.position.x, odometry_information.position.y, odometry_information.position.z,
                                                                        input_pose.position.x, input_pose.position.y, input_pose.position.z);
    std::vector<double> target(7);
    target[0] = input_pose.position.x;
    target[1] = input_pose.position.y;
    target[2] = input_pose.position.z;
    target[3] = odometry_information.orientation.x;
    target[4] = odometry_information.orientation.y;
    target[5] = odometry_information.orientation.z;
    target[6] = odometry_information.orientation.w;

    std::vector<double> start_state_(7);
    start_state_[0] = odometry_information.position.x;
    start_state_[1] = odometry_information.position.y;
    start_state_[2] = odometry_information.position.z;
    start_state_[3] = odometry_information.orientation.x;
    start_state_[4] = odometry_information.orientation.y;
    start_state_[5] = odometry_information.orientation.z;
    start_state_[6] = odometry_information.orientation.w;

    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    this->collision = false;
    this->start_state->setVariablePositions(start_state_);
    this->move_group->setStartState(*start_state);

    moveit::planning_interface::MoveItErrorCode moveiterrorcode = move_group->plan(plan);
    this->isPathValid = (moveiterrorcode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Moveit Error Code: %d", moveiterrorcode.val);
    if(this->isPathValid){
        ROS_INFO("Path is valid");
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            //ros::Duration(0.2).sleep();
        }
        geometry_msgs::PoseArray rrt_path;
        rrt_path.header.stamp = ros::Time::now();
        rrt_path.header.frame_id = "world";
        for(int i=0;i<trajectory.size();i++){
            ROS_INFO("Trajectory #:%d, Point: (%f, %f, %f)", i+1, trajectory[i].position.x, trajectory[i].position.y, trajectory[i].position.z);
            geometry_msgs::Pose p;
            p.position.x = trajectory[i].position.x;
            p.position.y = trajectory[i].position.y;
            p.position.z = trajectory[i].position.z;

            rrt_path.poses.push_back(p);
        }
        rrt_pub.publish(rrt_path);
    }
    else
    {
        ROS_INFO("Path is NOT valid");
        ROS_INFO("Moveit Error Code: %d", moveiterrorcode.val);
        geometry_msgs::PoseArray rrt_path;
        rrt_path.header.stamp = ros::Time::now();
        rrt_path.header.frame_id = "world";
        geometry_msgs::Pose p;
        p.position.x = -99;
        p.position.y = -99;
        p.position.z = -99;
        rrt_path.poses.push_back(p);
        rrt_pub.publish(rrt_path);
        ROS_INFO("Sending pose of (-99, -99, -99) (invalid)");
    }

    traversing = false;
}

void Quadrotor::computePathLengthCB(const geometry_msgs::Point::ConstPtr &point)
{
    int timeOut = 0;
    while(traversing && timeOut < 50){
        ROS_INFO("Waiting for drone traversal to finish before checking distance");
        timeOut++;
        ros::Duration(0.2).sleep();
    }
    traversing = true;

    ROS_INFO("Computing MoveIt distance from (%f,%f,%f) to (%f,%f,%f)", odometry_information.position.x, odometry_information.position.y, odometry_information.position.z,
                                                                        point->x, point->y, point->z);
    std::vector<double> target(7);
    target[0] = point->x;
    target[1] = point->y;
    target[2] = point->z;
    target[3] = odometry_information.orientation.x;
    target[4] = odometry_information.orientation.y;
    target[5] = odometry_information.orientation.z;
    target[6] = odometry_information.orientation.w;

    std::vector<double> start_state_(7);
    /*
    start_state_[0] = path->start.x;
    start_state_[1] = path->start.y;
    start_state_[2] = path->start.z;
    start_state_[3] = 0;
    start_state_[4] = 0;
    start_state_[5] = 0;
    start_state_[6] = 1;
    */
    start_state_[0] = odometry_information.position.x;
    start_state_[1] = odometry_information.position.y;
    start_state_[2] = odometry_information.position.z;
    start_state_[3] = odometry_information.orientation.x;
    start_state_[4] = odometry_information.orientation.y;
    start_state_[5] = odometry_information.orientation.z;
    start_state_[6] = odometry_information.orientation.w;

    std_msgs::Float64 distance;
    distance.data = 0.0;

    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    this->collision = false;
    this->start_state->setVariablePositions(start_state_);
    this->move_group->setStartState(*start_state);
    ROS_INFO("After computer distance");

    moveit::planning_interface::MoveItErrorCode moveiterrorcode = move_group->plan(plan);
    this->isPathValid = (moveiterrorcode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Moveit Error Code: %d", moveiterrorcode.val);
    if(this->isPathValid){
        ROS_INFO("Path is valid");
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            //ros::Duration(0.2).sleep();
        }

        for(int i=1;i<trajectory.size();i++){
            ROS_INFO("Trajectory #:%d, Point: (%f, %f, %f)", i+1, trajectory[i].position.x, trajectory[i].position.y, trajectory[i].position.z);
            double temp_distance = (trajectory[i].position.x - trajectory[i-1].position.x)*(trajectory[i].position.x - trajectory[i-1].position.x) +
                                   (trajectory[i].position.y - trajectory[i-1].position.y)*(trajectory[i].position.y - trajectory[i-1].position.y) +
                                   (trajectory[i].position.z - trajectory[i-1].position.z)*(trajectory[i].position.z - trajectory[i-1].position.z);
            temp_distance = sqrt(temp_distance);

            distance.data += temp_distance;
            ROS_INFO("Step Distance: %f", temp_distance);
            ROS_INFO("Running Distance: %f", distance.data);
        }
        distance_pub.publish(distance);
        ROS_INFO("MoveIt distance is %f", distance.data);
    }
    else
    {
    	ROS_INFO("Path is NOT valid");
    	ROS_INFO("Moveit Error Code: %d", moveiterrorcode.val);
    	distance.data = -1;
    	distance_pub.publish(distance);
    	ROS_INFO("Sending distance of -1 (invalid)");
    }
    traversing = false;
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

    ROS_INFO("Takeoff successful");
}


void Quadrotor::run()
{
    ros::AsyncSpinner spinner(4);
    if(ros::ok()){
        spinner.start();
    ros::waitForShutdown();
    }
}
