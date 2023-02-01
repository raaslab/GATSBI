#include <DJI_Navigator.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;

Quadrotor::Quadrotor(ros::NodeHandle& nh, std::string name) :
    as_(nh_, name, boost::bind(&Quadrotor::executeCB, this, _1), false),
    action_name_(name)
{
    ROS_INFO("Starting constructor");
    odom_received = false;
    trajectory_received = false;
    collision = false;

    current_map = NULL;

    first_flight = true;

    flight_step = 1;

    as_.start();

    attitudeSub = nh.subscribe("/dji_sdk/attitude", 10, &Quadrotor::attitude_callback,this);
    gpsSub = nh.subscribe("/dji_sdk/gps_position", 10, &Quadrotor::gps_callback,this);
    flightStatusSub = nh.subscribe("/dji_sdk/flight_status", 10, &Quadrotor::flight_status_callback,this);
    displayModeSub = nh.subscribe("/dji_sdk/display_mode", 10, &Quadrotor::display_mode_callback,this);
    localPosition = nh.subscribe("/dji_sdk/local_position", 10, &Quadrotor::local_position_callback,this);

    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

    ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);

// Basic services
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("/dji_sdk/query_drone_version");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("/dji_sdk/set_local_pos_ref");

    //base_sub = nh.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned",10,&Quadrotor::poseCallback,this);
    plan_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,&Quadrotor::planCallback,this);
    distance_sub = nh.subscribe<geometry_msgs::Point>("/compute_path/point",1,&Quadrotor::computePathLengthCB,this);

    distance_pub = nh.advertise<std_msgs::Float64>("/compute_path/length",1);
    path_pub = nh.advertise<nav_msgs::Path>("/path",1);

    ROS_INFO("Initiating moveit");
    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
    ROS_INFO("Loading model into moveit");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();

    ROS_INFO("Got model");
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

    ROS_INFO("Done with constructor");
}

void Quadrotor::moveFromObstacle(void)
{

}

bool Quadrotor::obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool Quadrotor::set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

void Quadrotor::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void Quadrotor::local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
  odom_received = true;
}

void Quadrotor::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  //if(elapsed_time > ros::Duration(0.02))
  //{
    //start_time = ros::Time::now();

    //if(mission.state == 1 && !mission.finished) {
      //square_mission.step();
    //}
  //}
}

void Quadrotor::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void Quadrotor::display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
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

	if(first_flight)
	{
		first_flight = false;
		ROS_INFO("Go up 2 meters");
		mission.reset();
		mission.start_gps_location = current_gps;
		mission.start_local_position = current_local_pos;

		ROS_INFO("Sending dji [0, 0, 2] with yaw 90");
		mission.setTarget(0, 0, 2.5, 0);
		mission.state = flight_step;
		flight_step++;
		mission.step();

		while(!mission.finished)
		{
			ros::Duration(0.02).sleep();
			mission.step();
		}
			
	}
	geometry_msgs::Pose current_pose;
	current_pose.position.x = current_local_pos.x;
	current_pose.position.y = current_local_pos.y;
	current_pose.position.z = current_local_pos.z;
	current_pose.orientation = current_atti;

	//geometry_msgs::Pose current_pose_transformed = transformPose(current_pose, "world", "dji");
	geometry_msgs::Pose current_pose_transformed = current_pose;
	current_pose_transformed.orientation = current_atti;

    feedback_.feedback_pose = current_pose_transformed;

    std::vector<double> target(7);
    target[0] = goal->goal_pose.position.x;
    target[1] = goal->goal_pose.position.y;
    target[2] = goal->goal_pose.position.z;
    target[3] = goal->goal_pose.orientation.x;
    target[4] = goal->goal_pose.orientation.y;
    target[5] = goal->goal_pose.orientation.z;
    target[6] = goal->goal_pose.orientation.w;

    std::vector<double> start_state_(7);
    start_state_[0] = current_pose_transformed.position.x;
    start_state_[1] = current_pose_transformed.position.y;
    start_state_[2] = current_pose_transformed.position.z;
    start_state_[3] = current_pose_transformed.orientation.x;
    start_state_[4] = current_pose_transformed.orientation.y;
    start_state_[5] = current_pose_transformed.orientation.z;
    start_state_[6] = current_pose_transformed.orientation.w;

    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    this->collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf] with orienation [%lf, %lf, %lf, %lf]",current_pose_transformed.position.x,current_pose_transformed.position.y,current_pose_transformed.position.z,current_pose_transformed.orientation.x,current_pose_transformed.orientation.y,current_pose_transformed.orientation.z,current_pose_transformed.orientation.w);
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
            //ROS_INFO("Waiting for trajectory");
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
           /* if(checkCollision(trajectory[i]))
            {
                ROS_INFO("Current trajectory step will send drone into collision. Abort trajectory and plan for next frontier");
                break;
            }
	  */		
			mission.reset();
			mission.start_gps_location = current_gps;
			mission.start_local_position = current_local_pos;

			geometry_msgs::Pose dji_transformed_test = transformPose(trajectory[i], "dji", "world");
			
			geometry_msgs::Pose dji_transformed = trajectory[i];
			dji_transformed.position.x = dji_transformed.position.x - current_local_pos.x;
			dji_transformed.position.y = dji_transformed.position.y - current_local_pos.y;
			dji_transformed.position.z = dji_transformed.position.z - current_local_pos.z;

			ROS_INFO("Trajectory position (%f, %f, %f)\n", trajectory[i].position.x, trajectory[i].position.y, trajectory[i].position.z);
			ROS_INFO("Trajectory after manual transform position (%f, %f, %f)\n", dji_transformed.position.x, dji_transformed.position.y, dji_transformed.position.z);
			ROS_INFO("Trajectory after auto transform position (%f, %f, %f)\n", dji_transformed_test.position.x, dji_transformed_test.position.y, dji_transformed_test.position.z);
			double traj_yaw = tf::getYaw(goal->goal_pose.orientation);
			traj_yaw = traj_yaw * 180/ 3.14159;
			if(i <= trajectory.size() -2)
			{
				traj_yaw = 0;
			}			
			ROS_INFO("Trajectory target yaw: %f\n", tf::getYaw(trajectory[i].orientation));
			ROS_INFO("Trajectory target yaw (degrees): %f\n", traj_yaw);
			ROS_INFO("Sending dji [%f, %f, %f] with yaw %f", dji_transformed.position.x, dji_transformed.position.y, dji_transformed.position.z, traj_yaw);
			mission.setTarget(dji_transformed.position.x, dji_transformed.position.y, dji_transformed.position.z, traj_yaw);
			mission.state = flight_step;
			flight_step++;
			mission.step();
			//ROS_INFO("##### Start route %d ....", mission.state);

			while(!mission.finished)
			{
				ros::Duration(0.02).sleep();
				mission.step();
			}
	    //Add DJI code here            
    
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

void Quadrotor::planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    ROS_INFO("In plan callback");
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
    ROS_INFO("Got trajectory");
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

void Quadrotor::computePathLengthCB(const geometry_msgs::Point::ConstPtr &point)
{
    int timeOut = 0;
    while(traversing && timeOut < 50){
        ROS_INFO("Waiting for drone traversal to finish before checking distance");
        timeOut++;
        ros::Duration(0.2).sleep();
    }
    traversing = true;

	geometry_msgs::Pose current_pose;
	current_pose.position.x = current_local_pos.x;
	current_pose.position.y = current_local_pos.y;
	current_pose.position.z = current_local_pos.z;
	current_pose.orientation = current_atti;

	geometry_msgs::Pose current_pose_transformed = transformPose(current_pose, "world", "dji");
	current_pose_transformed.orientation = current_atti;

    ROS_INFO("Computing MoveIt distance from (%f,%f,%f) to (%f,%f,%f)", current_pose_transformed.position.x, current_pose_transformed.position.y, current_pose_transformed.position.z,
                                                                        point->x, point->y, point->z);
    std::vector<double> target(7);
    target[0] = point->x;
    target[1] = point->y;
    target[2] = point->z;
    target[3] = current_pose_transformed.orientation.x;
    target[4] = current_pose_transformed.orientation.y;
    target[5] = current_pose_transformed.orientation.z;
    target[6] = current_pose_transformed.orientation.w;

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
    start_state_[0] = current_pose_transformed.position.x;
    start_state_[1] = current_pose_transformed.position.y;
    start_state_[2] = current_pose_transformed.position.z;
    start_state_[3] = current_pose_transformed.orientation.x;
    start_state_[4] = current_pose_transformed.orientation.y;
    start_state_[5] = current_pose_transformed.orientation.z;
    start_state_[6] = current_pose_transformed.orientation.w;

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

bool Quadrotor::is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

bool Quadrotor::monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
     
    /* 
    mission.state = 1;
    mission.reset();
    mission.start_gps_location = current_gps;
    mission.start_local_position = current_local_pos;
    mission.setTarget(0, 0, 2, 10);
    mission.step();

    while(!mission.finished)
    {
        ros::Duration(0.02).sleep();
        mission.step();
    }
    ROS_INFO("Finished going up 2 meters");
    */
    
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool Quadrotor::M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool Quadrotor::takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

void Quadrotor::takeoff()
{

    // Add takeoff code here

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

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    //ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    //ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_local_position.z + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 50)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.5 &&
      std::abs(yOffsetRemaining) < 0.5 &&
      std::abs(zOffsetRemaining) < 0.5 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 50)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }

}
