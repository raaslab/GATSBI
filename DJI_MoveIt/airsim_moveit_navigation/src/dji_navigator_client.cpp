#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <airsim_moveit_navigation/AirSim_NavigationAction.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "dji_navigator_client");

	ros::NodeHandle n("~");
	std::string filename = "-1";
	n.param<std::string>("path_file", filename, "-1");
	ROS_INFO("Path file: %s", filename.c_str());

	if(filename == "-1")
	{
		ROS_WARN("Need to set path filename in launch file, make sure to roslaunch and not rosrun");
		return -1;
	}
	actionlib::SimpleActionClient<airsim_moveit_navigation::AirSim_NavigationAction> ac("dji_navigator", true);
	
	std::ifstream myfile;
	myfile.open(filename);

	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");

	airsim_moveit_navigation::AirSim_NavigationGoal goal;

	std::string line;
	if(myfile.is_open())
	{
		std::getline(myfile, line);
		while(std::getline(myfile,line))
		{

			double x = 0;
			double y = 0;
			double z = 4;
			double yaw = 0;
			std::stringstream str_strm;
			str_strm << line;

			std::string temp_str;
			double temp_doub;
			int count = 0;
			while(!str_strm.eof())
			{
				str_strm >> temp_str;

				if(std::stringstream(temp_str) >> temp_doub)
				{

					if(count == 0)
						x = temp_doub;
					else if(count == 1)
						y = temp_doub;
					else if(count == 2)
						z = temp_doub;
					else if(count == 3)
						yaw = temp_doub;
					else
						ROS_WARN("Shouldn't get here in file parsing. Incorrect file format");

					count++;
				}
				temp_str = "";
			}
			
			ROS_INFO("Point [%f, %f, %f] with yaw [%f]", x, y, z, yaw);

			yaw = yaw * M_PI / 180;
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY(0, 0, yaw);

			goal.goal_pose.position.x = x;
			goal.goal_pose.position.y = y;
			goal.goal_pose.position.z = z;
			goal.goal_pose.orientation.x = myQuaternion.getX();
			goal.goal_pose.orientation.y = myQuaternion.getY();
			goal.goal_pose.orientation.z = myQuaternion.getZ();
			goal.goal_pose.orientation.w = myQuaternion.getW();

			ROS_INFO("Goal position [%f, %f, %f] with orientation [%f, %f, %f %f]", goal.goal_pose.position.x, goal.goal_pose.position.y, goal.goal_pose.position.z, goal.goal_pose.orientation.x, goal.goal_pose.orientation.y, goal.goal_pose.orientation.z, goal.goal_pose.orientation.w);
			
			
			ac.sendGoal(goal);

			bool finished_before_timeout = ac.waitForResult();

			if(finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Action finished: %s", state.toString().c_str());
			}
			else
				ROS_INFO("Action did not finish before the time out.");
			
		}
	
		myfile.close();
	}
	else
		ROS_WARN("Couldn't open path file.");
	
	return 0;
}
