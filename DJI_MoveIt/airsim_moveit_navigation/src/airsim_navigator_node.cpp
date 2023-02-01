#include <AirSim_Navigator.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "dji_navigator");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    
    Quadrotor quad(std::ref(node_handle), "dji_navigator");
    /*
    bool obtain_control_result = quad.obtain_control();
    bool takeoff_result;
	if (!quad.set_local_position())
	{ 
		ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
		return 0;
	}

	if(quad.is_M100())
	{
		ROS_INFO("M100 taking off!");
		takeoff_result = quad.M100monitoredTakeoff();
	}
	else
	{
		ROS_INFO("A3/N3 taking off!");
		takeoff_result = quad.monitoredTakeoff();
	}

	if(!takeoff_result)
	{
		ROS_INFO("Bad takeoff");
		return 0;
	}
    */
    //quad.takeoff();
    quad.run();
    return 0;
}
