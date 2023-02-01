#include <AirSim_Navigator.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "airsim_navigator");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    
    Quadrotor quad(std::ref(node_handle), "airsim_navigator");
    quad.takeoff();
    quad.run();
    return 0;
}