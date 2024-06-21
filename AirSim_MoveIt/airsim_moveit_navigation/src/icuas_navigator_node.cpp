#include <ICUAS_Navigator.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "icuas_navigator");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    
    Quadrotor quad(std::ref(node_handle), "icuas_navigator");
    quad.run();
    return 0;
}