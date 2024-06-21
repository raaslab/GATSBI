#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    // Convert the message to an octomap::AbstractOcTree
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);

    // Cast to an octomap::OcTree
    if (abstract_tree) {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        if (octree) {
            ROS_INFO("Successfully converted Octomap message to OcTree.");

            // Iterate over each node in the OcTree
            for (octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it) {
                double occupancy = it->getOccupancy();
                octomap::point3d coord = it.getCoordinate();
                ROS_INFO("Node at (%f, %f, %f) with occupancy %f", coord.x(), coord.y(), coord.z(), occupancy);
            }
        } else {
            ROS_ERROR("Error casting to OcTree.");
        }
    } else {
        ROS_ERROR("Error converting Octomap message to AbstractOcTree.");
    }
    delete abstract_tree; // Don't forget to free the memory
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/octomap_full", 1, octomapCallback);

    ros::spin();
    return 0;
}
