#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

class OctomapSubscriber {
public:
    OctomapSubscriber() {
        octomap_subscriber_ = nh_.subscribe("/octomap_full", 1, &OctomapSubscriber::octomapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/free_cells_markers", 10);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg) {
        octomap::OcTree* octree = new octomap::OcTree(resolution);
        octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*octomap_msg);
        octree = dynamic_cast<octomap::OcTree*>(tree);

        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        octomap::OcTreeNode* result;
        //octomap::OcTree::leaf_iterator it;
        //octomap::OcTree::leaf_iterator endLeaf;

        for (octomap::OcTree::iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
            if(!octree->isNodeOccupied(*it))
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = octomap_msg->header.frame_id;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = it.getX();
                marker.pose.position.y = it.getY();
                marker.pose.position.z = it.getZ();
                marker.scale.x = octree->getResolution();
                marker.scale.y = octree->getResolution();
                marker.scale.z = octree->getResolution();
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.id = id++;
                marker_array.markers.push_back(marker);
            }   
        }
        delete octree;
        marker_pub_.publish(marker_array);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_subscriber_;
    ros::Publisher marker_pub_;
    double resolution = 1;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_subscriber");
    OctomapSubscriber octomap_subscriber;
    ros::spin();
    return 0;
}
