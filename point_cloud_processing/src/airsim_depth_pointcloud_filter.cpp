#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>

class PointCloudFilter {
public:
    PointCloudFilter() : nh_("~") {
        // Read parameters
        nh_.param<std::string>("input_topic", input_topic_, "/lidar/point_cloud");
        nh_.param<std::string>("output_topic", output_topic_, "/filtered_point_cloud");
        nh_.param<std::string>("uav_moving_topic_", uav_moving_topic_, "/uav_moving");

        // Subscribe to the point cloud topic
        sub_ = nh_.subscribe(input_topic_, 1, &PointCloudFilter::pointCloudCallback, this);
        uav_moving_sub_ = nh_.subscribe(uav_moving_topic_, 1, &PointCloudFilter::uavMovingCallback, this);
        
        // Advertise the filtered point cloud
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

        uav_moving_ = false;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
        if(!uav_moving_)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*input_cloud_msg, *cloud);
            
            // Filter points within 1 meter radius of the origin
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.0, 1.0); // Limits z axis within 0 to 1 meter
            pass.setFilterLimitsNegative(true);
            pass.filter(*cloud);

            // Publish the filtered point cloud
            sensor_msgs::PointCloud2 output_cloud_msg;
            pcl::toROSMsg(*cloud, output_cloud_msg);
            output_cloud_msg.header = input_cloud_msg->header;
            pub_.publish(output_cloud_msg);
        }
    }

    void uavMovingCallback(const std_msgs::BoolConstPtr& bool_msg) {
        uav_moving_ = bool_msg->data;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Subscriber uav_moving_sub_;
    ros::Publisher pub_;
    std::string input_topic_;
    std::string output_topic_;
    std::string uav_moving_topic_;
    bool uav_moving_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_filter");
    PointCloudFilter pc_filter;
    ros::spin();
    return 0;
}