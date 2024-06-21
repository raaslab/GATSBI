#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

ros::Publisher merged_pub;
PointCloud::Ptr cloud1(new PointCloud);
PointCloud::Ptr cloud2(new PointCloud);
boost::shared_ptr<tf2_ros::Buffer> tf_buffer;
boost::shared_ptr<tf2_ros::TransformListener> tf_listener;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg1, const sensor_msgs::PointCloud2ConstPtr& msg2)
{
  pcl::fromROSMsg(*msg1, *cloud1);
  pcl::fromROSMsg(*msg2, *cloud2);

  PointCloud::Ptr merged_cloud(new PointCloud);
  sensor_msgs::PointCloud2 merged_msg;

  if (cloud1->empty())
  {

    *merged_cloud = *cloud2;
    pcl::toROSMsg(*merged_cloud, merged_msg);
    merged_msg.header = msg2->header;

    merged_pub.publish(merged_msg);
    return;
  }
  else if(cloud2->empty())
  {
    *merged_cloud = *cloud1;
    pcl::toROSMsg(*merged_cloud, merged_msg);
    merged_msg.header = msg1->header;

    merged_pub.publish(merged_msg);
  }
  else if(cloud1->empty() && cloud2->empty()) {
    ROS_WARN("Both clouds empty.");
    return;
  }
  else
  {
  //ROS_INFO("Got pointclouds");

    try
    {
      geometry_msgs::TransformStamped transform;
      transform = tf_buffer->lookupTransform("world_enu", msg1->header.frame_id, msg1->header.stamp);
      pcl_ros::transformPointCloud(*cloud1, *cloud1, transform.transform);
      transform = tf_buffer->lookupTransform("world_enu", msg2->header.frame_id, msg2->header.stamp);
      pcl_ros::transformPointCloud(*cloud2, *cloud2, transform.transform);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Failed to transform point clouds: " << ex.what());
      return;
    }

    
    *merged_cloud = *cloud1 + *cloud2;

    pcl::toROSMsg(*merged_cloud, merged_msg);
    merged_msg.header.frame_id = "world_enu";

    merged_pub.publish(merged_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_merge");
  ros::NodeHandle nh("~");

  tf_buffer.reset(new tf2_ros::Buffer);
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));

  std::string topic1, topic2, combined_topic;
  nh.param<std::string>("topic1", topic1, "/point_cloud1");
  nh.param<std::string>("topic2", topic2, "/point_cloud2");
  nh.param<std::string>("combined_topic", combined_topic, "/merged_point_cloud");

  ROS_INFO("First pointcloud topic: %s", topic1.c_str());
  ROS_INFO("Second pointcloud topic: %s", topic2.c_str());
  ROS_INFO("Second pointcloud topic: %s", topic2.c_str());

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, topic1, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, topic2, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub1, sub2);
  sync.registerCallback(boost::bind(&pointCloudCallback, _1, _2));

  merged_pub = nh.advertise<sensor_msgs::PointCloud2>(combined_topic.c_str(), 1);

  ros::spin();

  return 0;
}