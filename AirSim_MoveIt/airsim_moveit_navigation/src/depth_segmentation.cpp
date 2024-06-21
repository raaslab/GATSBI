#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub_;

void callback(const sensor_msgs::ImageConstPtr& depth_data, const sensor_msgs::ImageConstPtr& segmentation_data) {
  ROS_INFO("Got images");

  sensor_msgs::Image depth_segmented = *depth_data;
  //depth_segmented.encoding = depth_data->encoding;
  int height = depth_data->height;
  int width = depth_data->width;
  int pixels = height * width;
  for(int i = 0; i < pixels; i++)
  {
    int b = segmentation_data->data[i*3];
    int g = segmentation_data->data[(i*3) + 1];
    int r = segmentation_data->data[(i*3) + 2];
    //if((b == 221 && g == 101 && r == 255) || (b == 156 && g == 70 && r == 187))
    if((r != 146) && (g != 52) && (b != 70))
    {
      depth_segmented.data[(i*4)] = 255;
      depth_segmented.data[(i*4) + 1] = 255;
      depth_segmented.data[(i*4) + 2] = 255;
      depth_segmented.data[(i*4) + 3] = 0;

    }
    /*else {
      depth_segmented.data[(i*4)] = 255;
      depth_segmented.data[(i*4) + 1] = 0;
      depth_segmented.data[(i*4) + 2] = 0;

    }
    */
  }
  pub_.publish(depth_segmented);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "depth_segmentation");

  ros::NodeHandle nh("~");
  std::string depth_segmented_topic;
  std::string depth_camera_topic;
  std::string segmented_camera_topic;
  nh.param<std::string>("depth_segmentation/depth_segmented_topic", depth_segmented_topic, "/DepthSegmented");
  nh.param<std::string>("depth_segmentation/depth_camera_topic", depth_camera_topic, "/DepthCamera");
  nh.param<std::string>("depth_segmentation/segmented_camera_topic", segmented_camera_topic, "/SegmentedCamera");

  pub_ = nh.advertise<sensor_msgs::Image>(depth_segmented_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_camera_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> segmentation_sub(nh, segmented_camera_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, segmentation_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();
  return 0;
}