#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <iostream>

ros::Publisher pub;
static int counter = 0;

void sync_callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::PointCloud2ConstPtr &input_cloud, const sensor_msgs::CameraInfoConstPtr &cam_info){

        // Get the image in cv::Mat format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); // CHANGED HERE
            cv_ptr = cv_bridge::toCvCopy(image);
            // cv_ptr->image.convertTo(cv_ptr->image, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image_cv_mat = cv_ptr->image;
        // Get the transformation between velodyne frame and image frame
        pcl::PCLPointCloud2 pcl_input;
        pcl_conversions::toPCL(*input_cloud, pcl_input);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::fromPCLPointCloud2(pcl_input, *temp_cloud);

        tf::TransformListener tf_listener;
        tf::StampedTransform transformer;
        // Eigen::Affine3d trans_eigen;
        tf::Vector3 origin;
        tf::Quaternion rot;
        tf_listener.waitForTransform("camera_rgb_optical_frame", "laser0_frame",ros::Time(0), ros::Duration(15.0));
        try {
            tf_listener.lookupTransform("camera_rgb_optical_frame", "laser0_frame",ros::Time(0), transformer);
        }
        catch(tf::TransformException exception){
            ROS_ERROR("%s", exception.what());
            return;
        }


        origin = transformer.getOrigin();
        rot = transformer.getRotation();
        rot.normalize();
        Eigen::Affine3d transf_mat;
        //pcl_ros::transformAsMatrix(transformer, transf_mat); // CHANGED HERE
        tf::transformTFToEigen(transformer, transf_mat);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::transformPointCloud(*temp_cloud, *transform_cloud, transf_mat);
        image_geometry::PinholeCameraModel cam_model_;
        cam_model_.fromCameraInfo(cam_info);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Apply HSV filters
        cv::Mat g_thresh, b_thresh, frame_HSV;
        cv::cvtColor(image_cv_mat, frame_HSV, cv::COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        // cv::inRange(frame_HSV, cv::Scalar(24, 39, 0), cv::Scalar(123, 255, 239), g_thresh);
        // cv::inRange(frame_HSV, cv::Scalar(0, 0, 0), cv::Scalar(24, 74, 169), b_thresh);

        // Only bridge
	cv::inRange(frame_HSV, cv::Scalar(0,0,0), cv::Scalar(180,30,156), g_thresh);
        //cv::inRange(frame_HSV, cv::Scalar(5,5,5), cv::Scalar(255,255,255), g_thresh);
        //Wall Only
        // cv::inRange(frame_HSV, cv::Scalar(0, 0, 118), cv::Scalar(176, 241, 242), g_thresh);

        cv::Mat elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::morphologyEx(b_thresh, b_thresh, cv::MORPH_CLOSE, elem);
        cv::morphologyEx(g_thresh, g_thresh, cv::MORPH_CLOSE, elem);

        cv::bitwise_not(g_thresh, b_thresh);
        cv::morphologyEx(b_thresh, b_thresh, cv::MORPH_CLOSE, elem);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        // pcl::ExtractIndices:: extract = new pcl::ExtractIndices<pcl::PointXYZI>();
        std::cout << (cam_info->K[0]) << " " << (cam_info->K[1]) << " " << (cam_info->K[2]) << "\n";
        for(int i = 0; i < transform_cloud->points.size(); i++){
            cv::Point3d pt(transform_cloud->points[i].x, transform_cloud->points[i].y, transform_cloud->points[i].z);
            cv::Point2f uv;
            uv = cam_model_.project3dToPixel(pt);
            if(uv.x > 0 && uv.x < cam_info->width && uv.y > 0 && uv.y < cam_info->height && transform_cloud->points[i].z > 0){
                if(((int(b_thresh.at<uchar>(uv.y, uv.x)) < 200))){
                    inliers->indices.push_back(i);
                    temp_cloud->points[i].r = '0' + image_cv_mat.at<cv::Vec3b>(uv.y, uv.x)[2];
                    temp_cloud->points[i].g = '0' + image_cv_mat.at<cv::Vec3b>(uv.y, uv.x)[1];
                    temp_cloud->points[i].b = '0' + image_cv_mat.at<cv::Vec3b>(uv.y, uv.x)[0];
                }
            }
        }

        extract.setInputCloud(temp_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*temp_cloud);

        counter++;
        std::cout << inliers->indices.size() << "\n";
        pcl::PassThrough<pcl::PointXYZRGB> pass_y;
        pass_y.setInputCloud(temp_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-3, 2);
        pass_y.filter(*temp_cloud);
        pcl::PassThrough<pcl::PointXYZRGB> pass_x;
        pass_x.setInputCloud(temp_cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0, 8);
        pass_x.filter(*temp_cloud);

        sensor_msgs::PointCloud2 ros_output_cloud;
        pcl::toROSMsg(*temp_cloud, ros_output_cloud);
        pub.publish(ros_output_cloud);
}

int main(int argc, char** argv){
    // Initialize topic and broadcaster.
    ros::init(argc, argv, "transform_cloud_to_cam_frame");

    ros::NodeHandle handler;

    // Pass the position and attitude topics through a filter,
    // so that they can by synced together.
    message_filters::Subscriber<sensor_msgs::Image> image_acquire (handler, "/camera/rgb/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> image_acquire
                                    // (handler, "/front_cam/camera/image", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_acquire (handler, "velodyne_points", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> loam_acquire
    //                                 (handler, "/velodyne_cloud_registered", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_acquire (handler, "/camera/rgb/camera_info", 1);
    // message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_acquire
                                // (handler, "/front_cam/camera/camera_info", 1);
    // Object call for devising a policy or parameters on how to
    // synchronize the topics
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync_params(SyncPolicy(5), image_acquire, pcl_acquire, cam_info_acquire);
    // Using these parameters of the sync policy, get the messages, which are synced
    sync_params.registerCallback(boost::bind(&sync_callback, _1, _2, _3));
    pub = handler.advertise<sensor_msgs::PointCloud2>("transformed_cloud_image_frame", 1);
    ROS_INFO("Before ros::spin\n");
    ros::spin();
    return 0;
}
