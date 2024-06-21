#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
import time
import os
import pypcd
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, PoseArray, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image

import airsim_moveit_navigation.msg

import struct
import actionlib

import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

bridge = CvBridge()

prev_candidate_distance_threshold = 5

predict_pointcloud = False

drone_1_traversed = False

drone_2_traversed = False

first = True

rrt_path = False

uav_paused = False

traj_traversed = False

pub = rospy.Publisher('/nbv_position_ready', Bool, queue_size=10)

pc_pub = rospy.Publisher('/predicted_pointcloud', PointCloud2, queue_size=1)

cand_pub = rospy.Publisher('/compute_multiagentnbv_path', Pose, queue_size=10)

drone_1_command_pub = rospy.Publisher('/drone_1_command', PoseStamped, queue_size=10)

drone_2_command_pub = rospy.Publisher('/drone_2_command', PoseStamped, queue_size=10)

rrt_poses = np.zeros(3)

#client = actionlib.SimpleActionClient('airsim_navigator', airsim_moveit_navigation.msg.AirSim_NavigationAction)

current_Position = Pose()
previous_Position = Pose()
previous_Position.position.x = 0
previous_Position.position.y = 0
previous_Position.position.z = 0

previous_2_Position = Pose()
previous_2_Position.position.x = 0
previous_2_Position.position.y = 0
previous_2_Position.position.z = 0

flight_count = 0
flight_2_count = 0

total_flight_distance = 0
total_flight_2_distance = 0

start_time = 0

initial_data = np.array([[0, 0, 0, 0, 0]])
np.save('/home/user/Data/Reconstruction/multiagent_prednbv_data.npy', initial_data)

def rrt_callback(data):
    #rospy.loginfo("Got RRT path")
    
    first_position = True
    global rrt_poses
    for item in data.poses:
        if first_position:
            first_position = False
            rrt_poses = np.array([[item.position.x, item.position.y, item.position.z]])
        else:
            position = np.array([[item.position.x, item.position.y, item.position.z]])
            rrt_poses = np.concatenate((rrt_poses, position), axis=0)


    global rrt_path
    rrt_path = True

def callback(data):

    global first
    if first:
        rospy.loginfo("Got initial pointcloud")
        pc = pypcd.PointCloud.from_msg(data)
        pc.save('/home/user/airsim.pcd')
        points=np.zeros((pc.pc_data.shape[0],3))
        points[:,0]=pc.pc_data['x']
        points[:,1]=pc.pc_data['y']
        points[:,2]=pc.pc_data['z']
        np.save('/home/user/airsim_observed.npy', points)
        rospy.loginfo("Filtering pointcloud")
        os.system('python3 ~/Code/PoinTr/filter_pc.py')

        #first = False

    predict_pointcloud = False

    
def bool_callback(data):
    rospy.loginfo("Got message to predict pointcloud")

    global predict_pointcloud
    predict_pointcloud = data.data


def drone_1_traversed_callback(data):
    rospy.loginfo("Drone 1 Traversed")

    global drone_1_traversed
    drone_1_traversed = data.data


def drone_2_traversed_callback(data):
    rospy.loginfo("Drone 2 Traversed")

    global drone_2_traversed
    drone_2_traversed = data.data


def odom_callback(data):

    global flight_count
    if flight_count < 5:
        flight_count += 1
        return

    flight_count = 0

    global previous_Position
    global total_flight_distance

    point1 = np.array((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z))
    point2 = np.array((previous_Position.position.x, previous_Position.position.y, previous_Position.position.z))
    distance = np.linalg.norm(point1 - point2)

    total_flight_distance += distance

    previous_Position = data.pose.pose


def odom_2_callback(data):

    global flight_2_count
    if flight_2_count < 5:
        flight_2_count += 1
        return

    flight_2_count = 0

    global previous_2_Position
    global total_flight_2_distance

    point1 = np.array((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z))
    point2 = np.array((previous_2_Position.position.x, previous_2_Position.position.y, previous_2_Position.position.z))
    distance = np.linalg.norm(point1 - point2)

    total_flight_2_distance += distance

    previous_2_Position = data.pose.pose


def uav_traj_pause_callback(data):
    rospy.loginfo("UAV paused");
    global uav_paused
    uav_paused = True


def callback_done(state, result):
    global traj_traversed
    traj_traversed = True;
    rospy.loginfo("Action server is done.")


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_feedback(feedback):
    rospy.loginfo("Got action server feedback")


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # Wait to connect to server
    rospy.loginfo("Waiting for server")
    #client.wait_for_server()

    #rospy.Subscriber("/points", PointCloud2, callback)

    rospy.Subscriber("/predict_pointcloud", Bool, bool_callback)

    rospy.Subscriber("/drone_1_arrived", Bool, drone_1_traversed_callback)

    rospy.Subscriber("/drone_2_arrived", Bool, drone_2_traversed_callback)

    rospy.Subscriber("/airsim_node/drone_1/odom_local_ned", Odometry, odom_callback)

    rospy.Subscriber("/airsim_node/drone_2/odom_local_ned", Odometry, odom_2_callback)

    rospy.Subscriber("/rrt_path", PoseArray, rrt_callback)

    rospy.Subscriber("/drone_trajectory_paused", Bool, uav_traj_pause_callback)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    global start_time
    start_time = rospy.get_rostime().secs
    first = True

    total_observed = float(0.0)
    previous_step_observed_total = float(0.0)
    it_count = 1

    candidates_visited = np.zeros(3)
    nbv_poses = np.zeros(3)
    first_candidate = True
    while True:

        rospy.loginfo("Running NBV planner: Iteration [%d]", it_count)
        global total_flight_distance
        global total_flight_2_distance
        rospy.loginfo("Total Flight Distance for Drone 1: [%f] meters", total_flight_distance)
        rospy.loginfo("Total Flight Distance for Drone 2: [%f] meters", total_flight_2_distance)
        rospy.loginfo("Total Flight Distance for Both: [%f] meters", (total_flight_distance + total_flight_2_distance))
        data = rospy.wait_for_message("/airsim_node/drone_1/depth_points", PointCloud2, timeout=None)
        data2 = rospy.wait_for_message("/airsim_node/drone_2/depth_points", PointCloud2, timeout=None)

        try:
            trans = tf_buffer.lookup_transform("pointr", data.header.frame_id,
                                           data.header.stamp,
                                           rospy.Duration(1))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            continue
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            continue
        cloud_out = do_transform_cloud(data, trans)

        try:
            trans = tf_buffer.lookup_transform("pointr", data2.header.frame_id,
                                           data2.header.stamp,
                                           rospy.Duration(1))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            continue
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            continue
        cloud_2_out = do_transform_cloud(data2, trans)

        if it_count == 1:
            rospy.loginfo("Got pointclouds")
            pc = pypcd.PointCloud.from_msg(cloud_out)
            pc.save('/home/user/airsim.pcd')
            points=np.zeros((pc.pc_data.shape[0],3))
            points[:,0]=pc.pc_data['x']
            points[:,1]=pc.pc_data['y']
            points[:,2]=pc.pc_data['z']
            np.save('/home/user/Data/Reconstruction/drone_1_pointcloud_iteration_0.npy', points)

            rospy.loginfo("Drone 1 depth points: %d", points.shape[0])

            pc2 = pypcd.PointCloud.from_msg(cloud_2_out)
            pc2.save('/home/user/airsim_2.pcd')
            points_2=np.zeros((pc2.pc_data.shape[0],3))
            points_2[:,0]=pc2.pc_data['x']
            points_2[:,1]=pc2.pc_data['y']
            points_2[:,2]=pc2.pc_data['z']
            np.save('/home/user/Data/Reconstruction/drone_2_pointcloud_iteration_0.npy', points_2)

            points = np.concatenate((points, points_2), axis=0)
            np.save('/home/user/Data/Reconstruction/drone_all_pointcloud_iteration_0.npy', points)

            rospy.loginfo("Both drones depth points: %d", points.shape[0])
            np.save('/home/user/airsim_observed.npy', points)
            rospy.loginfo("Filtering pointcloud")
            os.system('python3 ~/Code/PoinTr/filter_pc.py')

            rospy.loginfo('Saving initial images')
            rgb_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_color/Scene", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_rgb_image_iteration_0.jpeg', cv2_img)

            rgb_segmented_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_segmentation/Segmentation", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(rgb_segmented_image, "bgr8")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_rgb_segmented_image_iteration_0.jpeg', cv2_img)

            depth_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth/DepthPerspective", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(depth_image, "32FC1")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_depth_image_iteration_0.jpeg', cv2_img)

            depth_segmented = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth_segmented/DepthSegmented", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(depth_segmented, "32FC1")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_depth_segmented_image_iteration_0.jpeg', cv2_img)

            rgb_image = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_color/Scene", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_rgb_image_iteration_0.jpeg', cv2_img)

            rgb_segmented_image = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_segmentation/Segmentation", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(rgb_segmented_image, "bgr8")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_rgb_segmented_image_iteration_0.jpeg', cv2_img)

            depth_image = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_depth/DepthPerspective", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(depth_image, "32FC1")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_depth_image_iteration_0.jpeg', cv2_img)

            depth_segmented = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_depth_segmented/DepthSegmented", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(depth_segmented, "32FC1")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_depth_segmented_image_iteration_0.jpeg', cv2_img)

            first = False

        else:
            rospy.loginfo("Got pointclouds")
            rospy.loginfo("Concatenating pointclouds to previously observed")
            previously_observed = np.load('/home/user/airsim_filtered.npy')
            pc = pypcd.PointCloud.from_msg(cloud_out)

            points=np.zeros((pc.pc_data.shape[0],3))
            points[:,0]=pc.pc_data['x']
            points[:,1]=pc.pc_data['y']
            points[:,2]=pc.pc_data['z']
            np.save('/home/user/Data/Reconstruction/drone_1_pointcloud_iteration_' + str(it_count - 1) + '.npy', points)

            pc2 = pypcd.PointCloud.from_msg(cloud_2_out)
            pc2.save('/home/user/airsim_2.pcd')
            points_2=np.zeros((pc2.pc_data.shape[0],3))
            points_2[:,0]=pc2.pc_data['x']
            points_2[:,1]=pc2.pc_data['y']
            points_2[:,2]=pc2.pc_data['z']
            np.save('/home/user/Data/Reconstruction/drone_2_pointcloud_iteration_' + str(it_count - 1) + '.npy', points_2)

            points = np.concatenate((points, points_2), axis=0)
            np.save('/home/user/Data/Reconstruction/drone_all_pointcloud_iteration_' + str(it_count - 1) + '.npy', points)

            return

        all_filtered = np.load('/home/user/airsim_filtered.npy')
        centroid = np.mean(all_filtered, axis=0)

        # Center and normalize pointcloud (going to restore it after prediction)

        x = -25.0
        y = -25.0
        z = 3

        rospy.loginfo("Drone 1 Traj Step[1]: [%.2f, %.2f, %.2f]", x, y, z)
        drone_1_goal = PoseStamped()
        drone_1_goal.header.frame_id = 'world_enu'
        drone_1_goal.pose.position.x = x
        drone_1_goal.pose.position.y = y
        drone_1_goal.pose.position.z = z
        drone_1_goal.pose.orientation.x = centroid[0]
        drone_1_goal.pose.orientation.y = centroid[1]
        drone_1_goal.pose.orientation.z = centroid[2]

        global drone_1_traversed
        drone_1_traversed = False


        drone_1_command_pub.publish(drone_1_goal)

        '''
        rospy.loginfo("Drone 2 Traj Step[%d]: [%.2f, %.2f, %.2f]", i + 1, drone_2_trajectory[i][2], drone_2_trajectory[i][3], drone_2_trajectory[i][4])
        drone_2_goal = PoseStamped()
        drone_2_goal.header.frame_id = 'world_enu'
        drone_2_goal.pose.position.x = drone_2_trajectory[i][2]
        drone_2_goal.pose.position.y = drone_2_trajectory[i][3]
        drone_2_goal.pose.position.z = drone_2_trajectory[i][4]
        drone_2_goal.pose.orientation.x = centroid[0]
        drone_2_goal.pose.orientation.y = centroid[1]
        drone_2_goal.pose.orientation.z = centroid[2]

        global drone_2_traversed
        drone_2_traversed = False
    
        drone_2_command_pub.publish(drone_2_goal)
        '''
        drone_2_traversed = True
        while (not drone_1_traversed) or (not drone_2_traversed):
            rospy.sleep(0.1)

        rospy.loginfo('Saving NBV images')
        rgb_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_color/Scene", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_rgb_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        rgb_segmented_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_segmentation/Segmentation", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(rgb_segmented_image, "bgr8")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_rgb_segmented_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        depth_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth/DepthPerspective", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(depth_image, "32FC1")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_depth_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        depth_segmented = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth_segmented/DepthSegmented", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(depth_segmented, "32FC1")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_1_depth_segmented_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        rgb_image = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_color/Scene", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_rgb_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        rgb_segmented_image = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_segmentation/Segmentation", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(rgb_segmented_image, "bgr8")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_rgb_segmented_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        depth_image = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_depth/DepthPerspective", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(depth_image, "32FC1")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_depth_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        depth_segmented = rospy.wait_for_message("/airsim_node/drone_2/front_center_custom_depth_segmented/DepthSegmented", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(depth_segmented, "32FC1")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/drone_2_depth_segmented_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        it_count += 1
        rospy.sleep(1)

        


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

