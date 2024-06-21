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
from geometry_msgs.msg import Point, PoseArray, Pose, PointStamped
from nav_msgs.msg import Odometry

import airsim_moveit_navigation.msg
import basic_next_best_view.msg

import struct
import actionlib

import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

prev_candidate_distance_threshold = 2

pub = rospy.Publisher('/nbv_position_ready', Bool, queue_size=10)

occupancy_pub = rospy.Publisher('/basic_next_best_view/occupancy', PointCloud2, queue_size=1)

poi_pub = rospy.Publisher('/basic_next_best_view/poi', Point, queue_size=1)

cand_pub = rospy.Publisher('/compute_nbv_path', Point, queue_size=10)

rrt_poses = np.zeros(3)

client = actionlib.SimpleActionClient('airsim_navigator', airsim_moveit_navigation.msg.AirSim_NavigationAction)

nbv_finder_pub = rospy.Publisher('/basic_next_best_view/execute/goal', basic_next_best_view.msg.ExecuteActionGoal, queue_size=1)

current_Position = Pose()
previous_Position = Pose()
previous_Position.position.x = 0
previous_Position.position.y = 0
previous_Position.position.z = 0

flight_count = 0

total_flight_distance = 0

start_time = 0

initial_data = np.array([[0, 0, 0]])

np.save('/home/user/Data/Reconstruction/baseline_data.npy', initial_data)

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

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    global start_time
    start_time = rospy.get_rostime().secs

    # Wait to connect to server
    rospy.loginfo("Waiting for AirSim navigation server")
    client.wait_for_server()

    rospy.Subscriber("/rrt_path", PoseArray, rrt_callback)

    rospy.Subscriber("/airsim_node/drone_1/odom_local_ned", Odometry, odom_callback)

    rospy.Publisher('/basic_next_best_view/execute/goal', basic_next_best_view.msg.ExecuteActionGoal, queue_size=1)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    first = True

    it_count = 1

    candidates_visited = np.zeros(3)
    first_candidate = True
    total_observed = 0
    previous_observed = 0
    while True:

        rospy.loginfo("Running Baseline NBV planner: Iteration [%d]", it_count)

        data_depth = rospy.wait_for_message("/airsim_node/drone_1/depth_points", PointCloud2, timeout=None)

        try:
            trans = tf_buffer.lookup_transform("pointr", data_depth.header.frame_id,
                                           data_depth.header.stamp,
                                           rospy.Duration(1))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            continue
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            continue
        cloud_out = do_transform_cloud(data_depth, trans)

        if it_count == 1:
            rospy.loginfo("Got pointcloud")
            pc = pypcd.PointCloud.from_msg(cloud_out)
            pc.save('/home/user/airsim.pcd')
            points=np.zeros((pc.pc_data.shape[0],3))
            points[:,0]=pc.pc_data['x']
            points[:,1]=pc.pc_data['y']
            points[:,2]=pc.pc_data['z']
            rospy.loginfo(points.shape[0])
            np.save('/home/user/airsim_observed.npy', points)
            rospy.loginfo("Filtering pointcloud")
            os.system('python3 ~/Code/PoinTr/filter_pc.py')
            
        else:
            rospy.loginfo("Concatenating pointcloud to previously observed")
            previously_observed = np.load('/home/user/airsim_filtered.npy')
            pc = pypcd.PointCloud.from_msg(cloud_out)

            points=np.zeros((pc.pc_data.shape[0],3))
            points[:,0]=pc.pc_data['x']
            points[:,1]=pc.pc_data['y']
            points[:,2]=pc.pc_data['z']

            np.save('/home/user/airsim_observed.npy', points)
            rospy.loginfo("Filtering pointcloud")
            os.system('python3 ~/Code/PoinTr/filter_pc.py')

            new_observed = np.load('/home/user/airsim_filtered.npy')

            all_observed = np.concatenate((new_observed, previously_observed), axis=0)
            all_observed = np.unique(all_observed, axis=0)
            
            #all_observed = points
            np.save('/home/user/airsim_filtered.npy', all_observed)
            rospy.loginfo("Downsampling total observed pointcloud")
            os.system('python3 ~/Code/PoinTr/voxel_filter_baseline.py')


        # Center and normalize pointcloud (going to restore it after prediction)

        all_filtered = np.load('/home/user/airsim_filtered.npy')

        centroid = np.mean(all_filtered, axis=0)

        previous_observed = total_observed
        total_observed = float(all_filtered.shape[0])


        rospy.loginfo("Updating logged data")
        global initial_data
        current_data = np.array([[rospy.get_rostime().secs - start_time, total_flight_distance, total_observed]])
        initial_data = np.concatenate((initial_data, current_data), axis=0)

        np.save('/home/user/Data/Reconstruction/baseline_data.npy', initial_data)

        stop_ratio = previous_observed / total_observed
        rospy.loginfo("Previous Observed: %d", previous_observed)
        rospy.loginfo("Total Observed: %d", total_observed)
        rospy.loginfo("Old/New Points Ratio: %f", stop_ratio)
        if stop_ratio > 0.95:
            rospy.loginfo("Done with NBV")
            return

        data = rospy.wait_for_message("/octomap_points", PointCloud2, timeout=None)

        occupancy_pub.publish(data)

        rospy.sleep(0.5)
        

        poi = Point(centroid[0], centroid[1], centroid[2])

        poi_pub.publish(poi)
        rospy.sleep(0.5)

        goal = basic_next_best_view.msg.ExecuteActionGoal()

        goal.header.frame_id = "test"
        goal.goal.request.type = 3;
        goal.goal.request.has_index_range = False;

        nbv_finder_pub.publish(goal)

        nbv_poses = rospy.wait_for_message("/basic_next_best_view/execute/result", basic_next_best_view.msg.ExecuteActionResult, timeout=None)

        pose_count = 0
        found_pose = False
        for item in nbv_poses.result.response.best_poses.poses:

            candidate_too_close = False
            if not first_candidate:
                for i in range(0, candidates_visited.shape[0]):
                    #rospy.loginfo("Checking distance to previous candidate: [%f, %f, %f]", candidates_visited[i][0], candidates_visited[i][1], candidates_visited[i][2])
                    potential_pose = np.asarray((item.position.x, item.position.y, item.position.z))
                    dist_to_previous_candidate = np.linalg.norm(potential_pose - candidates_visited[i])
                    global prev_candidate_distance_threshold
                    rospy.loginfo("Distance to previously visited candidate: [%f] Iteration #[%d]", dist_to_previous_candidate, it_count)
                    rospy.loginfo("Distance Threshold: [%d]", prev_candidate_distance_threshold)
                    if dist_to_previous_candidate < prev_candidate_distance_threshold:
                        rospy.loginfo("Setting candidate too close to true")
                        candidate_too_close = True
                        break
            if candidate_too_close:
                rospy.loginfo("Candidate too close, checking next one")
                continue

            
            pose = Point(item.position.x, item.position.y, item.position.z)
            global rrt_path
            rrt_path = False
            cand_pub.publish(pose)
            while not rrt_path:
                rospy.sleep(0.1)
            
            global rrt_poses
            if(rrt_poses[0][0] == -99 and rrt_poses[0][1] == -99 and rrt_poses[0][2] == -99):
                rospy.loginfo("RRTConnect couldn't find path to candidate pose")
                continue

            found_pose = True
            rospy.loginfo("Found valid pose! NBV Pose #[%d]", pose_count)
            pose_count += 1

            if first_candidate:
                first_candidate = False
                candidates_visited = np.array([[pose.x, pose.y, pose.z]])
            else:
                current_visited = np.array([[pose.x, pose.y, pose.z]])
                candidates_visited = np.concatenate((candidates_visited, current_visited), axis=0)

            goal_position = airsim_moveit_navigation.msg.AirSim_NavigationGoal()

            goal_position.goal_pose.position.x = pose.x;
            goal_position.goal_pose.position.y = pose.y;
            goal_position.goal_pose.position.z = pose.z;
            goal_position.goal_pose.orientation.x = centroid[0];
            goal_position.goal_pose.orientation.y = centroid[1];
            goal_position.goal_pose.orientation.z = centroid[2];
            goal_position.goal_pose.orientation.w = 0;

            rospy.loginfo("Sending NBV to action server")
            client.send_goal(goal_position)

            rospy.loginfo("Waiting for result")
            client.wait_for_result()

            break

        if not found_pose:
            rospy.loginfo("Finished exploring frontiers")
            return
        it_count += 1
        rospy.sleep(1)

        


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

