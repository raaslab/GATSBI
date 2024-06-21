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
from geometry_msgs.msg import Point, PoseArray, Pose, PointStamped, PoseStamped, Quaternion
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

cand_pub = rospy.Publisher('/compute_multiagentnbv_path', Pose, queue_size=10)

drone_1_command_pub = rospy.Publisher('/drone_1_command', PoseStamped, queue_size=10)

drone_2_command_pub = rospy.Publisher('/drone_2_command', PoseStamped, queue_size=10)

rrt_poses = np.zeros(3)

#client = actionlib.SimpleActionClient('airsim_navigator', airsim_moveit_navigation.msg.AirSim_NavigationAction)

nbv_finder_pub = rospy.Publisher('/basic_next_best_view/execute/goal', basic_next_best_view.msg.ExecuteActionGoal, queue_size=1)

drone_1_traversed = False

drone_2_traversed = False

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
total_flight_distance_2 = 0

start_time = 0

initial_data = np.array([[0, 0, 0, 0, 0]])

np.save('/home/user/Data/Reconstruction/multiagent_baseline_data.npy', initial_data)

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
    global total_flight_distance_2

    point1 = np.array((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z))
    point2 = np.array((previous_2_Position.position.x, previous_2_Position.position.y, previous_2_Position.position.z))
    distance = np.linalg.norm(point1 - point2)

    total_flight_distance_2 += distance

    previous_2_Position = data.pose.pose

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
    #client.wait_for_server()

    rospy.Subscriber("/rrt_path", PoseArray, rrt_callback)

    rospy.Subscriber("/airsim_node/drone_1/odom_local_ned", Odometry, odom_callback)
    rospy.Subscriber("/airsim_node/drone_2/odom_local_ned", Odometry, odom_2_callback)

    rospy.Subscriber("/drone_1_arrived", Bool, drone_1_traversed_callback)
    rospy.Subscriber("/drone_2_arrived", Bool, drone_2_traversed_callback)

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
        data_depth_2 = rospy.wait_for_message("/airsim_node/drone_2/depth_points", PointCloud2, timeout=None)

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

        try:
            trans = tf_buffer.lookup_transform("pointr", data_depth_2.header.frame_id,
                                           data_depth_2.header.stamp,
                                           rospy.Duration(1))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            continue
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            continue
        cloud_2_out = do_transform_cloud(data_depth_2, trans)

        if it_count == 1:
            rospy.loginfo("Got pointclouds")

            pc = pypcd.PointCloud.from_msg(cloud_out)
            pc.save('/home/user/airsim.pcd')
            points=np.zeros((pc.pc_data.shape[0],3))
            points[:,0]=pc.pc_data['x']
            points[:,1]=pc.pc_data['y']
            points[:,2]=pc.pc_data['z']
            rospy.loginfo(points.shape[0])

            pc2 = pypcd.PointCloud.from_msg(cloud_2_out)
            pc2.save('/home/user/airsim_2.pcd')
            points_2=np.zeros((pc2.pc_data.shape[0],3))
            points_2[:,0]=pc2.pc_data['x']
            points_2[:,1]=pc2.pc_data['y']
            points_2[:,2]=pc2.pc_data['z']

            points = np.concatenate((points, points_2), axis=0)
            rospy.loginfo("Both drones depth points: %d", points.shape[0])
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

            pc2 = pypcd.PointCloud.from_msg(cloud_2_out)
            pc2.save('/home/user/airsim_2.pcd')
            points_2=np.zeros((pc2.pc_data.shape[0],3))
            points_2[:,0]=pc2.pc_data['x']
            points_2[:,1]=pc2.pc_data['y']
            points_2[:,2]=pc2.pc_data['z']

            points = np.concatenate((points, points_2), axis=0)

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
        current_data = np.array([[rospy.get_rostime().secs - start_time, (total_flight_distance + total_flight_distance_2), total_observed, total_flight_distance, total_flight_distance_2]])
        initial_data = np.concatenate((initial_data, current_data), axis=0)

        np.save('/home/user/Data/Reconstruction/multiagent_baseline_data.npy', initial_data)

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
        candidate_count = 1
        for item in nbv_poses.result.response.best_poses.poses:
            rospy.loginfo("Checking Drone 1 Candidate [%d/%d]", candidate_count, len(nbv_poses.result.response.best_poses.poses))
            candidate_count += 1
            candidate_too_close = False
            if not first_candidate:
                for i in range(0, candidates_visited.shape[0]):
                    #rospy.loginfo("Checking distance to previous candidate: [%f, %f, %f]", candidates_visited[i][0], candidates_visited[i][1], candidates_visited[i][2])
                    potential_pose = np.asarray((item.position.x, item.position.y, item.position.z))
                    dist_to_previous_candidate = np.linalg.norm(potential_pose - candidates_visited[i])
                    global prev_candidate_distance_threshold
                    rospy.loginfo("Drone 1 Distance to previously visited candidate: [%f] Iteration #[%d]", dist_to_previous_candidate, it_count)
                    rospy.loginfo("Drone 1 Distance Threshold: [%d]", prev_candidate_distance_threshold)
                    if dist_to_previous_candidate < prev_candidate_distance_threshold:
                        rospy.loginfo("Setting candidate too close to true")
                        candidate_too_close = True
                        break
            if candidate_too_close:
                rospy.loginfo("Candidate too close, checking next one")
                continue

            
            point = Point(item.position.x, item.position.y, item.position.z)
            orientation = Quaternion(0, 0, 0, 1)
            pose = Pose(point, orientation)
            global rrt_path
            rrt_path = False
            cand_pub.publish(pose)
            while not rrt_path:
                rospy.sleep(0.1)
            
            global rrt_poses
            rrt_poses_drone_1 = rrt_poses
            if(rrt_poses[0][0] == -99 and rrt_poses[0][1] == -99 and rrt_poses[0][2] == -99):
                rospy.loginfo("RRTConnect couldn't find path to candidate pose")
                continue

            found_pose = True
            rospy.loginfo("Found valid pose! NBV Pose #[%d]", pose_count)
            pose_count += 1

            if first_candidate:
                first_candidate = False
                candidates_visited = np.array([[point.x, point.y, point.z]])
            else:
                current_visited = np.array([[point.x, point.y, point.z]])
                candidates_visited = np.concatenate((candidates_visited, current_visited), axis=0)

            break


        # Find frontier for drone_2
        pose_count_2 = 0
        found_pose_2 = False
        candidate_count = 1
        for item in nbv_poses.result.response.best_poses.poses:
            rospy.loginfo("Checking Drone 2 Candidate [%d/%d]", candidate_count, len(nbv_poses.result.response.best_poses.poses))
            candidate_count += 1

            candidate_too_close = False
            if not first_candidate:
                for i in range(0, candidates_visited.shape[0]):
                    #rospy.loginfo("Checking distance to previous candidate: [%f, %f, %f]", candidates_visited[i][0], candidates_visited[i][1], candidates_visited[i][2])
                    potential_pose = np.asarray((item.position.x, item.position.y, item.position.z))
                    dist_to_previous_candidate = np.linalg.norm(potential_pose - candidates_visited[i])
                    global prev_candidate_distance_threshold
                    rospy.loginfo("Drone 2 Distance to previously visited candidate: [%f] Iteration #[%d]", dist_to_previous_candidate, it_count)
                    rospy.loginfo("Drone 2 Distance Threshold: [%d]", prev_candidate_distance_threshold)
                    if dist_to_previous_candidate < prev_candidate_distance_threshold:
                        rospy.loginfo("Setting candidate too close to true")
                        candidate_too_close = True
                        break
            if candidate_too_close:
                rospy.loginfo("Candidate too close, checking next one")
                continue

            
            point2 = Point(item.position.x, item.position.y, item.position.z)
            orientation2 = Quaternion(0, 0, 0, 2)
            pose2 = Pose(point2, orientation2)
            global rrt_path
            rrt_path = False
            cand_pub.publish(pose2)
            while not rrt_path:
                rospy.sleep(0.1)
            
            global rrt_poses
            rrt_poses_drone_2 = rrt_poses

            if(rrt_poses[0][0] == -99 and rrt_poses[0][1] == -99 and rrt_poses[0][2] == -99):
                rospy.loginfo("RRTConnect couldn't find path to candidate pose")
                continue

            
            found_pose_2 = True
            rospy.loginfo("Found valid pose! NBV Pose #[%d]", pose_count_2)
            pose_count_2 += 1

            if first_candidate:
                first_candidate = False
                candidates_visited = np.array([[point2.x, point2.y, point2.z]])
            else:
                current_visited = np.array([[point2.x, point2.y, point2.z]])
                candidates_visited = np.concatenate((candidates_visited, current_visited), axis=0)

            break

        if not found_pose and (not found_pose_2):
            rospy.loginfo("Finished exploring frontiers")
            return

        max_traj = np.amax(np.array([rrt_poses_drone_1.shape[0], rrt_poses_drone_2.shape[0]]))

        start_index = 0
        if max_traj > 1:
            start_index = 1

        for i in range(start_index, max_traj):
            if i < rrt_poses_drone_1.shape[0]:
                rospy.loginfo("Drone 1 Traj Step[%d]: [%.2f, %.2f, %.2f]", i + 1, rrt_poses_drone_1[i][0], rrt_poses_drone_1[i][1], rrt_poses_drone_1[i][2])
                drone_1_goal = PoseStamped()
                drone_1_goal.header.frame_id = 'world_enu'
                drone_1_goal.pose.position.x = rrt_poses_drone_1[i][0]
                drone_1_goal.pose.position.y = rrt_poses_drone_1[i][1]
                drone_1_goal.pose.position.z = rrt_poses_drone_1[i][2]
                drone_1_goal.pose.orientation.x = centroid[0]
                drone_1_goal.pose.orientation.y = centroid[1]
                drone_1_goal.pose.orientation.z = centroid[2]

                global drone_1_traversed
                drone_1_traversed = False

                drone_1_command_pub.publish(drone_1_goal)
            else:
                # To prevent hover mode, if drone_1 trajectory finishes before drone_2, send final trajectory position 
                final_index = rrt_poses_drone_1.shape[0] - 1
                drone_1_goal = PoseStamped()
                drone_1_goal.header.frame_id = 'world_enu'
                drone_1_goal.pose.position.x = rrt_poses_drone_1[final_index][0]
                drone_1_goal.pose.position.y = rrt_poses_drone_1[final_index][1]
                drone_1_goal.pose.position.z = rrt_poses_drone_1[final_index][2]
                drone_1_goal.pose.orientation.x = centroid[0]
                drone_1_goal.pose.orientation.y = centroid[1]
                drone_1_goal.pose.orientation.z = centroid[2]

                global drone_1_traversed
                drone_1_traversed = False

                drone_1_command_pub.publish(drone_1_goal)


            if i < rrt_poses_drone_2.shape[0]:
                rospy.loginfo("Drone 2 Traj Step[%d]: [%.2f, %.2f, %.2f]", i + 1, rrt_poses_drone_2[i][0], rrt_poses_drone_2[i][1], rrt_poses_drone_2[i][2])
                drone_2_goal = PoseStamped()
                drone_2_goal.header.frame_id = 'world_enu'
                drone_2_goal.pose.position.x = rrt_poses_drone_2[i][0]
                drone_2_goal.pose.position.y = rrt_poses_drone_2[i][1]
                drone_2_goal.pose.position.z = rrt_poses_drone_2[i][2]
                drone_2_goal.pose.orientation.x = centroid[0]
                drone_2_goal.pose.orientation.y = centroid[1]
                drone_2_goal.pose.orientation.z = centroid[2]

                global drone_2_traversed
                drone_2_traversed = False

                drone_2_command_pub.publish(drone_2_goal)
            else:
                # To prevent hover mode, if drone_2 trajectory finishes before drone_2, send final trajectory position 
                final_index = rrt_poses_drone_2.shape[0] - 1
                drone_2_goal = PoseStamped()
                drone_2_goal.header.frame_id = 'world_enu'
                drone_2_goal.pose.position.x = rrt_poses_drone_2[final_index][0]
                drone_2_goal.pose.position.y = rrt_poses_drone_2[final_index][1]
                drone_2_goal.pose.position.z = rrt_poses_drone_2[final_index][2]
                drone_2_goal.pose.orientation.x = centroid[0]
                drone_2_goal.pose.orientation.y = centroid[1]
                drone_2_goal.pose.orientation.z = centroid[2]

                global drone_2_traversed
                drone_2_traversed = False

                drone_2_command_pub.publish(drone_2_goal)


            while (not drone_1_traversed) or (not drone_2_traversed):
                rospy.sleep(0.1)


        it_count += 1
        rospy.sleep(1)

        


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

