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
from geometry_msgs.msg import Point, PoseArray, Pose
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

first = True

rrt_path = False

uav_paused = False

traj_traversed = False

pub = rospy.Publisher('/nbv_position_ready', Bool, queue_size=10)

pc_pub = rospy.Publisher('/predicted_pointcloud', PointCloud2, queue_size=1)

cand_pub = rospy.Publisher('/compute_nbv_path', Point, queue_size=10)

rrt_poses = np.zeros(3)

client = actionlib.SimpleActionClient('airsim_navigator', airsim_moveit_navigation.msg.AirSim_NavigationAction)

current_Position = Pose()
previous_Position = Pose()
previous_Position.position.x = 0
previous_Position.position.y = 0
previous_Position.position.z = 0

flight_count = 0

total_flight_distance = 0

start_time = 0

initial_data = np.array([[0, 0, 0]])
np.save('/home/user/Data/Reconstruction/prednbv_data.npy', initial_data)

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

    '''
    else:
        rospy.loginfo("Concatenating pointcloud to previously observed")
        previously_observed = np.load('/home/user/airsim_filtered.npy')
        pc = pypcd.PointCloud.from_msg(data)

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

    
                
    # Center and normalize pointcloud (going to restore it after prediction)

    all_filtered = np.load('/home/user/airsim_filtered.npy')
    centroid = np.mean(all_filtered, axis=0)
    all_filtered = all_filtered - centroid
    m = np.max(np.sqrt(np.sum(all_filtered**2, axis=1)))
    all_filtered = all_filtered / m
    all_filtered[:,1] *= -1

    np.save('/home/user/airsim_input_before_prediction.npy', all_filtered)
    np.save('/home/user/Code/PoinTr/data/ShapeNet55-34/shapenet_pc/02691156-airplane.npy', all_filtered)

    rospy.loginfo("Predicting pointcloud using PoinTr")
    os.system('bash /home/user/Code/PoinTr/scripts/test.sh 0 --ckpts /home/user/Code/PoinTr/pretrained/norm_360_0.1.pth --config /home/user/Code/PoinTr/cfgs/ShapeNet55_models/PoinTr.yaml --mode easy --exp_name test_example')

    predicted = np.load("/home/user/dense.npy")
    predicted = predicted.reshape((predicted.shape[1], predicted.shape[2]))
    predicted[:,1] *= -1
    predicted = predicted * m        
    predicted = predicted + centroid

    predicted_data = predicted

    rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
    rgb_col = np.empty(predicted.shape[0])
    rgb_col.fill(rgb)

    predicted = np.hstack((predicted, rgb_col.reshape(rgb_col.shape[0], 1)))
    header = data.header
    header.stamp = rospy.Time.now()

    points_out = []
    for i in range(0, predicted.shape[0]):
        points_out.append(predicted[i])
    #points_out.append(predicted.tolist())
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 16, PointField.UINT32, 1),
             ]

    predictedPC = point_cloud2.create_cloud(header, fields, points_out)
    rospy.loginfo("Publish predicted pointcloud")
    pc_pub.publish(predictedPC)

    rospy.loginfo("Find candidate poses")
    os.system('python3 /home/user/Code/nbv_simulation/find_candidate_poses.py')

    candidate_poses = np.load('/home/user/candidate_poses.npy')
    candidate_poses[:,1] *= -1
    candidate_poses = candidate_poses * m        
    candidate_poses = candidate_poses + centroid
    
    first_check = True
    valid_candidate_poses = np.zeros(3)
    trajectories = np.zeros(4)

    valid_count = 0
    for item in candidate_poses:
        pose = Point(item[0], item[1], item[2])
        global rrt_path
        rrt_path = False
        cand_pub.publish(pose)
        while not rrt_path:
            rospy.sleep(0.1)
        
        global rrt_poses
        if(rrt_poses[0][0] == -99 and rrt_poses[0][1] == -99 and rrt_poses[0][2] == -99):
            #rospy.loginfo("RRTConnect couldn't find path to candidate pose")
            continue

        rrt_poses = rrt_poses - centroid
        rrt_poses = rrt_poses / m
        rrt_poses[:,1] *= -1

        item = item - centroid
        item = item / m
        item[1] *= -1

        if first_check:
            first_check = False
            valid_candidate_poses = np.array([[item[0], item[1], item[2]]])
            pose_number = np.empty([rrt_poses.shape[0], 1])
            pose_number.fill(valid_count)

            trajectories = np.concatenate((pose_number, rrt_poses), axis=1)

        else:
            position = np.array([[item[0], item[1], item[2]]])
            valid_candidate_poses = np.concatenate((valid_candidate_poses, position), axis=0)

            pose_number = np.empty([rrt_poses.shape[0], 1])
            pose_number.fill(valid_count)

            pose_number = np.concatenate((pose_number, rrt_poses), axis=1)

            trajectories = np.concatenate((trajectories, pose_number), axis=0)

        valid_count = valid_count + 1


        
    rospy.loginfo("Saving valid candidate poses and RRT Connect trajectories")
    np.save('/home/user/valid_candidate_poses.npy', valid_candidate_poses)
    np.save('/home/user/rrt_trajectories.npy', trajectories)

    rospy.loginfo("Find Next Best View")
    os.system('python3 /home/user/Code/nbv_simulation/find_nbv.py')

    next_pose = np.load('/home/user/next_candidate.npy')
    next_pose[1] *= -1
    next_pose = next_pose * m        
    next_pose = next_pose + centroid

    goal = airsim_moveit_navigation.msg.AirSim_NavigationGoal()

    goal.goal_pose.position.x = next_pose[0];
    goal.goal_pose.position.y = next_pose[1];
    goal.goal_pose.position.z = next_pose[2];
    goal.goal_pose.orientation.x = centroid[0];
    goal.goal_pose.orientation.y = centroid[1];
    goal.goal_pose.orientation.z = centroid[2];
    goal.goal_pose.orientation.w = 0;

    rospy.loginfo("Sending NBV to action server")
    client.send_goal(goal)

    rospy.loginfo("Waiting for result")
    client.wait_for_result()

    rospy.sleep(1)

    
    partial_data = np.load("/home/user/partial.npy")
    partial_data = partial_data.reshape((partial_data.shape[1], partial_data.shape[2]))
    partial_data = partial_data * m        
    partial_data = partial_data + centroid


    np.save("/home/user/only_input.npy", partial_data)


    for item in partial_data:
        idx = np.argwhere(predicted_data == item)
        predicted_data = np.delete(predicted_data, idx[0][0], axis=0)


    np.save("/home/user/only_predicted.npy", predicted_data)
    '''

    #rospy.loginfo("Finding next best view and saving NBV position")
    #os.system('python3 ~/Code/NBV_Reconstruction/nbv_pointr.py')


    #rospy.loginfo("Publishing that NBV position is ready")
    #global pub
    #pub_msgs = Bool()
    #pub_msgs.data = True
    #pub.publish(pub_msgs)
    predict_pointcloud = False

    
def bool_callback(data):
    rospy.loginfo("Got message to predict pointcloud")

    global predict_pointcloud
    predict_pointcloud = data.data


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
    client.wait_for_server()

    #rospy.Subscriber("/points", PointCloud2, callback)

    rospy.Subscriber("/predict_pointcloud", Bool, bool_callback)

    rospy.Subscriber("/airsim_node/drone_1/odom_local_ned", Odometry, odom_callback)

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
        rospy.loginfo("Total Flight Distance: [%f] meters", total_flight_distance)
        data = rospy.wait_for_message("/airsim_node/drone_1/depth_points", PointCloud2, timeout=None)

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

            rospy.loginfo('Saving initial images')
            rgb_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_color/Scene", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/rgb_image_iteration_0.jpeg', cv2_img)

            rgb_segmented_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_segmentation/Segmentation", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(rgb_segmented_image, "bgr8")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/rgb_segmented_image_iteration_0.jpeg', cv2_img)

            depth_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth/DepthPerspective", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(depth_image, "32FC1")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/depth_image_iteration_0.jpeg', cv2_img)

            depth_segmented = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth_segmented/DepthSegmented", Image, timeout=None)
            cv2_img = bridge.imgmsg_to_cv2(depth_segmented, "32FC1")
            cv2.imwrite('/home/user/Data/Reconstruction/Images/depth_segmented_image_iteration_0.jpeg', cv2_img)
            first = False

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
            os.system('python3 ~/Code/PoinTr/voxel_filter.py')


        # Center and normalize pointcloud (going to restore it after prediction)

        all_filtered = np.load('/home/user/airsim_filtered.npy')


        total_observed = float(all_filtered.shape[0])

        rospy.loginfo("Updating logged data")
        global initial_data
        current_data = np.array([[rospy.get_rostime().secs - start_time, total_flight_distance, total_observed]])
        initial_data = np.concatenate((initial_data, current_data), axis=0)
        np.save('/home/user/Data/Reconstruction/prednbv_data.npy', initial_data)

        
        rospy.loginfo("previously observed: %f", previous_step_observed_total)
        rospy.loginfo("current observed: %f", total_observed)

        if it_count > 1:
            stop_ratio = previous_step_observed_total / total_observed
            rospy.loginfo("Old/New Points Ratio: %f", stop_ratio)
            if stop_ratio > 0.95:
                np.save('/home/user/Data/Reconstruction/visited_poses.npy', nbv_poses)
                rospy.loginfo("Done with NBV")
                return
        previous_step_observed_total = total_observed

        centroid = np.mean(all_filtered, axis=0)
        all_filtered = all_filtered - centroid
        m = np.max(np.sqrt(np.sum(all_filtered**2, axis=1)))
        all_filtered = all_filtered / m
        #all_filtered[:,1] *= -1

        np.save('/home/user/airsim_input_before_prediction.npy', all_filtered)
        np.save('/home/user/Data/Reconstruction/airsim_observed_iteration_' + str(it_count) + '.npy', all_filtered)
        np.save('/home/user/Code/PoinTr/data/ShapeNet55-34/shapenet_pc/02691156-airplane.npy', all_filtered)

        rospy.loginfo("Predicting pointcloud using PoinTr")
        os.system('bash /home/user/Code/PoinTr/scripts/test.sh 0 --ckpts /home/user/Code/PoinTr/pretrained/norm_360_0.2.pth --config /home/user/Code/PoinTr/cfgs/ShapeNet55_models/PoinTr.yaml --mode easy --exp_name test_example')

        predicted = np.load("/home/user/dense.npy")
        np.save('/home/user/Data/Reconstruction/airsim_predicted_iteration_' + str(it_count) + '.npy', predicted)
        predicted = predicted.reshape((predicted.shape[1], predicted.shape[2]))
        #predicted[:,1] *= -1
        predicted = predicted * m        
        predicted = predicted + centroid

        predicted_data = predicted

        rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
        rgb_col = np.empty(predicted.shape[0])
        rgb_col.fill(rgb)

        predicted = np.hstack((predicted, rgb_col.reshape(rgb_col.shape[0], 1)))
        header = data.header
        header.stamp = rospy.Time.now()
        header.frame_id = "pointr"

        points_out = []
        for i in range(0, predicted.shape[0]):
            points_out.append(predicted[i])
        #points_out.append(predicted.tolist())
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 16, PointField.UINT32, 1),
                 ]

        predictedPC = point_cloud2.create_cloud(header, fields, points_out)
        rospy.loginfo("Publish predicted pointcloud")
        pc_pub.publish(predictedPC)

        rospy.loginfo("Find candidate poses")
        os.system('python3 /home/user/Code/nbv_simulation/find_candidate_poses.py')

        candidate_poses = np.load('/home/user/candidate_poses.npy')
        #candidate_poses[:,1] *= -1
        candidate_poses = candidate_poses * m        
        candidate_poses = candidate_poses + centroid
        
        first_check = True
        valid_candidate_poses = np.zeros(3)
        trajectories = np.array([[0,0,0,0]])

        valid_count = 0
        candidate_count = 1
        for item in candidate_poses:
            rospy.loginfo("Checking if Drone can fly to candidate viewpoint [%d/%d]", candidate_count, candidate_poses.shape[0])
            candidate_count += 1

            candidate_too_close = False
            if not first_candidate:
                for i in range(0, candidates_visited.shape[0]):
                    #rospy.loginfo("Checking distance to previous candidate: [%f, %f, %f]", candidates_visited[i][0], candidates_visited[i][1], candidates_visited[i][2])
                    dist_to_previous_candidate = np.linalg.norm(item - candidates_visited[i])
                    global prev_candidate_distance_threshold
                    #rospy.loginfo("Distance to previously visited candidate: [%f] Iteration #[%d]", dist_to_previous_candidate, it_count)
                    #rospy.loginfo("Distance Threshold: [%d]", prev_candidate_distance_threshold)
                    if dist_to_previous_candidate < prev_candidate_distance_threshold:
                        candidate_too_close = True
                        #rospy.loginfo("Setting candidate too close to true")
                        break

            if candidate_too_close:
                #rospy.loginfo("Candidate too close, checking next one")
                continue

            pose = Point(item[0], item[1], item[2])
            global rrt_path
            rrt_path = False
            cand_pub.publish(pose)
            while not rrt_path:
                rospy.sleep(0.1)
            
            global rrt_poses
            if(rrt_poses[0][0] == -99 and rrt_poses[0][1] == -99 and rrt_poses[0][2] == -99):
                #rospy.loginfo("RRTConnect couldn't find path to candidate pose")
                continue

            rrt_poses = rrt_poses - centroid
            rrt_poses = rrt_poses / m
            #rrt_poses[:,1] *= -1

            item = item - centroid
            item = item / m
            #item[1] *= -1

            if first_check:
                first_check = False
                valid_candidate_poses = np.array([[item[0], item[1], item[2]]])
                pose_number = np.empty([rrt_poses.shape[0], 1])
                pose_number.fill(valid_count)

                trajectories = np.concatenate((pose_number, rrt_poses), axis=1)

            else:
                position = np.array([[item[0], item[1], item[2]]])
                valid_candidate_poses = np.concatenate((valid_candidate_poses, position), axis=0)

                pose_number = np.empty([rrt_poses.shape[0], 1])
                pose_number.fill(valid_count)

                pose_number = np.concatenate((pose_number, rrt_poses), axis=1)

                trajectories = np.concatenate((trajectories, pose_number), axis=0)

            valid_count = valid_count + 1


        if trajectories.shape[1] == 1:
            rospy.loginfo("Couldn't find candidate pose far enough away from already visited ones. Terminating.")
            return
            
        rospy.loginfo("Saving valid candidate poses and RRT Connect trajectories")
        np.save('/home/user/valid_candidate_poses.npy', valid_candidate_poses)
        np.save('/home/user/rrt_trajectories.npy', trajectories)

        rospy.loginfo("Find Next Best View with updated trajectory")
        os.system('python3 /home/user/Code/nbv_simulation/find_nbv_updated.py')

        next_pose = np.load('/home/user/next_candidate.npy')
        #next_pose[1] *= -1


        if first_candidate:
            first_candidate = False
            candidates_visited = np.array([[next_pose[0], next_pose[1], next_pose[2]]])
            nbv_poses = np.array([[next_pose[0], next_pose[1], next_pose[2]]])
            np.save('/home/user/Data/Reconstruction/visited_poses.npy', nbv_poses)
            #return
        else:
            current_visited = np.array([[next_pose[0], next_pose[1], next_pose[2]]])
            candidates_visited = np.concatenate((candidates_visited, current_visited), axis=0)
            visited_pose = np.array([[next_pose[0], next_pose[1], next_pose[2]]])
            nbv_poses = np.concatenate((nbv_poses, visited_pose))
            np.save('/home/user/Data/Reconstruction/visited_poses.npy', nbv_poses)


        next_pose = next_pose * m        
        next_pose = next_pose + centroid
        goal = airsim_moveit_navigation.msg.AirSim_NavigationGoal()

        goal.goal_pose.position.x = next_pose[0];
        goal.goal_pose.position.y = next_pose[1];
        goal.goal_pose.position.z = next_pose[2];
        goal.goal_pose.orientation.x = centroid[0];
        goal.goal_pose.orientation.y = centroid[1];
        goal.goal_pose.orientation.z = centroid[2];
        goal.goal_pose.orientation.w = 0;

        

        rospy.loginfo("Sending NBV to action server")
        client.send_goal(goal,
                         active_cb=callback_active,
                         feedback_cb=callback_feedback,
                         done_cb=callback_done)

        rospy.loginfo("Waiting for result and taking pointcloud observations on path")

        points=np.zeros((1,3))

        global uav_paused
        uav_paused = False
        global traj_traversed
        traj_traversed = False

        while not traj_traversed:
            
            if uav_paused:
                rospy.loginfo("Take observations")
                rospy.sleep(3)
                data = rospy.wait_for_message("/points", PointCloud2, timeout=None)

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

                pc = pypcd.PointCloud.from_msg(cloud_out)

                points_temp=np.zeros((pc.pc_data.shape[0],3))
                points_temp[:,0]=pc.pc_data['x']
                points_temp[:,1]=pc.pc_data['y']
                points_temp[:,2]=pc.pc_data['z']

                np.save('/home/user/airsim_trajectory_observation.npy', points_temp)
                rospy.loginfo("Filtering pointcloud")
                os.system('python3 ~/Code/PoinTr/filter_observations_pc.py')

                observation_filtered = np.load('/home/user/airsim_observation_filtered.npy')
                points = np.concatenate((points, observation_filtered), axis=0)
                
                uav_paused = False
            rospy.sleep(0.1)
            # Take observations

        rospy.loginfo("Trajectory traversed and observations complete")
        if(points.shape[0] > 1):
            points = np.delete(points, 0, 0)
            points = np.unique(points, axis=0)
            np.save('/home/user/observations.npy', points)

            rospy.loginfo("Downsampling total observed pointcloud")
            os.system('python3 ~/Code/PoinTr/voxel_filter_observations.py')

            observations_filtered = np.load('/home/user/observations.npy')
            all_filtered = np.load('/home/user/airsim_filtered.npy')
            all_filtered = np.concatenate((all_filtered, observations_filtered), axis=0)
            all_filtered = np.unique(all_filtered, axis=0)
            np.save('/home/user/airsim_filtered.npy', all_filtered)
        
        #rospy.loginfo("Waiting for result")
        #client.wait_for_result()



        #np.save('/home/user/airsim_filtered.npy', points)
        

        rospy.loginfo('Saving NBV images')
        rgb_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_color/Scene", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/rgb_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        rgb_segmented_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_segmentation/Segmentation", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(rgb_segmented_image, "bgr8")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/rgb_segmented_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        depth_image = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth/DepthPerspective", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(depth_image, "32FC1")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/depth_image_iteration_' + str(it_count) + '.jpeg', cv2_img)

        depth_segmented = rospy.wait_for_message("/airsim_node/drone_1/front_center_custom_depth_segmented/DepthSegmented", Image, timeout=None)
        cv2_img = bridge.imgmsg_to_cv2(depth_segmented, "32FC1")
        cv2.imwrite('/home/user/Data/Reconstruction/Images/depth_segmented_image_iteration_' + str(it_count) + '.jpeg', cv2_img)
        it_count += 1
        rospy.sleep(1)

        


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

