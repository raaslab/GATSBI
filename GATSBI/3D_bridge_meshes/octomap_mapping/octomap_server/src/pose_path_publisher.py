#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

def read_poses_from_csv(csv_file_path):
    poses = []
    with open(csv_file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        first = True
        for row in reader:
            if first:
                first = False
                continue
            pose = Pose()
            pose.position.x = float(row[0])
            pose.position.y = float(row[1])
            pose.position.z = float(row[2])
            pose.orientation.x = float(row[3])
            pose.orientation.y = float(row[4])
            pose.orientation.z = float(row[5])
            pose.orientation.w = float(row[6])
            poses.append(pose)
    return poses

def read_points_from_csv(csv_file_path):
    points = []
    with open(csv_file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        first = True
        for row in reader:
            if first:
                first = False
                continue
            point = Point()
            point.x = float(row[0])
            point.y = float(row[1])
            point.z = float(row[2])
            points.append(point)
    return points

def publish_path_and_markers(poses, points, inspected):
    rospy.init_node('pose_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/gtsp_path', Path, queue_size=10)
    marker_pub = rospy.Publisher('/gtsp_marker_array', MarkerArray, queue_size=10)
    point_marker_pub = rospy.Publisher('/inspectable_marker_iteration', MarkerArray, queue_size=10)
    inspected_marker_pub = rospy.Publisher('/inspected_marker_iteration', MarkerArray, queue_size=10)


    rate = rospy.Rate(10)  # 10hz
    path = Path()
    path.header.frame_id = "world_enu"
    
    marker_array = MarkerArray()
    point_marker_array = MarkerArray()
    inspected_marker_array = MarkerArray()

    for idx, pose in enumerate(poses):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world_enu"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose
        path.poses.append(pose_stamped)
        
        marker = Marker()
        marker.header.frame_id = "world_enu"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "spheres"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)
    
    for idx, point in enumerate(points):
        marker = Marker()
        marker.header.frame_id = "world_enu"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "squares"
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.9
        marker.scale.y = 0.9
        marker.scale.z = 0.9
        marker.color.a = 0.6
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        point_marker_array.markers.append(marker)

    for idx, point in enumerate(inspected):
        marker = Marker()
        marker.header.frame_id = "world_enu"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "squares"
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        inspected_marker_array.markers.append(marker)


    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        for marker in marker_array.markers:
            marker.header.stamp = rospy.Time.now()
        for marker in point_marker_array.markers:
            marker.header.stamp = rospy.Time.now()
        for marker in inspected_marker_array.markers:
            marker.header.stamp = rospy.Time.now()

        path_pub.publish(path)
        marker_pub.publish(marker_array)
        point_marker_pub.publish(point_marker_array)
        inspected_marker_pub.publish(inspected_marker_array)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        csv_file_path = '/home/user/GATSBI_Journal/ICUAS_7/gtsp_data_loop_2.csv'
        point_csv_file_path = '/home/user/GATSBI_Journal/ICUAS_7/inspectable_voxels_data_loop_2.csv'
        inspected_file_path = '/home/user/GATSBI_Journal/ICUAS_7/inspected_voxels_data_loop_2.csv'
        poses = read_poses_from_csv(csv_file_path)
        points = read_points_from_csv(point_csv_file_path)
        inspected = read_points_from_csv(inspected_file_path)

        publish_path_and_markers(poses, points, inspected)
    except rospy.ROSInterruptException:
        pass
