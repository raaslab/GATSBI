#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
import tf2_ros
import tf2_geometry_msgs

class PosePublisher:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/uav_pose', PoseStamped, queue_size=10)
        self.csv_file_path = '/home/user/bridgeInspection/AirsimInspectionPath.csv'
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(50)
        self.read_and_publish_poses()
        

    def transform_pose(self, input_pose, from_frame, to_frame):
        try:
            # Wait for the transformation to be available
            self.tf_buffer.can_transform(to_frame, from_frame, rospy.Time(), rospy.Duration(3.0))

            # Lookup the transformation
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time())

            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, transform)
            return transformed_pose

        except tf2_ros.LookupException as e:
            rospy.logerr('Transform lookup failed: %s' % str(e))
        except tf2_ros.ConnectivityException as e:
            rospy.logerr('Transform connectivity failed: %s' % str(e))
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr('Transform extrapolation failed: %s' % str(e))

    def read_and_publish_poses(self):
        while not rospy.is_shutdown():
            with open(self.csv_file_path, 'r') as csv_file:
                csv_reader = csv.DictReader(csv_file)
                for i, row in enumerate(csv_reader):
                    if i % 25 == 0:
                        print(i)
                        pose = PoseStamped()

                        pose.pose.orientation.x = float(row['ori_x'])
                        pose.pose.orientation.y = float(row['ori_y'])
                        pose.pose.orientation.z = float(row['ori_z'])
                        pose.pose.orientation.w = float(row['ori_w'])

                        from_frame = 'world_ned'
                        to_frame = 'world_enu'
                        pose = self.transform_pose(pose, from_frame, to_frame)
                        pose.pose.position.x = float(row['x'])
                        pose.pose.position.y = float(row['y'])
                        pose.pose.position.z = float(row['z'])
                        
                        pose.header.seq = int(row['seq'])
                        pose.header.stamp = rospy.Time.from_sec(float(row['stamp']))
                        pose.header.frame_id = row['frame_id']

                        #rospy.loginfo(f"Publishing pose: {pose}")
                        self.pose_pub.publish(pose)
                        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pose_publisher', anonymous=True)
    pose_publisher = PosePublisher()
    rospy.spin()
