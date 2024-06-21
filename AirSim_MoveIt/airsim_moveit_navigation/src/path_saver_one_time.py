#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class PathSaver:
    def __init__(self):
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)
        self.gatsbi_sub = rospy.Subscriber('/gatsbi_done', Bool, self.gatsbi_callback)
        self.baseline_sub = rospy.Subscriber('/baseline_done', Bool, self.baseline_callback)
        self.csv_file = open('/home/user/bridgeInspection/AirsimInspectionPath.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['seq', 'frame_id', 'stamp', 'x', 'y', 'z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])
        self.gatsbi_done = False

    def path_callback(self, msg):
        if(self.gatsbi_done):
            for pose in msg.poses:
                seq = msg.header.seq
                frame_id = msg.header.frame_id
                stamp = msg.header.stamp.to_sec()
                x = pose.pose.position.x
                y = pose.pose.position.y
                z = pose.pose.position.z
                ori_x = pose.pose.orientation.x
                ori_y = pose.pose.orientation.y
                ori_z = pose.pose.orientation.z
                ori_w = pose.pose.orientation.w
                self.csv_writer.writerow([seq, frame_id, stamp, x, y, z, ori_x, ori_y, ori_z, ori_w])

            rospy.loginfo("Path saved to CSV. Shutting down.")
            self.shutdown()

    def gatsbi_callback(self, msg):
        rospy.loginfo("Received GATSBI done message.")
        self.gatsbi_done = msg.data

    def baseline_callback(self, msg):
        rospy.loginfo("Received Baseline done message.")
        self.gatsbi_done = msg.data

    def shutdown(self):
        self.path_sub.unregister()
        self.csv_file.close()
        rospy.signal_shutdown("Received one message and saved to CSV.")

if __name__ == '__main__':
    rospy.init_node('path_saver', anonymous=True)
    path_saver = PathSaver()
    rospy.spin()
