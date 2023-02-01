#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
import pypcd

def callback(data):
    rospy.loginfo("Got pointcloud")
    pc = pypcd.PointCloud.from_msg(data)
    pc.save('/home/user/airsim.pcd')
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

