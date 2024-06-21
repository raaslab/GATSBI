#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from octomap_msgs.msg import Octomap
from octomap_msgs.msg import Octomap
from octomap_msgs import conversions
import numpy as np

class OctomapSubscriber:
    def __init__(self):
        rospy.init_node('octomap_subscriber', anonymous=True)
        rospy.Subscriber("/octomap_full", Octomap, self.octomap_callback)
        self.marker_pub = rospy.Publisher('/free_cells_markers', MarkerArray, queue_size=10)
        self.rate = rospy.Rate(1) # Adjust this rate as per your requirement

    def octomap_callback(self, octomap_msg):
        octree = conversions.msg_to_octree(octomap_msg)
        marker_array = MarkerArray()
        i = 0
        for leaf in octree:
            if octree.isNodeFree(leaf):
                marker = Marker()
                marker.header.frame_id = octomap_msg.header.frame_id
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.pose.position.x = leaf.x
                marker.pose.position.y = leaf.y
                marker.pose.position.z = leaf.z
                marker.scale.x = octree.resolution()
                marker.scale.y = octree.resolution()
                marker.scale.z = octree.resolution()
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.id = i
                marker_array.markers.append(marker)
                i += 1
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        octomap_subscriber = OctomapSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
