#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np

predicted_voxel_centers = np.load('/home/user/predicted_voxel_centers.npy')
real_voxel_centers = np.load('/home/user/real_voxel_centers.npy')

def talker():

    rospy.init_node('marker', anonymous=True)
    rate = rospy.Rate(0.5)

    predicted_topic = 'airsim_predicted_voxel_grid'
    real_topic = 'airsim_real_voxel_grid'

    predicted_publisher = rospy.Publisher(predicted_topic, MarkerArray, queue_size=10)
    real_publisher = rospy.Publisher(real_topic, MarkerArray, queue_size=10)

    predictedMarkerArray = MarkerArray()

    count = 0
    for item in predicted_voxel_centers:
        marker = Marker()
        #marker.header.frame_id = "/world_enu"
        marker.header.frame_id = "/front_center_custom_depth_optical/static"
        marker.id = count
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = item[0]
        marker.pose.position.y = item[1]
        marker.pose.position.z = item[2]

        predictedMarkerArray.markers.append(marker)
        count = count + 1

    realMarkerArray = MarkerArray()
    count = 0
    for item in real_voxel_centers:
        marker = Marker()
        #marker.header.frame_id = "/world_enu"
        marker.header.frame_id = "/front_center_custom_depth_optical/static"
        marker.id = count
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = item[0]
        marker.pose.position.y = item[1]
        marker.pose.position.z = item[2]

        realMarkerArray.markers.append(marker)
        count = count + 1

    while not rospy.is_shutdown():
        predicted_publisher.publish(predictedMarkerArray)
        real_publisher.publish(realMarkerArray)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass