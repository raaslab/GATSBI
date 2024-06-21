#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
import os

class CameraInfoSubscriber:
    def __init__(self):
        rospy.init_node('camera_info_subscriber', anonymous=True)
        self.camera_info_sub = rospy.Subscriber("/zed/zed_node/left/camera_info", CameraInfo, self.camera_info_callback)
        self.save_directory = rospy.get_param("~save_directory", os.path.expanduser('~'))
        self.file_path = os.path.join(self.save_directory, "camera_info.txt")

    def camera_info_callback(self, data):
        try:
            with open(self.file_path, 'w') as file:
                file.write("Camera Info:\n")
                file.write("Width: {}\n".format(data.width))
                file.write("Height: {}\n".format(data.height))
                file.write("Distortion Model: {}\n".format(data.distortion_model))
                file.write("D: {}\n".format(data.D))
                file.write("K: {}\n".format(data.K))
                file.write("R: {}\n".format(data.R))
                file.write("P: {}\n".format(data.P))
                file.write("Bin: {}\n".format(data.binning_x))
                file.write("Bin: {}\n".format(data.binning_y))
            rospy.loginfo("Camera info saved to %s" % self.file_path)
        except Exception as e:
            rospy.logerr("Error saving camera info: %s" % str(e))

if __name__ == '__main__':
    try:
        camera_info_subscriber = CameraInfoSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
