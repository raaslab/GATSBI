#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.save_directory = rospy.get_param("~save_directory", os.path.expanduser('~'))

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")

            # Normalize the depth image to a range suitable for saving as an image file (0-255)
            #normalized_depth_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)



        except Exception as e:
            rospy.logerr("Error converting ROS Image to OpenCV image: %s" % str(e))
            return

        filename = os.path.join(self.save_directory, "image_%s.png" % rospy.Time.now().to_nsec())

        try:
            cv2.imwrite(filename, cv_image)
            rospy.loginfo("Image saved as %s" % filename)
        except Exception as e:
            rospy.logerr("Error saving image: %s" % str(e))

if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
