#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np

Width = 512
Height = 512
CameraFOV = 90
Fx = Fy = Width / (2 * math.tan(CameraFOV * math.pi / 360))
Cx = Width / 2
Cy = Height / 2
Colour = (0, 255, 0)
RGB = "%d %d %d" % Colour # Colour for points
bridge = CvBridge()

def generatepointcloud(depth):
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = (depth > 0) & (depth < 255)
    z = 1000 * np.where(valid, depth / 256.0, np.nan)
    x = np.where(valid, z * (c - Cx) / Fx, 0)
    y = np.where(valid, z * (r - Cy) / Fy, 0)
    return np.dstack((x, y, z))


def savepointcloud(image, filename):
    f = open(filename, "w+")
    for x in range(image.shape[0]):
        for y in range(image.shape[1]):
            pt = image[x, y]
            if math.isinf(pt[0]) or math.isnan(pt[0]) or pt[0] > 10000 or pt[1] > 10000 or pt[2] > 10000:
                None
            else:
                f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2] - 1, RGB))
    f.close()

def callback(data):
    rospy.loginfo("Got image")
    depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
    np.save('chair_depth', depth_image)
    depth_image[depth_image > 60] = 60
    img2d = np.reshape(depth_image, (512, 512))
    pcl = generatepointcloud(img2d)
    savepointcloud(pcl, 'pcl.asc')
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/airsim_node/drone_1/front_center_custom_depth_segmented/DepthSegmented", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

