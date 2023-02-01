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


predict_pointcloud = False

pub = rospy.Publisher('/nbv_position_ready', Bool, queue_size=10)

"""
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


def convert_asc_to_pcd():
    filename = 'pcl.asc'
    file = open(filename,"r+")
    count = 0
    # Statistical Source File Points
    for line in file:
        count=count+1
    file.close()

    #output = open("out.pcd","w+")
    f_prefix = filename.split('.')[0]
    output_filename = '{prefix}.pcd'.format(prefix=f_prefix)
    output = open(output_filename,"w+")

    list = ['# .PCD v.5 - Point Cloud Data file format\n','VERSION .5\n','FIELDS x y z\n','SIZE 4 4 4\n','TYPE F F F\n','COUNT 1 1 1\n']

    output.writelines(list)
    output. write ('WIDTH') # Note that there are spaces behind it
    output.write(str(count))
    output.write('\nHEIGHT')
    output.write(str (1))# mandatory type conversion, file input can only be STR format
    output.write('\nPOINTS ')
    output.write(str(count))
    output.write('\nDATA ascii\n')
    file1 = open(filename,"r")
    all = file1.read()
    output.write(all)
    output.close()
    file1.close()

    os.system('python3 ~/Code/Depth_to_Voxel/depth_to_voxel.py')

    os.system('python2 ~/Code/3D-RecGAN-extended/demo_3D-RecGAN++.py')


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

    convert_asc_to_pcd()
"""

def callback(data):
    global predict_pointcloud
    if predict_pointcloud:
        rospy.loginfo("Got pointcloud")
        pc = pypcd.PointCloud.from_msg(data)
        pc.save('/home/user/airsim.pcd')

        rospy.loginfo("Converting segmented depth image to voxel grid in 3DRecGAN++ required format")
        os.system('python3 ~/Code/Depth_to_Voxel/depth_to_voxel.py')

        rospy.loginfo("Predicting voxel grid using 3DRecGAN++")
        os.system('python2 ~/Code/3D-RecGAN-extended/demo_3D-RecGAN++.py')

        rospy.loginfo("Saving predicted voxel grid centers")
        os.system('python2 ~/Code/3D-RecGAN-extended/save_predicted_voxel_centers.py')

        rospy.loginfo("Finding next best view and saving NBV position")
        os.system('python3 ~/Code/NBV_Reconstruction/nbv.py')

        rospy.loginfo("Publishing that NBV position is ready")
        global pub
        pub_msgs = Bool()
        pub_msgs.data = True
        pub.publish(pub_msgs)
        predict_pointcloud = False

    #depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
    #np.save('chair_depth', depth_image)
    #depth_image[depth_image > 60] = 60
    #img2d = np.reshape(depth_image, (512, 512))
    #pcl = generatepointcloud(img2d)

    #savepointcloud(pcl, 'pcl.asc')
    
def bool_callback(data):
    rospy.loginfo("Got message to predict pointcloud")

    global predict_pointcloud
    predict_pointcloud = data.data


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/points", PointCloud2, callback)

    rospy.Subscriber("/predict_pointcloud", Bool, bool_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

