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
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import struct

predict_pointcloud = False

pub = rospy.Publisher('/nbv_position_ready', Bool, queue_size=10)

pc_pub = rospy.Publisher('/predicted_pointcloud', PointCloud2, queue_size=10)


def callback(data):
    global predict_pointcloud
    if predict_pointcloud:
        rospy.loginfo("Got pointcloud")
        pc = pypcd.PointCloud.from_msg(data)
        pc.save('/home/user/airsim.pcd')

        points=np.zeros((pc.pc_data.shape[0],3))
        points[:,0]=pc.pc_data['x']
        points[:,1]=pc.pc_data['y']
        points[:,2]=pc.pc_data['z']
        
        # Center and normalize pointcloud (going to restore it after prediction)

        centroid = np.mean(points, axis=0)
        points = points - centroid
        m = np.max(np.sqrt(np.sum(points**2, axis=1)))
        points = points / m
        

        np.save('/home/user/input.npy', points)
        np.save('/home/user/Code/PoinTr/data/ShapeNet55-34/shapenet_pc/02691156-airplane.npy', points)

        #rospy.loginfo("Predicting pointcloud using PoinTr")
        #os.system('bash /home/user/Code/PoinTr/scripts/test.sh 0 --ckpts /home/user/Code/PoinTr/pretrained/rotation_mini_translation.pth --config /home/user/Code/PoinTr/cfgs/ShapeNet55_models/PoinTr.yaml --mode easy --exp_name test_example')

        predicted = np.load("/home/user/dense.npy")
        predicted = predicted.reshape((predicted.shape[1], predicted.shape[2]))
        predicted = predicted * m        
        predicted = predicted + centroid

        predicted_data = predicted

        rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
        rgb_col = np.empty(predicted.shape[0])
        rgb_col.fill(rgb)

        predicted = np.hstack((predicted, rgb_col.reshape(rgb_col.shape[0], 1)))
        header = data.header
        header.stamp = rospy.Time.now()

        points_out = []
        for i in range(0, predicted.shape[0]):
            points_out.append(predicted[i])
        #points_out.append(predicted.tolist())
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 16, PointField.UINT32, 1),
                 ]

        predictedPC = point_cloud2.create_cloud(header, fields, points_out)
        rospy.loginfo("Publish predicted pointcloud")
        pc_pub.publish(predictedPC)

        partial_data = np.load("/home/user/partial.npy")
        partial_data = partial_data.reshape((partial_data.shape[1], partial_data.shape[2]))
        partial_data = partial_data * m        
        partial_data = partial_data + centroid


        np.save("/home/user/only_input.npy", partial_data)


        for item in partial_data:
            idx = np.argwhere(predicted_data == item)
            predicted_data = np.delete(predicted_data, idx[0][0], axis=0)


        np.save("/home/user/only_predicted.npy", predicted_data)

        rospy.loginfo("Finding next best view and saving NBV position")
        os.system('python3 ~/Code/NBV_Reconstruction/nbv_pointr.py')


        rospy.loginfo("Publishing that NBV position is ready")
        global pub
        pub_msgs = Bool()
        pub_msgs.data = True
        pub.publish(pub_msgs)
        predict_pointcloud = False

    
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

