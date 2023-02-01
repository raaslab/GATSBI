import os
# from show_results__ import*
# from tqdm import tqdm
# import torch
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from filelock import FileLock
import numpy as np

source_image_dir = '../deep_lab_v3_material_detection/predicted_masks/'
pub = rospy.Publisher('image_mask', Image, queue_size = 10)
rospy.init_node('image_mask_node')

#for image_name in tqdm(os.listdir(source_image_dir)):
#print(image_name)
color = (0,0,0)
image_path = source_image_dir + 'test.jpeg'
lock = FileLock(image_path + '.lock')
while True:
    with lock:
        rospy.loginfo("File Locked")
        print(image_path)
        image = cv2.imread(image_path)
        width, height = image.shape[1], image.shape[0]
        mid_x, mid_y = int(width/2), int(height/2)
        cw2, ch2 = 640-width, 480-height
        border = cv2.copyMakeBorder(image,0,0,int(cw2/2),int(cw2/2),borderType = cv2.BORDER_CONSTANT, value = [0,0,0])
        width, height = border.shape[1], border.shape[0]
        mid_y = int(height/2)
        #borderNew = border[mid_y-(ch2/2):mid_y+(ch2/2),0:width]
        borderNew = border[0:height-32,0:width]
        
        #border = cv2.resize(border,(512,512))
        #borderNew = border

        bridge = CvBridge()
        # image_msg = bridge.cv2_to_imgmsg(image, "passthrough")
        cv2.imwrite("zzz4.jpeg",borderNew)
        image_msg = bridge.cv2_to_imgmsg(borderNew, 'bgr8')
        image_msg.header.stamp = rospy.Time.now()
        pub.publish(image_msg)
        rospy.loginfo('Publishing images')
    cv2.waitKey(50)

