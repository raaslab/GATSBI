#!/usr/bin/env python
import rospy
import tf
import csv

from nav_msgs.msg import Path
from std_msgs.msg import Header

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

path_pub = rospy.Publisher('/dji_path', Path, queue_size=10)
path = Path()

f_out = open('/home/user/dji_path.csv', 'w')
writer = csv.writer(f_out)
check_rate = 0
def gotposition(drone_position, drone_orientation):
    #print("Got drone position and orientation")
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "world"



    path.header = h
    pose = PoseStamped()
    pose.header = h
    pose.pose.position.x = drone_position.point.x
    pose.pose.position.y = drone_position.point.y
    pose.pose.position.z = drone_position.point.z
    pose.pose.orientation.x = drone_orientation.quaternion.x
    pose.pose.orientation.y = drone_orientation.quaternion.y
    pose.pose.orientation.z = drone_orientation.quaternion.z
    pose.pose.orientation.w = drone_orientation.quaternion.w
    path.poses.append(pose)
    path_pub.publish(path)

    global check_rate
    if check_rate >= 10:
        global writer
        row = [drone_position.point.x, drone_position.point.y, (drone_position.point.z-1)]
        writer.writerow(row)

    check_rate = check_rate + 1

def talker():
    rospy.init_node('dji_path_node')

    '''
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "world"

    path = Path()

    path.header = h
    pose = PoseStamped()

    count = 0
    for row in reader:
        if count == 0:
            count = 1
            continue
        pose = PoseStamped()
        pose.header = h
        pose.pose.position.x = float(row[0])
        pose.pose.position.y = float(row[1])
        pose.pose.position.z = float(row[2])
        converted = tf.transformations.quaternion_from_euler(0,0,float(row[3]))
        pose.pose.orientation.x = converted[0]
        pose.pose.orientation.y = converted[1]
        pose.pose.orientation.z = converted[2]
        pose.pose.orientation.w = converted[3]
        path.poses.append(pose)
        print(row)

    
    '''
    rate = rospy.Rate(10)
    

    position_sub = Subscriber("/dji_sdk/local_position", PointStamped)
    orientation_sub = Subscriber("/dji_sdk/attitude", QuaternionStamped)

    ats = ApproximateTimeSynchronizer([position_sub, orientation_sub], queue_size=5, slop=0.1)
    ats.registerCallback(gotposition)
    while not rospy.is_shutdown():
        #path_pub.publish(path)
        rate.sleep()

    f_out.close()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass