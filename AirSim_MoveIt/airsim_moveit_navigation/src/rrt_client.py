#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
import csv
import sys

terminate = False

def callback(data):
	rospy.loginfo("Got RRT path")
	global terminate
	terminate = True

	with open('/home/user/rrt_path.csv', 'w') as f:
		writer = csv.writer(f)

		for item in data.poses:
			position = [item.position.x, item.position.y, item.position.z]
			writer.writerow(position)

def rrt_client(x, y, z):
    rospy.init_node('rrt_client', anonymous=True)
    pub = rospy.Publisher('compute_rrt_path', Point, queue_size=1)
    rospy.Subscriber("rrt_path", PoseArray, callback)
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
		global terminate
		if terminate:
			break

		pub.publish(Point(float(x), float(y), float(z)))

		rate.sleep()


if __name__ == '__main__':
    try:
        rrt_client(sys.argv[1], sys.argv[2], sys.argv[3])
    except rospy.ROSInterruptException:
        pass