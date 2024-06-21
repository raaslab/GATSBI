#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import time

tf_buffer = 0
listener = 0


def transform_pose(input_pose, from_frame, to_frame):
    global tf_buffer
    global listener
    try:
        # Wait for the transformation to be available
        tf_buffer.can_transform(to_frame, from_frame, rospy.Time(), rospy.Duration(3.0))

        # Lookup the transformation
        transform = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time())

        # Transform the pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, transform)
        return transformed_pose

    except tf2_ros.LookupException as e:
        rospy.logerr('Transform lookup failed: %s' % str(e))
    except tf2_ros.ConnectivityException as e:
        rospy.logerr('Transform connectivity failed: %s' % str(e))
    except tf2_ros.ExtrapolationException as e:
        rospy.logerr('Transform extrapolation failed: %s' % str(e))

def path_callback(path_msg):
    rospy.loginfo("Received path message with %d poses", len(path_msg.poses))

    # Initialize the output path message
    output_path = Path()
    output_path.header = path_msg.header

    current_pose = PoseStamped()

    count = 0
    for pose in path_msg.poses:
        if count < 25:
            count = count + 1
            continue
        count = 0
        # Append each pose to the output path
        output_path.poses.append(pose)

        current_pose = pose
        

        from_frame = 'world_ned'
        to_frame = 'world_enu'
        current_pose = transform_pose(pose, from_frame, to_frame)

        current_pose.pose.position = pose.pose.position
        current_pose.header = path_msg.header           
        # Publish the current output path
        path_publisher.publish(output_path)
        pose_publisher.publish(current_pose)
        #rospy.loginfo("%.2f, %.2f, %.2f", pose.position.x, pose.position.y, pose.position.z)
        rospy.loginfo("Published path with %d poses", len(output_path.poses))
        # Delay of 0.1 seconds between each pose
        time.sleep(0.0005)

def main():
    # Initialize the ROS node
    rospy.init_node('path_republisher', anonymous=True)

    # Subscribe to the input path topic
    rospy.Subscriber('/path', Path, path_callback)

    # Publisher
    global path_publisher
    global pose_publisher
    global tf_buffer
    global listener

    path_publisher = rospy.Publisher('/path_delayed', Path, queue_size=10)
    pose_publisher = rospy.Publisher('/pose_delayed', PoseStamped, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("Path Republisher Node Initialized")

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass