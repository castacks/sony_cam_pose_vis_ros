#!/usr/bin/env python3

import rospy
# import tf2_ros
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

def read_csv_file(csv_file):
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        next(reader) # Skip header row
        for row in reader:
            quaternion = [float(row[i]) for i in range(4)]
            translation = [float(row[i]) for i in range(4, 7)]
            yield quaternion, translation

if __name__ == '__main__':
    rospy.init_node('csv_tf_publisher')

    csv_file_path = rospy.get_param('~csv_file_path', 'data/frame_data.csv')

    odom_pub = rospy.Publisher('frame_cam', Odometry, queue_size=1)

    rate = rospy.Rate(10) # Set the publish rate to 2Hz

    while not rospy.is_shutdown():
        for quaternion, translation in read_csv_file(csv_file_path):
            # Normalize quaternion
            # quaternion = quaternion_norm(quaternion)

            # Create pose message
            pose = Pose()
            pose.position = Point(*translation)
            pose.orientation = Quaternion(*quaternion)

            # Create odometry message
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'frame'
            odom.pose.pose = pose

            # Publish odometry message
            odom_pub.publish(odom)

            print('.', end='', flush=True) # Print a dot to indicate progress

            rate.sleep()

    rospy.spin()

