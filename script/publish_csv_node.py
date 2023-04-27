#!/usr/bin/env python

import rospy
import tf2_ros
import csv
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_multiply, quaternion_norm, quaternion_conjugate

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

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(2) # Set the publish rate to 2Hz

    while not rospy.is_shutdown():
        for quaternion, translation in read_csv_file('frame_data.csv'):
            # Normalize quaternion
            quaternion = quaternion_norm(quaternion)

            # Create transform message
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = 'world'
            transform_msg.child_frame_id = 'frame'
            transform_msg.transform.translation.x = translation[0]
            transform_msg.transform.translation.y = translation[1]
            transform_msg.transform.translation.z = translation[2]
            transform_msg.transform.rotation.x = quaternion[0]
            transform_msg.transform.rotation.y = quaternion[1]
            transform_msg.transform.rotation.z = quaternion[2]
            transform_msg.transform.rotation.w = quaternion[3]

            # Publish transform message
            tf_broadcaster.sendTransform(transform_msg)

            rate.sleep()

    rospy.spin()

