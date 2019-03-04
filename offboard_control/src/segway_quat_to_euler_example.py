#!/usr/bin/env python

import math
import rospy
import subprocess
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation(msg):
    global roll, pitch, yaw, quat
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    quat = quaternion_from_euler(roll,pitch,yaw)

if __name__ == "__main__":
    rospy.init_node('test_conversion')
    rospy.sleep(0.5)
    rospy.Subscriber('/segway/odometry/local_filtered', Odometry, get_rotation)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        if 'yaw' in globals():
            print math.degrees(yaw)
            print quat
            subprocess.call(["clear"])