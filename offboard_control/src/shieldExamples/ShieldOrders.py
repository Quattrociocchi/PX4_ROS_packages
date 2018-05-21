#!/usr/bin/env python

import rospy
from qcontrol_defs.msg import *
# from nav_msgs.msg import Odometry












# ------------------------------------------------------------------ #

def curr_pos_callback(msg):
    global current_position
    current_position = msg.pose.pose.position






# ------------------------------------------------------------------ #

if __name__ == "__main__":

    shield_bool = Shield()
    rospy.Subscriber("/uav1/mavros/local_position/odom",Shield , curr_pos_callback)
    shield_bool = Shield_boolean_orders

	# Initialize node.
	rospy.init_node('Shield_xyz_Orders')

    # Should be the prefix added to the offboard control node (prefix of all mavros topics)
    # for example "" or "/Quad9" or "/Quad10"
    quad_ros_namespace = "/uav1"

    # Create a publisher to send target positions
    send_shield_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/shield_xyz_orders', Shield , queue_size=10)


