#!/usr/bin/env python

import rospy
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
from qcontrol_defs.msg import *
from utils_functions import *
import time
from std_msgs.msg import Bool


def trajComplete_callback(msg):
    if msg.data:
        global trajComplete1
        trajComplete1 = True

if __name__ == "__main__":
    # Initialize Node.
    rospy.init_node('Quad_offboard_pvacontrol_1')
    # Set namespaces
    quad_ros_namespace1 = "/uav1"
    # Start by taking off
    rospy.sleep(0.5)
    start_pva_control(quad_name= quad_ros_namespace1 , takeoff_before= True)
    rospy.sleep(0.5)
    # Set up action publishers
    action_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/shield_action_orders', ShieldOutput , queue_size=10)

    # Action list. Use this for list input
    action_list_1 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]

    # Create action messages
    action_msg_1 = ShieldOutput()

    # # Publish first action -----> Uncomment next 3 lines for list input
    # action_msg_1.action = [action_list_1[0]]
    # rospy.sleep(0.5)
    # action_pub_1.publish(action_msg_1)

    # Set up subscriber for completed trajectory flag
    rospy.Subscriber(quad_ros_namespace1 + '/trajComplete', Bool, trajComplete_callback)

    global trajComplete1
    trajComplete1 = True

    trash = input("Enter any number to begin: ")

    counter = 0
    while not rospy.is_shutdown():
        if 'trajComplete1' in globals():
            # # Uncomment for terminal control
            # data = input("Enter the action you would like the quad to perform:")
            # action_msg_1.action = [data]

            # Uncomment the following 2 lines to use list input
            action_msg_1.action = [action_list_1[counter]]
            counter += 1

            trash = input("Enter any number to begin the next agent move: ")

            action_pub_1.publish(action_msg_1)
            del globals()['trajComplete1']
        else:
            pass

    # Land the vehicle
    start_landing(quad_name= quad_ros_namespace1)