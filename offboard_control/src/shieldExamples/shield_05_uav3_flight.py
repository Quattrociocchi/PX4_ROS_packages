#!/usr/bin/env python


import rospy
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
import time


def trajComplete_callback(msg):
    global trajComplete3
    trajComplete3 = True


if __name__ == "__main__":
    # Initialize Node.
    rospy.init_node('Quad_offboard_pvacontrol_3')
    # Set namespaces
    quad_ros_namespace3 = "/uav3"
    # Start by taking off
    rospy.sleep(0.5)
    start_pva_control(quad_name= quad_ros_namespace3 , takeoff_before= True)
    rospy.sleep(0.5)
    # Set up action publishers
    action_pub_3 = rospy.Publisher(quad_ros_namespace3 + '/shield_action_orders', ShieldOutput , queue_size=10)
    # Action list
    action_list_3 = [15, 5, 15, 16, 13, 16, 17, 16, 16, 15]
    # Create action messages
    action_msg_3 = ShieldOutput()
    # Publish first action -----> comment out next 3 lines to use terminal input
    action_msg_3.action = [action_list_3[0]]
    rospy.sleep(0.5)
    action_pub_3.publish(action_msg_3)
    # Set up subscriber for completed trajectory flag
    rospy.Subscriber(quad_ros_namespace3 + '/trajComplete', ShieldBool, trajComplete_callback)

    # # Uncomment to use terminal input
    # global trajComplete3
    # trajComplete3 = True

    trash = input("Enter any number to begin: ")

    counter = 1
    while not rospy.is_shutdown():
        if 'trajComplete3' in globals():
            # # Uncomment for terminal control
            # data = input("Enter the action you would like the quad to perform:")
            # action_msg_3.action = [data]

            action_msg_3.action = [action_list_3[counter]]
            # rospy.sleep(0.5)
            action_pub_3.publish(action_msg_3)

            counter = counter + 1
            del globals()['trajComplete3']
        else:
            pass

#     # Land the vehicle
#     start_landing(quad_name= quad_ros_namespace3)
#     start_landing(quad_name= quad_ros_namespace2)