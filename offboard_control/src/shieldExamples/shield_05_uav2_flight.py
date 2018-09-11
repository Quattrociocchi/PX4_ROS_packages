#!/usr/bin/env python


import rospy
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
import time


def trajComplete_callback(msg):
    global trajComplete2
    trajComplete2 = True


if __name__ == "__main__":
    # Initialize Node.
    rospy.init_node('Quad_offboard_pvacontrol_2')
    # Set namespaces
    quad_ros_namespace2 = "/uav2"
    # Start by taking off
    rospy.sleep(0.5)
    start_pva_control(quad_name= quad_ros_namespace2 , takeoff_before= True)
    rospy.sleep(0.5)
    # Set up action publishers
    action_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/shield_action_orders', ShieldOutput , queue_size=10)
    # Action list
    action_list_2 = [16, 0, 17, 0, 8, 0, 13, 16, 16, 17]
    # Create action messages
    action_msg_2 = ShieldOutput()
    # Publish first action -----> comment out next 3 lines to use terminal input
    action_msg_2.action = [action_list_2[0]]
    rospy.sleep(0.5)
    action_pub_2.publish(action_msg_2)
    # Set up subscriber for completed trajectory flag
    rospy.Subscriber(quad_ros_namespace2 + '/trajComplete', ShieldBool, trajComplete_callback)

    # # Uncomment to use terminal input
    # global trajComplete2
    # trajComplete2 = True

    trash = input("Enter any number to begin: ")

    counter = 1
    while not rospy.is_shutdown():
        if 'trajComplete2' in globals():
            # # Uncomment for terminal control
            # data = input("Enter the action you would like the quad to perform:")
            # action_msg_2.action = [data]

            action_msg_2.action = [action_list_2[counter]]
            # rospy.sleep(0.5)
            action_pub_2.publish(action_msg_2)

            counter = counter + 1
            del globals()['trajComplete2']
        else:
            pass

#     # Land the vehicle
#     start_landing(quad_name= quad_ros_namespace1)
#     start_landing(quad_name= quad_ros_namespace2)