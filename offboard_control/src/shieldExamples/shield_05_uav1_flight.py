#!/usr/bin/env python


import rospy
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
import time


def trajComplete_callback(msg):
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
    # Action list
    action_list_1 = [16]
    # Create action messages
    action_msg_1 = ShieldOutput()
    # Set up subscriber for completed trajectory flag
    rospy.Subscriber(quad_ros_namespace1 + '/trajComplete', ShieldBool, trajComplete_callback)

    global trajComplete1
    trajComplete1 = True

    counter = 0
    while not rospy.is_shutdown():
        if 'trajComplete1' in globals():
            # Uncomment for terminal control
            data = input("Enter the action you would like the quad to perform:")
            action_msg_1.action = [data]

            # action_msg_1.action = [action_list_1[counter]]
            rospy.sleep(0.5)
            action_pub_1.publish(action_msg_1)

            counter = counter + 1
            del globals()['trajComplete1']
        else:
            pass

#     # Land the vehicle
#     start_landing(quad_name= quad_ros_namespace1)
#     start_landing(quad_name= quad_ros_namespace2)