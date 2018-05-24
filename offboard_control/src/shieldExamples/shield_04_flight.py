#!/usr/bin/env python


import rospy
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
import time


def trajComplete_callback1(msg):
    global trajComplete1
    trajComplete1 = True

def trajComplete_callback2(msg):
    global trajComplete2
    trajComplete2 = True

def trajComplete_callback3(msg):
    global trajComplete3
    trajComplete3 = True

def trajComplete_callback4(msg):
    global trajComplete4
    trajComplete4 = True


if __name__ == "__main__":
    # Initialize Node.
    rospy.init_node('Quad_offboard_pvacontrol')
    # Set namespaces
    quad_ros_namespace1 = "/uav1"
    quad_ros_namespace2 = "/uav2"
    quad_ros_namespace3 = "/uav3"
    quad_ros_namespace4 = "/uav4"
    # Start by taking off
    start_pva_control(quad_name= quad_ros_namespace1 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace2 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace3 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace4 , takeoff_before= True)
    # Set up action publishers
    action_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/shield_action_orders', ShieldOutput , queue_size=10)
    action_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/shield_action_orders', ShieldOutput , queue_size=10)
    action_pub_3 = rospy.Publisher(quad_ros_namespace3 + '/shield_action_orders', ShieldOutput , queue_size=10)
    action_pub_4 = rospy.Publisher(quad_ros_namespace4 + '/shield_action_orders', ShieldOutput , queue_size=10)
    # Action list
    action_list_1 = [5, 14, 1, 15]
    action_list_2 = [13, 15, 8, 14]
    action_list_3 = [14, 14, 9, 3]
    action_list_4 = [14, 14, 14, 7]
    # Create action messages
    action_msg_1 = ShieldOutput()
    action_msg_1.action = [action_list_1[0]]
    action_msg_2 = ShieldOutput()
    action_msg_2.action = [action_list_2[0]]
    action_msg_3 = ShieldOutput()
    action_msg_3.action = [action_list_3[0]]
    action_msg_4 = ShieldOutput()
    action_msg_4.action = [action_list_4[0]]

    # publish first action
    rospy.sleep(0.5)
    action_pub_1.publish(action_msg_1)
    action_pub_2.publish(action_msg_2)
    action_pub_3.publish(action_msg_3)
    action_pub_4.publish(action_msg_4)
    
    rospy.Subscriber(quad_ros_namespace1 + '/trajComplete', ShieldBool, trajComplete_callback1)
    rospy.Subscriber(quad_ros_namespace2 + '/trajComplete', ShieldBool, trajComplete_callback2)
    rospy.Subscriber(quad_ros_namespace3 + '/trajComplete', ShieldBool, trajComplete_callback3)
    rospy.Subscriber(quad_ros_namespace4 + '/trajComplete', ShieldBool, trajComplete_callback4)

    counter = 1
    while not rospy.is_shutdown():
        if 'trajComplete1' in globals() and 'trajComplete2' in globals() and 'trajComplete3' in globals() and 'trajComplete4' in globals():
            action_msg_1.action = [action_list_1[counter]]
            action_msg_2.action = [action_list_2[counter]]
            action_msg_3.action = [action_list_3[counter]]
            action_msg_4.action = [action_list_4[counter]]

            action_pub_1.publish(action_msg_1)
            action_pub_2.publish(action_msg_2)
            action_pub_3.publish(action_msg_3)
            action_pub_4.publish(action_msg_4)

            counter = counter + 1
            del globals()['trajComplete1']
            del globals()['trajComplete2']
            del globals()['trajComplete3']
            del globals()['trajComplete4']
        else:
            pass

#     # Land the vehicle
#     start_landing(quad_name= quad_ros_namespace1)
#     start_landing(quad_name= quad_ros_namespace2)