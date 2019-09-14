#!/usr/bin/env python

import sys
sys.path.insert( 0 , '~/catkin_ws/src/offboard_control/src')


import rospy
from qcontrol_defs.msg import *
from utils_functions import *
from geometry_msgs.msg import PoseStamped

def current_pos_callback(msg):
    global current_pos
    current_pos = msg.pose.position


# PVA control is  more accurate than just basics osition control
# since it considers time constraints between each waypoint and don't just 
# rely on the control loop delay implemented in PX4
if __name__ == "__main__":

    rospy.init_node('example_offboard_pvacontrol')
    quad_ros_namespace = ""
    xOffset = rospy.get_param('~xOffset')
    yOffset = rospy.get_param('~yOffset')
    # Create a publisher to send target positions
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, current_pos_callback)
    rospy.sleep(0.1)
    # current_pos = current_pos_msg.pose.position
    # Start position control with taking off at the beginning
    start_pva_control(quad_name= quad_ros_namespace , takeoff_before= False)
    # current_pos = Point(0,0,0)
    target1 = Point(198 , 600 , 10)
    target2 = Point(264, 602 , 10)
    target3 = Point(269.5, 594 , 10)
    target4 = Point(270, 423 , 10)
    target5 = Point(270, 407 , 10)
    # target6 = Point(324, 404 , 10)
    # target7 = Point(335.5, 453.5 , 10)
    # target8 = Point(449, 463 , 10)
    # target9 = Point(449, 606.5 , 10)
    # target10 = Point(,  , 10)
    time_full_traj = 180 #seconds
    freq = 20
    wait_rate = rospy.Rate(freq)

    x_traj = [current_pos.x , target1.x - xOffset , target2.x - xOffset, target3.x - xOffset, target4.x - xOffset, target5.x - xOffset]
    y_traj = [current_pos.y , target1.y - yOffset , target2.y - yOffset, target3.y - yOffset, target4.y - yOffset, target5.y - yOffset]
    z_traj = [current_pos.z , target1.z , target2.z, target3.z, target4.z, target5.z]
    pva_list = generate_traj_3d(x=x_traj , y=y_traj , z=z_traj , traj_time=[0,time_full_traj] , corr=None , freq = freq)
    
    # Send the generate traj to the vehicle
    for elem in pva_list.pva :
        send_pva_pub.publish(elem)
        wait_rate.sleep()
    
    # Land the vehicule
    start_landing(quad_name= quad_ros_namespace)
