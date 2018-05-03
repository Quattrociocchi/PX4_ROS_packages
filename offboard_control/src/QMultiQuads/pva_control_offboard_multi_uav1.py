#!/usr/bin/env python

import sys
sys.path.insert( 0 , '~/catkin_ws/src/offboard_control/src')


import rospy
from qcontrol_defs.msg import *
from utils_functions import *
from nav_msgs.msg import Odometry # Added

def curr_pos_callback(msg):
    global current_position
    current_position = msg.pose.pose.position

# PVA control is more accurate than just basics position control
# since it considers time constraints between each waypoint and does
# not just rely on the control loop delay implemented in PX4
if __name__ == "__main__":

    rospy.init_node('Quad1_offboard_pvacontrol')   # CHANGED !!!! #

    # Should be the prefix added to the offboard control node (prefix of all mavros topics)
    # for example "" or "/Quad9" or "/Quad10"
    quad_ros_namespace = "/uav1" # CHANGED !!!! #

    # Create a publisher to send target positions
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    
    # Start position control with taking off at the beginning
    start_pva_control(quad_name= quad_ros_namespace , takeoff_before= False)

    # It's assume the vehicle is initially at 0,0,0 position but the real position
    # can be obtained by subscribing to the appropriate mavros topic
    current_position = Point()
    rospy.Subscriber("/uav1/mavros/local_position/odom",Odometry , curr_pos_callback)   # CHANGED !!!! #
    current_pos = current_position
    # current_pos = Point(0,0,0)

    target1 = Point(2 , 2 , 4)
    target2 = Point(3 , 2 , 3)
    target3 = Point(20 , 3 , 4)
    target4 = Point(3 , 2 , 7)
    target5 = Point(0 , 0 ,0)

    time_full_traj = 50 #seconds
    freq = 50

    wait_rate = rospy.Rate(freq)

    x_traj = [current_pos.x , target1.x , target2.x , target3.x , target4.x , target5.x]
    y_traj = [current_pos.y , target1.y , target2.y , target3.y , target4.y , target5.y]
    z_traj = [current_pos.z , target1.z , target2.z , target3.z , target4.z , target5.z]

    pva_list = generate_traj_3d(x=x_traj , y=y_traj , z=z_traj , traj_time=[0,time_full_traj] , corr=None , freq = freq)
    
    # Send the generate traj to the vehicle
    for elem in pva_list.pva :
        send_pva_pub.publish(elem)
        wait_rate.sleep()
    

    # Land the vehicule
    start_landing(quad_name= quad_ros_namespace)

    
    # ####### Initialize grid specifications #################
    # GRID_SIZE     = 1
    # Z_ALTITUDE    = 2.0
    # X_NUMBER_TILE = 10   # X correspond to the larger size -> width
    # Y_NUMBER_TILE = 10   # Y height correspond to the smaller -> height

    # # the point at the left down corner in your coordinate system
    # base = Point()
    # base.x = -5
    # base.y = -5
    # base.z = Z_ALTITUDE

    # #
    # maxi = Point()
    # maxi.x = GRID_SIZE * X_NUMBER_TILE + base.x
    # maxi.y = GRID_SIZE * Y_NUMBER_TILE + base.y
    # maxi.z = Z_LEVEL

    # # Grid creation
    # grid = Grid(X_NUMBER_TILE ,Y_NUMBER_TILE , base , maxi)

    # print grid.x , grid.y , grid.base , grid.maximum , grid.blocklengthX , grid.blocklengthY

    # ####### End grid specifications #############################
