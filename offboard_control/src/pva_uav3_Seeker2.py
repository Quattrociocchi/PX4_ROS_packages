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

    # Pull in the trajectory
    traj = []
    import csv
    with open('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/statehistory2.txt','rb') as csvfile:
        totalstate = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in totalstate:
             traj.append(map(int,row))

    # Initialize node.
    rospy.init_node('Quad3_offboard_pvacontrol')

    # Should be the prefix added to the offboard control node (prefix of all mavros topics)
    # for example "" or "/Quad9" or "/Quad10"
    quad_ros_namespace = "/uav3"

    # Create a publisher to send target positions
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    
    # Start position control with taking off at the beginning
    start_pva_control(quad_name= quad_ros_namespace , takeoff_before= True)

    # Initial position is set with mavros
    current_position = Point()
    rospy.Subscriber("/uav3/mavros/local_position/odom",Odometry , curr_pos_callback)
    current_pos = current_position
    # current_pos = Point(0,0,0)

    # Set time stuff
    time_full_traj = 200 #seconds
    freq = 200
    wait_rate = rospy.Rate(freq)

    # For loop that sets the trajectory that is sent out to gazebo
    x_traj = [current_pos.x]
    y_traj = [current_pos.y]
    z_traj = [current_pos.z]

    for s in traj:
        temp = Point(s[0] , s[1] , 2)
        x_traj.append(temp.x)
        y_traj.append(temp.y)
        z_traj.append(temp.z)
        # x_traj.append(s[0])
        # y_traj.append(s[1])
        # z_traj.append(2)


    # Make the list that is sent out using this nifty function(?) thing...
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
