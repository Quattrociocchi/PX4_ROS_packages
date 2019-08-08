#!/usr/bin/env python

''' UAV Flight Controller for multi-agent sim for Nasa presentation. May 2019 '''

import sys
sys.path.append('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
import rospy
from qcontrol_defs.msg import *
from utils_functions import *
from geometry_msgs.msg import PoseStamped
import os
import numpy as np
# ------------------------------------------------------------------ #
def quad_pos_callback(msg):
    global quad_current_pos
    quad_current_pos = msg.pose.position
# ------------------------------------------------------------------ #

if __name__ == "__main__":
    # Initialize node.
    rospy.init_node('quad_action', anonymous=True)
    # Get params from server
    quad_ros_namespace = rospy.get_param('~quad_ros_namespace')
    xOffset = rospy.get_param('~xOffset')
    yOffset = rospy.get_param('~yOffset')
    zOffset = rospy.get_param('~zOffset')
    scaling_factor = rospy.get_param('~scaling_factor')
    freq = rospy.get_param('~freq')
    time_full_traj = rospy.get_param('~time_full_traj')
    wait_rate = rospy.Rate(freq)
    z_nom = rospy.get_param('~z_nom')
    path_to_fly = rospy.get_param('~path_to_fly')
##########################################################################################################
#--- The final two points are not actually towers, they are near the tower at (2,4) in order to allow for multiple vehicles to wait without collision
    tower_positions = {'wp0':(0.,0.), 'wp1':(1.,1.), 'wp2':(1.,2.8), 'wp3':(2.,2.), 'wp4':(1.3,-0.2), 'wp5':(4.,3.), 'wp6':(2.8,3.4), 'wp7':(3.1,1.8), 'wp8':(2.,4.), 'wp9':(1.9,4.3), 'wp10':(2.1,4.2)}

    # Create a publisher to send target positions
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    # Create subscriber for current position of quad from mavros (sensor)
    rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, quad_pos_callback)
    rospy.sleep(1.0)
    current_pos = quad_current_pos

    # Pull waypoints from list and define them in order
    x_traj = [current_pos.x]
    y_traj = [current_pos.y]
    z_traj = [current_pos.z] 
    targets = []

    for elem in path_to_fly:
        tt = np.array(tower_positions[elem])*scaling_factor
        # Store incase we need to view it
        targets.append(Point(tt[0], tt[1], z_nom))
        # Build trajectory list. Should be 1 element longer than the given waypoints (includes starting position)
        x_traj.append(targets[-1].x - xOffset*scaling_factor)
        y_traj.append(targets[-1].y - yOffset*scaling_factor)
        z_traj.append(targets[-1].z - zOffset*scaling_factor)

    # Find the complete minimum snap trajectories
    pva_list = []
    for ii in range(0, len(x_traj) - 1):
        x_traj_temp = [x_traj[ii], x_traj[ii + 1]]
        y_traj_temp = [y_traj[ii], y_traj[ii + 1]]
        z_traj_temp = [z_traj[ii], z_traj[ii + 1]]
        pva_list_temp = generate_traj_3d(x=x_traj_temp , y=y_traj_temp , z=z_traj_temp , traj_time=[0,time_full_traj] , corr=None , freq = freq)
        pva_list.append(pva_list_temp)




##########################################################################################################
    # Start by taking off
    rospy.sleep(0.5)
    start_pva_control(quad_name= quad_ros_namespace , takeoff_before= True)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        # Send the generated traj to the vehicle
        for lst in pva_list:
            for elem in lst.pva:
                send_pva_pub.publish(elem)
                wait_rate.sleep()

        break

    # Land the vehicle
    start_landing(quad_name= quad_ros_namespace)
