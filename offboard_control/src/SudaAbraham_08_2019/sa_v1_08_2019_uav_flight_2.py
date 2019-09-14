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
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatusArray
# ------------------------------------------------------------------ #
def quad_pos_callback(msg):
    global quad_current_pos
    quad_current_pos = msg.pose.position

def status_callback(msg):
    global segway_status
    segway_status = msg

def trajComplete_callback(msg):
    global traj_status
    traj_status = msg.data
# ------------------------------------------------------------------ #

if __name__ == "__main__":
    # Initialize node.
    rospy.init_node('quad_action', anonymous=True)
    # Get params from server
    quad_ros_namespace = rospy.get_param('~quad_ros_namespace')
    xOffset = rospy.get_param('~xOffset')
    yOffset = rospy.get_param('~yOffset')
    zOffset = rospy.get_param('~zOffset')
    freq = rospy.get_param('~freq')
    time_full_traj = rospy.get_param('~time_full_traj')
    wait_rate = rospy.Rate(freq)
    z_nom = rospy.get_param('~z_nom')

    # Define global so it exists for the if statement below before the callback overwrites it
    global segway_status
    segway_status = GoalStatusArray()
    trajCompleted = Bool()
    # Define global that will track the status of the other quad
    global traj_status
    traj_status = False

    # This is a hack to figure out which namespace I need to ask for
    this_ns_num = [int(s) for s in quad_ros_namespace.split('v') if s.isdigit()][0]
    if this_ns_num == 1:
        other_ns_num = 2
    else:
        other_ns_num = 1
    # Set the param needed for the monitoring the quads status
    quad_ros_namespace_other = '/uav' + str(other_ns_num)

    # Create publishers
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    send_trajComplete_pub = rospy.Publisher(quad_ros_namespace + '/trajComplete', Bool, queue_size=10)
    # Create subscribers
    rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, quad_pos_callback)
    rospy.Subscriber(quad_ros_namespace_other + '/trajComplete', Bool, trajComplete_callback)
    segway_status_sub = rospy.Subscriber('segway_move_base/status', GoalStatusArray, status_callback)
    rospy.sleep(1.0)

    # waypoints = {'wp0':(11.9,41.3), 'wp1':(24.0,39.7), 'wp2':(37.4,40.2), 'wp3':(13.3,29.4), 'wp4':(25.8,28.8), 'wp5':(40.3,25.9), 'wp6':(12.7,15.2), 'wp7':(25.5,13.2), 'wp8':(37.1,13.0)}
    
    path_trajs = {'path1':[[],[],[]], 'path2':[[],[],[]], 'path3':[[],[],[]], 'path4':[[],[],[]], 'path5':[[],[],[]], 'path6':[[],[],[]]}
    path1_x = [35,30,25.5,24,22,19,15,12.7]
    path1_x = np.array(path1_x) - xOffset
    path1_y = [42,42.5,41.5,37,21,19,19,15.2]
    path1_y = np.array(path1_x) - xOffset
    path1_z = list(np.repeat([z_nom],len(path1_x)))
    path_trajs['path1'][0] = path1_x
    path_trajs['path1'][1] = path1_y
    path_trajs['path1'][2] = path1_z

    path2_x = [15,19,22,24,25.5,30,35,37.4]
    path2_x = np.array(path2_x) - xOffset
    path2_y = [19,19,21,37,41.5,42.5,42,40.2]
    path2_y = np.array(path2_x) - xOffset
    path2_z = list(np.repeat([z_nom],len(path2_x)))
    path_trajs['path2'][0] = path2_x
    path_trajs['path2'][1] = path2_y
    path_trajs['path2'][2] = path2_z

    path3_x = [15,19,29,33,37.1]
    path3_x = np.array(path3_x) - xOffset
    path3_y = [19,19,12,12,13]
    path3_y = np.array(path3_x) - xOffset
    path3_z = list(np.repeat([z_nom],len(path3_x)))
    path_trajs['path3'][0] = path3_x
    path_trajs['path3'][1] = path3_y
    path_trajs['path3'][2] = path3_z

    path4_x = [33,29,19,15,12.7]
    path4_x = np.array(path4_x) - xOffset
    path4_y = [12,12,19,19,15.2]
    path4_y = np.array(path4_x) - xOffset
    path4_z = list(np.repeat([z_nom],len(path4_x)))
    path_trajs['path4'][0] = path4_x
    path_trajs['path4'][1] = path4_y
    path_trajs['path4'][2] = path4_z

    path5_x = [33,29,22,24,25.5,30,35,37.4]
    path5_x = np.array(path5_x) - xOffset
    path5_y = [12,12,21,37,41.5,42.5,42,40.2]
    path5_y = np.array(path5_x) - xOffset
    path5_z = list(np.repeat([z_nom],len(path5_x)))
    path_trajs['path5'][0] = path5_x
    path_trajs['path5'][1] = path5_y
    path_trajs['path5'][2] = path5_z

    path6_x = [35,30,25.5,24,22,29,33,37.1]
    path6_x = np.array(path6_x) - xOffset
    path6_y = [42,42.5,41.5,37,21,12,12,13]
    path6_y = np.array(path6_x) - xOffset
    path6_z = list(np.repeat([z_nom],len(path6_x)))
    path_trajs['path6'][0] = path6_x
    path_trajs['path6'][1] = path6_y
    path_trajs['path6'][2] = path6_z

    path_to_fly = ['path3','path4','path2']
    
    # raw_input('press a key to start the sim')
    
    # Start by taking off
    rospy.sleep(0.5)
    start_pva_control(quad_name= quad_ros_namespace , takeoff_before= True)
    rospy.sleep(0.5)


    while not rospy.is_shutdown():
        try:
            segway_status_value = segway_status.status_list[-1].status
        except:
            segway_status_value = 3

        if not traj_status or segway_status_value ==1:
            continue
        elif traj_status and segway_status_value ==3:
            for elem in path_to_fly:
                # Notify that trajectory is incomplete
                trajCompleted = False
                send_trajComplete_pub.publish(trajCompleted)

                current_pos = quad_current_pos
                x_traj = [current_pos.x].append(path_trajs[elem][0])
                y_traj = [current_pos.y].append(path_trajs[elem][1])
                z_traj = [current_pos.z].append(path_trajs[elem][2])
                pva_list = generate_traj_3d(x=x_traj , y=y_traj , z=z_traj , traj_time=[0,time_full_traj] , corr=None , freq = freq)

                # Send the generated traj to the vehicle
                for elem in pva_list.pva:
                    send_pva_pub.publish(elem)
                    wait_rate.sleep()

                # Notify that the trajectory is complete
                trajCompleted = True
                send_trajComplete_pub.publish(trajCompleted)


    # Land the vehicle
    start_landing(quad_name= quad_ros_namespace)
