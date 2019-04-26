#!/usr/bin/env python

import sys
# sys.path.insert( 1 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
sys.path.append('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
import rospy
from qcontrol_defs.msg import *
from utils_functions import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import simplejson as json
import pickle
from Grid_State_Conversions import *
import os
from subprocess import Popen, PIPE

# def get_quad_pos_from_state(converter, state, xOffset, yOffset, zOffset, z_nom):
#     point_from_converter = converter.state2vicon(state)
#     wp = Point(point_from_converter.x, point_from_converter.y, z_nom)
#     x_traj = [quad_current_pos.x, wp.x - xOffset]
#     y_traj = [quad_current_pos.y, wp.y - yOffset]
#     z_traj = [quad_current_pos.z, wp.z - zOffset]
#     return x_traj, y_traj, z_traj

# Instantiate a converter
base = Point()
base.x = 5
base.y = 5
maximum = Point()
maximum.x = 46
maximum.y = 46
converter = Grid(41,41,base,maximum)

point_from_converter = converter.state2vicon(1140)
print point_from_converter

# # Determine next cartesian position of the new quad state
# x_traj, y_traj, z_traj = get_quad_pos_from_state(converter, 169, xOffset, yOffset, zOffset, z_nom)
# print x_traj
# print y_traj
# print z_traj
sys.exit()


    # <param name="base_x" value="-15" />
    #     <param name="base_y" value="-19" />
    #     <param name="max_x" value="15" />
    #     <param name="max_y" value="15" />
    #     <param name="nb_y" value="34" />
    #     <param name="nb_x" value="31" />