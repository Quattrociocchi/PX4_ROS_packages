#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
from qcontrol_defs.msg import *
from utils_functions import *
from std_msgs.msg import Bool

def status_callback(msg):
    global segway_status
    segway_status = msg

def trajComplete_callback(msg):
    global traj_status
    traj_status = msg.data

def convert_points(dict_of_points_in_gazebo_frame):
    # Hard code the transform from "Gazebo" frame to /move_base_simple/goal frame
    xOffset = -6.944
    yOffset = 6.161
    zOffset = 0.001
    QxOffset = 0.000
    QyOffset = 0.000
    QzOffset = -0.001
    wOffset = 0.000
    dict_of_points_in_map_frame = {k: [] for k in dict_of_points_in_gazebo_frame.keys()}

    for key in dict_of_points_in_gazebo_frame.keys():
        if key == 'x':
            for wp in dict_of_points_in_gazebo_frame[key]:
                dict_of_points_in_map_frame[key].append(wp - xOffset)
        if key == 'y':
            for wp in dict_of_points_in_gazebo_frame[key]:
                dict_of_points_in_map_frame[key].append(wp - yOffset)
        if key == 'z':
            for wp in dict_of_points_in_gazebo_frame[key]:
                dict_of_points_in_map_frame[key].append(wp - zOffset)
        if key == 'w':
            for wp in dict_of_points_in_gazebo_frame[key]:
                dict_of_points_in_map_frame[key].append(wp - wOffset)
        if key == 'Qx':
            for wp in dict_of_points_in_gazebo_frame[key]:
                dict_of_points_in_map_frame[key].append(wp - QxOffset)
        if key == 'Qy':
            for wp in dict_of_points_in_gazebo_frame[key]:
                dict_of_points_in_map_frame[key].append(wp - QyOffset)
        if key == 'Qz':
            for wp in dict_of_points_in_gazebo_frame[key]:
                dict_of_points_in_map_frame[key].append(wp - QzOffset)
    return dict_of_points_in_map_frame

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('segway_commands_user')
    
    """ Publish the initial position of the segway. Perhaps turn this into a service, because you must
        wait for the segway to stop moving before you can start publishing waypoints. """ 
    segway_init_pos_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(0.5)
    init_position = PoseWithCovarianceStamped()
    init_position.header.frame_id = 'map'
    # Convert to map frame from gazebo frame
    init_pos_gazebo = {'x':[8],'y':[-1],'z':[0]}
    init_pose_map = convert_points(init_pos_gazebo)
    # Set the initial pose in map frame
    init_position.pose.pose.position.x = init_pose_map['x'][0]
    init_position.pose.pose.position.y = init_pose_map['y'][0]
    init_position.pose.pose.position.z = init_pose_map['z'][0]
    init_position.pose.pose.orientation.w = 1
    # Publish the inital pos
    segway_init_pos_pub.publish(init_position)
    rospy.sleep(5.)
    print 'Initial position set'
    rospy.sleep(2.) # Do this to wait for segway move base server to start. REPLACE WITH SERVER CHECK

    # Set up the waypoints publisher
    segway_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(0.5)
    # Set up waypoint dictionary
    waypoints_gazebo = {'x':[6,6,6,6,4,-3,-9,-9],'y':[-1,-1,-1,-1,-2,-2,-3,-3],'w':[0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05],'Qz':[-1,-1,-1,-1,-1,-1,-1,-1]} # Quaternion values are trial and error.....
    waypoints_map = convert_points(waypoints_gazebo)
    # Setup waypoint message variable
    waypoint = PoseStamped()
    waypoint.header.frame_id = 'map'
    waypoint.pose.position.x = waypoints_map['x'][0]
    waypoint.pose.position.y = waypoints_map['y'][0]
    waypoint.pose.orientation.w = waypoints_map['w'][0]
    waypoint.pose.orientation.z = waypoints_map['Qz'][0]

    # Set up the segway status subscriber
    segway_status_sub = rospy.Subscriber('segway_move_base/status', GoalStatusArray, status_callback)
    # Define global so it exists for the if statement below before the callback overwrites it
    global segway_status
    segway_status = GoalStatusArray()

    # Prompt user to start waypoint following
    trash = input("Enter any number to begin: ")

    # Set namespace for quad (target)
    quad_ros_namespace1 = "/uav1"
    # Set up subscriber for completed trajectory flag
    rospy.Subscriber(quad_ros_namespace1 + '/trajComplete', Bool, trajComplete_callback)
    # Define global that will track the status of the quad (target)
    global traj_status
    traj_status = False

    ii = 0
    while not rospy.is_shutdown():
        if ii == len(waypoints_map['x']):
            print 'The segway has arrived at the final waypoint.'
            break

        try:
            segway_status_value = segway_status.status_list[-1].status
        except:
            segway_status_value = 3

        if not traj_status or segway_status_value == 1:
            continue
        elif traj_status and segway_status_value == 3 and len(waypoints_map['x']) > ii:
            print 'You have arrived'
            rospy.sleep(0.5)
            # Update the waypoint
            waypoint.pose.position.x = waypoints_map['x'][ii]
            waypoint.pose.position.y = waypoints_map['y'][ii]
            waypoint.pose.orientation.w = waypoints_map['w'][ii]
            ii += 1
            print 'Moving to the next Waypoint'
            segway_pub.publish(waypoint)
            rospy.sleep(1.)
            traj_status = False
        else:
            print 'Something went wrong'
            print segway_status.status_list[-1].status

    # while not rospy.is_shutdown():
    #     if (not segway_status.status_list and not traj_status) or segway_status.status_list[-1].status == 1:
    #         continue
    #     elif segway_status.status_list[-1].status == 3 and len(waypoints_map['x']) > ii:
    #         print 'You have arrived'
    #         rospy.sleep(0.5)
    #         # Update the waypoint
    #         waypoint.pose.position.x = waypoints_map['x'][ii]
    #         waypoint.pose.position.y = waypoints_map['y'][ii]
    #         waypoint.pose.orientation.w = waypoints_map['w'][ii]
    #         ii += 1
    #         print 'Moving to the next Waypoint'
    #         segway_pub.publish(waypoint)
    #         rospy.sleep(1.)
    #         # trajComplete1 = False
    #     elif ii == len(waypoints_map['x']):
    #         print 'The segway has arrived at the final waypoint.'
    #         break
    #     else:
    #         # print 'Something went wrong'
    #         print segway_status.status_list[-1].status































