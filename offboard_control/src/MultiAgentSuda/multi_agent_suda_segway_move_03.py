#!/usr/bin/env python

import rospy
# import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from Grid_State_Conversions import *
# from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
from qcontrol_defs.msg import *
# from utils_functions import *
from std_msgs.msg import Bool
import simplejson as json
import pickle

# def status_callback(msg):
#     global segway_status
#     segway_status = msg

# def trajComplete_callback(msg):
#     global traj_status
#     traj_status = msg.data

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

# def quad_pos_callback(msg):
#     global quad_pos_msg
#     quad_pos_msg = msg.pose

# def parseJson(filename):
#     automaton = dict()
#     file = open(filename)
#     data = json.load(file)
#     file.close()
#     variables = dict()
#     for var in data['variables']:
#         v = var.split('@')[0]
#         if v not in variables.keys():
#             for var2ind in range(data['variables'].index(var), len(data['variables'])):
#                 var2 = data['variables'][var2ind]
#                 if v != var2.split('@')[0]:
#                     variables[v] = [data['variables'].index(var), data['variables'].index(var2)]
#                     break
#                 if data['variables'].index(var2) == len(data['variables']) - 1:
#                     variables[v] = [data['variables'].index(var), data['variables'].index(var2)+1]

#     for s in data['nodes'].keys():
#         automaton[int(s)] = dict.fromkeys(['State','Successors','Predecessors'])
#         automaton[int(s)]['State'] = dict()
#         automaton[int(s)]['Successors'] = []
#         automaton[int(s)]['Predecessors'] = []
#         for v in variables.keys():
#             bin = data['nodes'][s]['state'][variables[v][0]:variables[v][1]]
#             automaton[int(s)]['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
#             automaton[int(s)]['Successors'] = data['nodes'][s]['trans']
#     for s in data['nodes'].keys():
#         for t in automaton[int(s)]['Successors']:
#             automaton[t]['Predecessors'].append(int(s))
#     return automaton

# def next_game_and_agent_state(automaton, game_state, target_state, iset):
#     agent_state = automaton[game_state]['State']['s']
#     is_vis = not invis_check(agent_state, target_state, iset) # This should be true if the agent state is visible from the target state

#     new_game_state = None
#     if is_vis:
#         for k in automaton[game_state]['Successors']:
#             if automaton[k]['State']['st'] == target_state:
#                 new_game_state = k
#                 break
#     else:
#         for k in automaton[game_state]['Successors']:
#             if automaton[k]['State']['st'] > 1053:
#                 new_game_state = k
#                 break
#     assert new_game_state != None, 'No successor found!'

#     new_agent_state = automaton[new_game_state]['State']['s']
#     # print 'Printing game state'
#     # print gs
#     return new_game_state, new_agent_state

# def invis_check(agent_state, target_state, iset):
#     return (target_state in iset[agent_state])


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('segway_commands_user')

    ''' Publish the initial position of the segway. Perhaps turn this into a service, because you must
        wait for the segway to stop moving before you can start publishing waypoints. ''' 
    segway_init_pos_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(0.5)
    init_position = PoseWithCovarianceStamped()
    init_position.header.frame_id = 'map'
    # Convert to map frame from gazebo frame
    init_pos_gazebo = {'x':[-3.87097],'y':[4.5],'z':[0]}
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

    sys.exit()

    # # Set up the waypoints publisher
    # segway_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    # rospy.sleep(0.5)
    # # Setup waypoint message variable
    # waypoint = PoseStamped()
    # waypoint.header.frame_id = 'map'
    # waypoint.pose.orientation.w = 0.10033986
    # waypoint.pose.orientation.z = -0.99495322
    # # Set up the segway status subscriber
    # rospy.Subscriber('segway_move_base/status', GoalStatusArray, status_callback)
    # # segway_status_sub = rospy.Subscriber('segway_move_base/status', GoalStatusArray, status_callback) <--- FOR REFERENCE. DELETE WHEN FINISHED
    # # Set up subscriber for completed trajectory flag
    # rospy.Subscriber(quad_ros_namespace + '/trajComplete', Bool, trajComplete_callback)
    # # Define global so it exists for the if statement below before the callback overwrites it
    # global segway_status
    # segway_status = GoalStatusArray()
    # # Define global that will track the status of the quad (target)
    # global traj_status
    # traj_status = False

    # # THIS SHOULD BE REPLACED WITH A USER INPUT OPTION
    # filename = 'Example3_Gazebo.json'
    # # Generate the automaton dictionary
    # automaton = parseJson(filename)
    # game_state = 0

    # # load up the invisible state dictionary
    # pickle_in = open("gazeboExample_invis_states.pickle","rb")
    # iset = pickle.load(pickle_in)

    # # This is A DEBUG CHECK.
    # quad_gaz_point = Point()
    # quad_gaz_point.x = quad_pos_msg.position.x + quad_xOffset
    # quad_gaz_point.y = quad_pos_msg.position.y + quad_yOffset
    # quad_state = converter.vicon2state(quad_gaz_point)
    # print 'The quads state initial state is:'
    # print quad_state


    # # Prompt user to start the script
    # trash = input("Enter any number to begin: ")

    # while not rospy.is_shutdown():
    #     try:
    #         segway_status_value = segway_status.status_list[-1].status
    #     except:
    #         segway_status_value = 3

    #     if not traj_status or segway_status_value == 1:
    #         continue
    #     elif traj_status and segway_status_value == 3:
    #         print 'Determining location of the Target'
    #         # This is the work of checking the quads state.
    #         quad_gaz_point = Point()
    #         quad_gaz_point.x = quad_pos_msg.position.x + quad_xOffset
    #         quad_gaz_point.y = quad_pos_msg.position.y + quad_yOffset
    #         quad_state = converter.vicon2state(quad_gaz_point)
    #         print 'Printing the quads current state'
    #         print quad_state

    #         game_state, segway_state = next_game_and_agent_state(automaton, game_state, quad_state, iset)
    #         print 'Printing the segways state is:'
    #         print segway_state
    #         print 'The game state is:'
    #         print game_state

    #         segway_point_from_converter = converter.state2vicon(segway_state)
    #         segway_gaz_dict = {'x':[segway_point_from_converter.x],'y':[segway_point_from_converter.y],'z':[0]}
    #         segway_waypoint = convert_points(segway_gaz_dict)
    #         waypoint.pose.position.x = segway_waypoint['x'][0]
    #         waypoint.pose.position.y = segway_waypoint['y'][0]

    #         print 'Moving to the next Waypoint'
    #         segway_pub.publish(waypoint)
    #         rospy.sleep(1.)
    #         traj_status = False
    #     else:
    #         print 'Something went wrong'
    #         print segway_status.status_list[-1].status
