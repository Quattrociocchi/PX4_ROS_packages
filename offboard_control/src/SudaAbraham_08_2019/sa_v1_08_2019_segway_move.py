#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
# from qcontrol_defs.msg import *
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')

def status_callback(msg):
    global segway_status
    segway_status = msg

def trajComplete_callback_1(msg):
    global traj_status_1
    traj_status_1 = msg.data

def trajComplete_callback_2(msg):
    global traj_status_2
    traj_status_2 = msg.data

def convert_points(dict_of_points_in_gazebo_frame):
    # Hard code the transform from "Gazebo" frame to /move_base_simple/goal frame
    xOffset = 0.000
    yOffset = 0.000
    zOffset = 0.000
    QxOffset = 0.000
    QyOffset = 0.000
    QzOffset = -0.004
    wOffset = 1.000
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

    # Get the params needed for the monitoring the quads status
    quad_ros_namespace_1 = rospy.get_param('/Quad1_action/quad_ros_namespace')
    quad_ros_namespace_2 = rospy.get_param('/Quad2_action/quad_ros_namespace')
    # Set up subscriber for completed trajectory flag
    rospy.Subscriber(quad_ros_namespace_1 + '/trajComplete', Bool, trajComplete_callback_1)
    rospy.Subscriber(quad_ros_namespace_2 + '/trajComplete', Bool, trajComplete_callback_2)
    # Set up publishers to start the sim
    send_trajComplete_pub_1 = rospy.Publisher(quad_ros_namespace_1 + '/trajComplete', Bool, queue_size=10)
    send_trajComplete_pub_2 = rospy.Publisher(quad_ros_namespace_2 + '/trajComplete', Bool, queue_size=10)



    # Define global that will track the status of the quad 1
    global traj_status_1
    traj_status_1 = False
    # Define global that will track the status of the quad 2
    global traj_status_2
    traj_status_2 = False

    # Prompt user to ask if the initial pose needs to be set and check to make sure a number was entered
    try:
        ipua_int = int(input("Does the initial pose need to be set? 1=YES  2=NO:\n"))
    except:
        print("You have not entered a number, so this program assumes you want to exit. Goodbye")
        sys.exit()
    # Check to make sure a valid number was entered
    if not (ipua_int == 1 or ipua_int == 2):
        print("You have not entered a '1' or '2', so this program assumes you want to exit. Goodbye")
        sys.exit()

    if ipua_int == 2:
        True
    else:
        ''' Publish the initial position of the segway. Perhaps turn this into a service, because you must
            wait for the segway to stop moving before you can start publishing waypoints. ''' 
        segway_init_pos_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.sleep(0.5)
        init_position = PoseWithCovarianceStamped()
        init_position.header.frame_id = 'map'
        # Convert to map frame from gazebo frame
        init_pos_gazebo = {'x':[12.7],'y':[15.2],'z':[0]}
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
    # Setup waypoint message variable
    waypoint = PoseStamped()
    waypoint.header.frame_id = 'map'
    waypoint.pose.orientation.w = 0.10033986
    waypoint.pose.orientation.z = -0.99495322
    # Set up the segway status subscriber
    segway_status_sub = rospy.Subscriber('segway_move_base/status', GoalStatusArray, status_callback)
    # Define global so it exists for the if statement below before the callback overwrites it
    global segway_status
    segway_status = GoalStatusArray()

    # Define the list of waypoints
    wps = {1:[11.9,41.3,0.985,0.172], 2:[24.0,39.7,0.597,0.802], 3:[37.4,40.2,-0.451,0.892], 4:[13.3,29.4,0.961,0.275], 5:[25.8,28.8,0.993,0.117], 6:[40.3,25.9,0.920,39.1], 7:[12.7,15.2,-0.863,0.504], 8:[25.5,13.2,0.840,0.541], 9:[37.1,13.0,0.223,0.974]}

    # Define the trajectory
    traj = [wps[1],wps[2],wps[3]]
    num_wp = len(traj)
    wp_tracker = 0

    raw_input("\nPress any key to start the simulation\n")
    safety_check = True
    if safety_check:
        send_trajComplete_pub_1.publish(True)
        send_trajComplete_pub_2.publish(True)


    while not rospy.is_shutdown():
        try:
            segway_status_value = segway_status.status_list[-1].status
        except:
            segway_status_value = 3

        if not traj_status_1 or not traj_status_2 or segway_status_value ==1:
            continue
        elif traj_status_1 and traj_status_2 and segway_status_value ==3:
            segway_gaz_dict = {'x':[traj[wp_tracker][0]],'y':[traj[wp_tracker][1]],'z':[0]}
            segway_waypoint = convert_points(segway_gaz_dict)
            waypoint.pose.position.x = segway_waypoint['x'][0]
            waypoint.pose.position.y = segway_waypoint['y'][0]
            waypoint.pose.orientation.z = traj[wp_tracker][2]
            waypoint.pose.orientation.w = traj[wp_tracker][3]

            print 'Moving to the next Waypoint'
            segway_pub.publish(waypoint)
            rospy.sleep(1.)
            
            wp_tracker += 1
            if wp_tracker == num_wp:
                print("\nThe trajectory has been traversed. Goodbye\n")
                sys.exit()

## This was used before making synchronous and works ## Delete once finished
        # if segway_status_value == 3:
        #     segway_gaz_dict = {'x':[traj[wp_tracker][0]],'y':[traj[wp_tracker][1]],'z':[0]}
        #     segway_waypoint = convert_points(segway_gaz_dict)
        #     waypoint.pose.position.x = segway_waypoint['x'][0]
        #     waypoint.pose.position.y = segway_waypoint['y'][0]
        #     waypoint.pose.orientation.z = traj[wp_tracker][2]
        #     waypoint.pose.orientation.w = traj[wp_tracker][3]

        #     print 'Moving to the next Waypoint'
        #     segway_pub.publish(waypoint)
        #     rospy.sleep(1.)
            
        #     wp_tracker += 1
        #     if wp_tracker == num_wp:
        #         print("\nThe trajectory has been traversed. Goodbye\n")
        #         sys.exit()
