#!/usr/bin/env python


import rospy
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
import time



def trajComplete_callback(msg):
    global trajComplete
    trajComplete = True

















if __name__ == "__main__":
    # Initialize Node.
    rospy.init_node('Quad_offboard_pvacontrol')
    # Set namespaces
    quad_ros_namespace1 = "/uav1"
    quad_ros_namespace2 = "/uav2"
    # Start by taking off
    start_pva_control(quad_name= quad_ros_namespace1 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace2 , takeoff_before= True)
    # Set up action publishers
    action_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/shield_bool_orders', ShieldBool , queue_size=10)
    action_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/shield_bool_orders', ShieldBool , queue_size=10)
    # Create action messages
    action_msg_1 = ShieldBool()
    action_msg_1.shield_bool = [True]
    action_msg_2 = ShieldBool()
    action_msg_2.shield_bool = [True]

    
    rospy.sleep(0.5)
    pub.publish(msg)
    
    rospy.Subscriber(quad_ros_namespace1 + '/trajComplete', ShieldBool, trajComplete_callback)

    while not rospy.is_shutdown():
        if 'trajComplete' in globals():
            pub.publish(msg)
            del globals()['trajComplete']
        else:
            pass




action_list = []
action_list.append('/action_00')
action_list.append('/action_01')
action_list.append('/action_02')
action_list.append('/action_03')
action_list.append('/action_04')
action_list.append('/action_05')
action_list.append('/action_06')
action_list.append('/action_07')
action_list.append('/action_08')
action_list.append('/action_09')
action_list.append('/action_10')
action_list.append('/action_11')
action_list.append('/action_12')
action_list.append('/action_13')














#     # Get the automaton from the json file
#     directory = '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/jsonFiles'

#     jsonInputFile = directory + '/example2.json'
#     A = parseJson(jsonInputFile)
    
#     # readableOutputFile = directory + '/sheildFirstOutput'
#     # writeJson(jsonInputFile,readableOutputFile)
#     # sys.exit()
#     ##############################

#     # Initialize node.
#     rospy.init_node('Quad_offboard_pvacontrol')

#     # Should be the prefix added to the offboard control node (prefix of all mavros topics)
#     # for example "" or "/Quad9" or "/Quad10" (or "/uav1" for px4)
#     quad_ros_namespace1 = "/uav1"
#     quad_ros_namespace2 = "/uav2"

#     # Create a publisher to send target positions
#     send_pva_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/qcontrol/pva_control', PVA , queue_size=10)
#     send_pva_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/qcontrol/pva_control', PVA , queue_size=10)

#     # Create a publisher to send shield boolean message
#     send_shield_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/shield_bool_list',ShieldInput, queue_size=10)
#     send_shield_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/shield_bool_list',ShieldInput, queue_size=10)

#     # # Start position control with taking off at the beginning
#     start_pva_control(quad_name= quad_ros_namespace1 , takeoff_before= True)
#     start_pva_control(quad_name= quad_ros_namespace2 , takeoff_before= True)

#     ##############################

#     # Set time parameters
#     time_traj = 1.25   # Seconds
#     freq = 10
#     wait_rate = rospy.Rate(freq)

#     ##############################

#     # Set the direction that the quad will try to fly
#     uLoc1_traj = [0, 0, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 0, 1, 0]   # North-South
#     uLoc0_traj = [0, 0, 4, 4, 4, 3, 4, 4, 3, 3, 3, 3, 3, 3, 0]   # East-West
#     ncols = 5   # Determined by grid size

#     # Define traj's for later
#     traj_s1 = []
#     traj_s0 = []

#     ##############################

#     # Set offsets and initial positions (local frame)
#     xOffset_s1 = 2
#     yOffset_s1 = 0
#     xOffset_s0 = 0
#     yOffset_s0 = 2

#     initial_pos = Point(0,0,2)

#     # Set initial positions
#     x_traj_s1 = [initial_pos.x]
#     y_traj_s1 = [initial_pos.y]
#     z_traj_s1 = [initial_pos.z]

#     x_traj_s0 = [initial_pos.x]
#     y_traj_s0 = [initial_pos.y]
#     z_traj_s0 = [initial_pos.z]

#     ##############################

#     ns = 0 # Value of the next state
#     counter = 0
#     for uLoc1, uLoc0 in zip(uLoc1_traj,uLoc0_traj):

#         # Check to see if the quads will violate the safety specification
#         for k in A[ns]['Successors']:
#             if A[k]['State']['uloc0'] == uLoc0 and A[k]['State']['uloc1'] == uLoc1:
#                 ns = k
#                 break

#         # These are cardinal directions
#         uShield1 = A[ns]['State']['ushield1'] # Movements North or South (y)
#         uShield0 = A[ns]['State']['ushield0'] # Movements East or West (x)

# # ---------------------------------- Start new code for implementing the position change with seperate node ----------------------------------- #
#         # Has the order {+x, -x, +y, -y, +z, -z}
#         bool_list_1 = card2shield_input_msg(uShield1)
#         bool_list_2 = card2shield_input_msg(uShield0)

#         ############### Code for figuring out the dx and dy ####################
#         send_shield_pub_1.publish(bool_list_1)
#         send_shield_pub_2.publish(bool_list_2)

#         # # Create the subscriber to receive dx adn dy
#         rospy.Subscriber(quad_ros_namespace1 + '/shield_orders_pos', ShieldOutput, shield_orders_callback1)
#         rospy.Subscriber(quad_ros_namespace2 + '/shield_orders_pos', ShieldOutput, shield_orders_callback0)

#         while True:
#             if 'position_change1' in globals() and 'position_change0' in globals():
#                 action1 = position_change1.posChange[0]
#                 action0 = position_change0.posChange[1]
#                 # del globals()['position_change1']
#                 # del globals()['position_change0']
#                 break
#             else:
#                 pass

#     # ---------------------------------- End new code for implementing the position change with seperate node ----------------------------------- #

#         # Update traj
#         temp_s1 = Point(x_traj_s1[counter] , y_traj_s1[counter] + action1, 2)
#         x_traj_s1.append(temp_s1.x)
#         y_traj_s1.append(temp_s1.y)
#         z_traj_s1.append(temp_s1.z)

#         temp_s0 = Point(x_traj_s0[counter] + action0, y_traj_s0[counter], 2)
#         x_traj_s0.append(temp_s0.x)
#         y_traj_s0.append(temp_s0.y)
#         z_traj_s0.append(temp_s0.z)

#         counter = counter + 1

#         # Make the trajectory list for s1(uav1) and s0(uav2)
#         pva_list_1 = generate_traj_3d(x=x_traj_s1[-2:] , y=y_traj_s1[-2:] , z=z_traj_s1[-2:] , traj_time=[0,time_traj] , corr=None , freq = freq)
#         pva_list_2 = generate_traj_3d(x=x_traj_s0[-2:] , y=y_traj_s0[-2:] , z=z_traj_s0[-2:] , traj_time=[0,time_traj] , corr=None , freq = freq)

#         # Send the generated traj to the vehicles one at a time
#         for i in range(len(pva_list_1.pva)):
#             send_pva_pub_1.publish(pva_list_1.pva[i])
#             # wait_rate.sleep()
#         for j in range(len(pva_list_2.pva)):
#             send_pva_pub_2.publish(pva_list_2.pva[i])
#             wait_rate.sleep()



#     # Land the vehicle
#     start_landing(quad_name= quad_ros_namespace1)
#     start_landing(quad_name= quad_ros_namespace2)