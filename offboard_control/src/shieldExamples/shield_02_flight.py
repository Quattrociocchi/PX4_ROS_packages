#!/usr/bin/env python

# Uses shield_01.launch and shield_01_mavros_sitl.launch with appropriate initial conditions.

import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')

import csv
import rospy
from qcontrol_defs.msg import *
from utils_functions import *
import simplejson as json

##############################

def shield_orders_callback1(msg):
    global position_change1
    position_change1 = msg

def shield_orders_callback0(msg):
    global position_change0
    position_change0 = msg

def card2shield_input_msg(card):
    shield_input_msg = ShieldInput()
    shield_input_msg.positiveX = False
    shield_input_msg.negativeX = False
    shield_input_msg.positiveY = False
    shield_input_msg.negativeY = False
    shield_input_msg.positiveZ = False
    shield_input_msg.negativeZ = False

    if card == 0:
        pass
    elif card == 1:
        shield_input_msg.positiveX = True # North +x
    elif card == 2:
        shield_input_msg.negativeX = True # South -x
    elif card == 3:
        shield_input_msg.negativeY = True # West -y
    elif card == 4:
        shield_input_msg.positiveY = True # East +y
    else:
        print('Error. The variable "card" must be an integer from the set {0,1,2,3,4} = {R,N,S,W,E}.')

    return shield_input_msg

# def card2boolList(card):
#   shield_bool_list = [False] * 6
#   if card == 0:
#       pass
#   elif card == 1:
#       shield_bool_list[0] = True # North +x
#   elif card == 2:
#       shield_bool_list[1] = True # South -x
#   elif card == 3:
#       shield_bool_list[3] = True # West -y
#   elif card == 4:
#       shield_bool_list[2] = True # East +y
#   else:
#       print('Error. The variable card must be an integer from the set {0,1,2,3,4} = {R,N,S,W,E}.')

#     return shield_bool_list


def state2coord(ncols,state):
    # Assumes that the first state is 0
    coords = [0, 0, 0]
    coords[0] = state / ncols
    coords[1] = state % ncols
    coords[2] = 2  # Specific to the current problem. Change as needed
    return coords   # List of integers


def coord2state(ncols,coords):
    # Takes in only x (coords[0]) and y (coords[1])
    s = coords[1] * ncols + coords[0]
    return s


def writeJson(infile,outfile,dict=None):
    if dict is None:
        dict = parseJson(infile)
    j = json.dumps(dict, indent=1)
    f = open(outfile, 'w')
    print >> f, j
    f.close()

def parseJson(filename):
    automaton = dict()
    file = open(filename)
    data = json.load(file)
    file.close()
    variables = dict()
    for var in data['variables']:
            v = var.split('@')[0]
            if v not in variables.keys():
                for var2ind in range(data['variables'].index(var),len(data['variables'])):
                    var2 = data['variables'][var2ind]
                    if v != var2.split('@')[0]:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2)]
                        break
                    if data['variables'].index(var2) == len(data['variables'])-1:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2) + 1]

    for s in data['nodes'].keys():
        automaton[int(s)] = dict.fromkeys(['State','Successors'])
        automaton[int(s)]['State'] = dict()
        automaton[int(s)]['Successors'] = []
        for v in variables.keys():
            if variables[v][0] == variables[v][1]:
                bin  = [data['nodes'][s]['state'][variables[v][0]]]
            else:
                bin = data['nodes'][s]['state'][variables[v][0]:variables[v][1]]
            automaton[int(s)]['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
            automaton[int(s)]['Successors'] = data['nodes'][s]['trans']
    return automaton



def computeAutomatonState(automaton,currstate,state):
    allautstates = set()
    for autstate in automaton[currstate]['Successors']:
        check = 0
        for var in state.keys():
            if automaton[autstate]['State'][var] == state[var] or state[var] == None:
                check+=1
            else:
                break
        if check == len(state.keys()):
            allautstates.add(autstate)
    return allautstates

##############################

# PVA control is more accurate than basic position control
# since it considers time constraints between each waypoint and does
# not just rely on the control loop delay implemented in PX4
if __name__ == "__main__":

    # Get the automaton from the json file
    directory = '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/jsonFiles'

    jsonInputFile = directory + '/example2.json'
    A = parseJson(jsonInputFile)
    
    # readableOutputFile = directory + '/sheildFirstOutput'
    # writeJson(jsonInputFile,readableOutputFile)
    # sys.exit()
    ##############################

    # Initialize node.
    rospy.init_node('Quad_offboard_pvacontrol')

    # Should be the prefix added to the offboard control node (prefix of all mavros topics)
    # for example "" or "/Quad9" or "/Quad10" (or "/uav1" for px4)
    quad_ros_namespace1 = "/uav1"
    quad_ros_namespace2 = "/uav2"

    # Create a publisher to send target positions
    send_pva_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/qcontrol/pva_control', PVA , queue_size=10)
    send_pva_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/qcontrol/pva_control', PVA , queue_size=10)

    # Create a publisher to send shield boolean message
    send_shield_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/shield_bool_list',ShieldInput, queue_size=10)
    send_shield_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/shield_bool_list',ShieldInput, queue_size=10)

    # Start position control with taking off at the beginning
    start_pva_control(quad_name= quad_ros_namespace1 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace2 , takeoff_before= True)

    ##############################

    # Set time parameters
    time_traj = 1.25   # Seconds
    freq = 10
    wait_rate = rospy.Rate(freq)

    ##############################

    # Set the direction that the quad will try to fly
    uLoc1_traj = [0, 0, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 0, 1]   # North-South
    uLoc0_traj = [0, 0, 4, 4, 4, 3, 4, 4, 3, 3, 3, 3, 3, 3]   # East-West
    ncols = 5   # Determined by grid size

    # Define traj's for later
    traj_s1 = []
    traj_s0 = []

    ##############################

    # Set offsets and initial positions (local frame)
    xOffset_s1 = 2
    yOffset_s1 = 0
    xOffset_s0 = 0
    yOffset_s0 = 2

    initial_pos = Point(0,0,2)

    # Set initial positions
    x_traj_s1 = [initial_pos.x]
    y_traj_s1 = [initial_pos.y]
    z_traj_s1 = [initial_pos.z]

    x_traj_s0 = [initial_pos.x]
    y_traj_s0 = [initial_pos.y]
    z_traj_s0 = [initial_pos.z]

    ##############################



    ns = 0 # Value of the next state
    counter = 0
    for uLoc1, uLoc0 in zip(uLoc1_traj,uLoc0_traj):

        # Check to see if the quads will violate the safety specification
        for k in A[ns]['Successors']:
            if A[k]['State']['uloc0'] == uLoc0 and A[k]['State']['uloc1'] == uLoc1:
                ns = k
                break

        # These are cardinal directions
        uShield1 = A[ns]['State']['ushield1'] # Movements North or South (y)
        uShield0 = A[ns]['State']['ushield0'] # Movements East or West (x)

# ---------------------------------- Start new code for implementing the position change with seperate node ----------------------------------- #
    # Has the order {+x, -x, +y, -y, +z, -z}
    bool_list_1 = card2shield_input_msg(uShield1)
    bool_list_2 = card2shield_input_msg(uShield0)

    ############### Code for figuring out the dx and dy ####################
    print('Sleeping for 0.5 seconds')
    rospy.sleep(0.5)
    send_shield_pub_1.publish(bool_list_1)
    send_shield_pub_2.publish(bool_list_2)

    # Create the subscriber to receive dx adn dy
    print('Sleeping for 0.5 seconds')
    rospy.sleep(0.5)
    rospy.Subscriber(quad_ros_namespace1 + '/shield_orders_pos', ShieldOutput, shield_orders_callback1)
    rospy.Subscriber(quad_ros_namespace2 + '/shield_orders_pos', ShieldOutput, shield_orders_callback0)

    print('Sleeping for 1 second')
    rospy.sleep(1)
    action1 = position_change1.posChange[1]
    action0 = position_change0.posChange[0]

    print(position_change1)
    print(position_change0)
    sys.exit()

# ---------------------------------- End new code for implementing the position change with seperate node ----------------------------------- #

    # Update traj
    temp_s1 = Point(x_traj_s1[counter] , y_traj_s1[counter] + action1, 2)
    x_traj_s1.append(temp_s1.x)
    y_traj_s1.append(temp_s1.y)
    z_traj_s1.append(temp_s1.z)

    temp_s0 = Point(x_traj_s0[counter] + action0, y_traj_s0[counter], 2)
    x_traj_s0.append(temp_s0.x)
    y_traj_s0.append(temp_s0.y)
    z_traj_s0.append(temp_s0.z)

    counter = counter + 1

    # Make the trajectory list for s1(uav1) and s0(uav2)
    pva_list_1 = generate_traj_3d(x=x_traj_s1[-2:] , y=y_traj_s1[-2:] , z=z_traj_s1[-2:] , traj_time=[0,time_traj] , corr=None , freq = freq)
    pva_list_2 = generate_traj_3d(x=x_traj_s0[-2:] , y=y_traj_s0[-2:] , z=z_traj_s0[-2:] , traj_time=[0,time_traj] , corr=None , freq = freq)

    # Send the generated traj to the vehicles one at a time
    for i in range(len(pva_list_1.pva)):
        send_pva_pub_1.publish(pva_list_1.pva[i])
        # wait_rate.sleep()
    for j in range(len(pva_list_2.pva)):
        send_pva_pub_2.publish(pva_list_2.pva[i])
        wait_rate.sleep()



    # Land the vehicle
    start_landing(quad_name= quad_ros_namespace1)
    start_landing(quad_name= quad_ros_namespace2)