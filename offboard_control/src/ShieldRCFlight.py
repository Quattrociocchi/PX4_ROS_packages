#!/usr/bin/env python

import sys
sys.path.insert( 0 , '~/catkin_ws/src/offboard_control/src')

import csv
import rospy
from qcontrol_defs.msg import *
from utils_functions import *
# from mavros_msgs.msg import HomePosition
# from geometry_msgs.msg import PoseStamped
import simplejson as json

##############################

# def curr_pos_callback(msg):
#     global current_position
#     current_position = msg.pose.pose.position

def card2coordAction(card,action):
    if card == 0:
        action = 0
    elif card == 1:
        action = -1
    elif card == 2:
        action = 1
    elif card == 3:
        action = -1
    elif card == 4:
        action = 1
    else:
        print('Error. The variable card must be an integer from the set {0,1,2,3,4} = {R,N,S,W,E}.')

    return action

def state2coord(ncols,state):
    # Assumes that the first state is 0
    coords = [0, 0, 0]
    coords[0] = state % ncols
    coords[1] = state / ncols
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
    directory = '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src'

    jsonInputFile = directory + '/example2.json'
    readableOutputFile = directory + '/sheildFirstOutput'

    A = parseJson(jsonInputFile)

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

    # Start position control with taking off at the beginning
    start_pva_control(quad_name= quad_ros_namespace1 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace2 , takeoff_before= True)

    ##############################

    # Set time parameters
    time_traj = 3   # Seconds
    freq = 40
    wait_rate = rospy.Rate(freq)

    ##############################

    # Set the direction that the quad will try to fly
    uLoc1_traj = [1, 1, 1, 1]   # North-South
    uLoc0_traj = [4, 4, 4, 4]   # East-West
    ncols = 5   # Determined by grid size
    # Defien traj's for later
    traj_s1 = []
    traj_s0 = []

    ##############################

    # Set offsets and initial positions (local frame)
    xOffset_s1 = 2
    yOffset_s1 = 4
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
    for uLoc1, uLoc0 in zip(uLoc1_traj,uLoc0_traj):

        # Check to see if the quads will violate the safety specification
        for k in A[ns]['Successors']:
            if A[k]['State']['uloc0'] == uLoc0 and A[k]['State']['uloc1'] == uLoc1:
                ns = k
                break

        # These are cardinal directions
        uShield1 = A[ns]['ushield1']
        uShield0 = A[ns]['ushield0']

        # 


        traj_s1.append(state2coord(ncols,p))
        traj_s0.append(state2coord(ncols,p))

        # ######## This will probably be done afer checking the allowable states.....
        temp_s1 = Point(s1[0] - xOffset_s1 , s1[1] - yOffset_s1 , 2)
        x_traj_s1.append(temp_s1.x)
        y_traj_s1.append(temp_s1.y)
        z_traj_s1.append(temp_s1.z)

        temp_s0 = Point(s0[0] - xOffset_s0 , s0[1] - yOffset_s0 , 2)
        x_traj_s0.append(temp_s0.x)
        y_traj_s0.append(temp_s0.y)
        z_traj_s0.append(temp_s0.z)





        # Create a point out of the selected next action (point the agent should move to)
        p2 = A[ns]['State']['s']
        traj2.append(state2coord(ncols,p2))
        temp_2 = Point(traj2[-1][0] - xOffset_2 , traj2[-1][1] - yOffset_2 , 2)
        x_traj_2.append(temp_2.x)
        y_traj_2.append(temp_2.y)
        z_traj_2.append(temp_2.z)

        # print(p2)
        # variable = raw_input('input anything to move on: ')

        # Make the trajectory list for target(1) and agent (2)
        pva_list_1 = generate_traj_3d(x=y_traj_1[-2:] , y=x_traj_1[-2:] , z=z_traj_1[-2:] , traj_time=[0,time_traj] , corr=None , freq = freq)
        pva_list_2 = generate_traj_3d(x=y_traj_2[-2:] , y=x_traj_2[-2:] , z=z_traj_2[-2:] , traj_time=[0,time_traj] , corr=None , freq = freq)

        # print(pva_list_1)

        # Send the generated traj to the vehicles one at a time
        for i in range(len(pva_list_1.pva)):
            send_pva_pub_1.publish(pva_list_1.pva[i])
            wait_rate.sleep()
        for j in range(len(pva_list_2.pva)):
            send_pva_pub_2.publish(pva_list_2.pva[i])
            # wait_rate.sleep()



    # Land the vehicle
    start_landing(quad_name= quad_ros_namespace1)
    start_landing(quad_name= quad_ros_namespace2)