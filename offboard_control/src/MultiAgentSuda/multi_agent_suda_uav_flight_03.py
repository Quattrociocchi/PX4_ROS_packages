#!/usr/bin/env python

''' UAV Flight Controller for multi-agent sim with suda. March 2019 '''

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

from gridworld import *
import cv2
import subprocess
import time
import copy
import itertools

# ------------------------------------------------------------------ #

def quad_pos_callback(msg):
    global quad_current_pos
    quad_current_pos = msg.pose.position

def segway_pos_callback(msg):
    global segway_current_pos
    segway_current_pos = msg.pose.pose.position

def get_segway_state(converter):
    segway_gaz_point = Point() # Do not want the z component
    segway_gaz_point.x = segway_current_pos.x
    segway_gaz_point.y = segway_current_pos.y
    segway_state = converter.vicon2state(segway_gaz_point)
    return segway_state

def get_quad_state(converter, xOffset, yOffset):
    quad_gaz_point = Point() # Do not want the z component
    quad_gaz_point.x = quad_current_pos.x + xOffset
    quad_gaz_point.y = quad_current_pos.y + yOffset
    quad_state = converter.vicon2state(quad_gaz_point)
    return quad_state

def get_quad_pos_from_state(converter, state, xOffset, yOffset, zOffset, z_nom):
    point_from_converter = converter.state2vicon(state)
    wp = Point(point_from_converter.x, point_from_converter.y, z_nom)
    x_traj = [quad_current_pos.x, wp.x - xOffset]
    y_traj = [quad_current_pos.y, wp.y - yOffset]
    z_traj = [quad_current_pos.z, wp.z - zOffset]
    return x_traj, y_traj, z_traj

def parseJson(filename):
    automaton = dict()
    file = open(filename)
    data = json.load(file)
    file.close()
    variables = dict()
    for var in data['variables']:
        v = var.split('@')[0]
        if v not in variables.keys():
            for var2ind in range(data['variables'].index(var), len(data['variables'])):
                var2 = data['variables'][var2ind]
                if v != var2.split('@')[0]:
                    variables[v] = [data['variables'].index(var), data['variables'].index(var2)]
                    break
                if data['variables'].index(var2) == len(data['variables']) - 1:
                    variables[v] = [data['variables'].index(var), data['variables'].index(var2)+1]

    for s in data['nodes'].keys():
        automaton[int(s)] = dict.fromkeys(['State','Successors','Predecessors'])
        automaton[int(s)]['State'] = dict()
        automaton[int(s)]['Successors'] = []
        automaton[int(s)]['Predecessors'] = []
        for v in variables.keys():
            bin = data['nodes'][s]['state'][variables[v][0]:variables[v][1]]
            automaton[int(s)]['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
            automaton[int(s)]['Successors'] = data['nodes'][s]['trans']
    for s in data['nodes'].keys():
        for t in automaton[int(s)]['Successors']:
            automaton[t]['Predecessors'].append(int(s))
    return automaton

def next_game_and_agent_state(automaton, game_state, target_state, iset, allowed_states, beliefcombs, pg,allstates):
    agent_state = automaton[game_state]['State']['s']
    is_vis = not invis_check(agent_state, target_state, iset) # This should be true if the agent state is visible from the target state

    new_game_state = None
    if is_vis and target_state in allowed_states:
        for k in automaton[game_state]['Successors']:
            if automaton[k]['State']['st'] == target_state:
                new_game_state = k
                break
    else:
        for k in automaton[game_state]['Successors']:
            next_target_state = automaton[k]['State']['st']
            if next_target_state != allstates[-1] and next_target_state >= 1681:
                next_beliefs = beliefcombs[next_target_state - 1681]
                if any(target_state in pg[x] for x in next_beliefs) or target_state not in allowed_states:
                    new_game_state = k
        if new_game_state == None:
            print '-------------------_> your here'
            for k in automaton[game_state]['Successors']: 
                next_target_state = automaton[k]['State']['st']
                # print '____________________>' , k
                # print '********************>' , quad_ros_namespace
                # print '-------------------->', next_target_state
                if next_target_state == allstates[-1]:
                    new_game_state = k
                    # print '_-_-_-_-_-_-_-_-_->' , new_game_state
                    break
                
    assert new_game_state != None, 'No successor found!'

    new_agent_state = automaton[new_game_state]['State']['s']
    return new_game_state, new_agent_state

def invis_check(agent_state, target_state, iset):
    return (target_state in iset[agent_state])

def powerset(s):
    x = len(s)
    a = []
    for i in range(1,1<<x):
        a.append({s[j] for j in range(x) if (i &(1<<j))})
    return a

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
    base_x = rospy.get_param('~base_x')
    base_y = rospy.get_param('~base_y')
    max_x = rospy.get_param('~max_x')
    max_y = rospy.get_param('~max_y')
    nb_y = rospy.get_param('~nb_y')
    nb_x = rospy.get_param('~nb_x')
    z_nom = 1.5
    controller_file = rospy.get_param('~controller_file')
    AbsPath = rospy.get_param('~Abs_file_path')
    Allowed_states_pickle = rospy.get_param('~Allowed_states_pickle_file')
    Invis_states_pickle = rospy.get_param('~Invis_states_pickle_file')

    # Start by taking off
    rospy.sleep(0.5)
    start_pva_control(quad_name= quad_ros_namespace , takeoff_before= True)
    rospy.sleep(0.5)




##########################################################  NEED to clean this up at some point  ##################

    # Make sure to state the agent and the target far enough from each other such thehat the games initial conditions do not violate safety.
    mapname = 'Q_building_3'
    scale = (41,41)
    filename = [AbsPath + mapname + '.pgm',scale,cv2.INTER_LINEAR_EXACT]


    ######################################
    
    nagents = 2
    targets = [[],[]]#[[946],[538],[],[],[]]
    initial = [797,777]
    moveobstacles = [637]
    gwg = Gridworld(filename,nagents=nagents, targets=targets, initial=initial, moveobstacles=moveobstacles)
    # allowed_states = [[None]] * nagents
    pg = [[None]]*nagents
    #################### Agent allowed states ####################
    # allowed_states[0] = set.union(*[set(range(586 + x * scale[0], 586 + x * scale[0] + 14)) for x in range(25)]).union(*[set(range(961 + x * scale[0], 961 + x * scale[0] + 25)) for x in range(15)])  - set(gwg.obstacles)
    # allowed_states[1] = set(gwg.states) - allowed_states[0] - set(gwg.obstacles)
    ################### Single Partition ###################
    # pg[0] = {0: set.union(*[set(range(586 + x * scale[0], 586 + x * scale[0] + 14)) for x in range(11)])  - set(gwg.obstacles), 1: set.union(*[set(range(1026 + x * scale[0], 1026 + x * scale[0] + 14)) for x in range(14)])  - set(gwg.obstacles),
    #          2: set.union(*[set(range(961 + x * scale[0], 961 + x * scale[0] + 12)) for x in range(15)])  - set(gwg.obstacles), 3: set.union(*[set(range(973 + x * scale[0], 973 + x * scale[0] + 13)) for x in range(15)])  - set(gwg.obstacles)}
    # pg[1] = {0: set.union(*[set(range(361 + x * scale[0], 361 + x * scale[0] + 14)) for x in range(14)])  - set(gwg.obstacles), 1: set.union(*[set(range(335 + x * scale[0], 335 + x * scale[0] + 11)) for x in range(16)])  - set(gwg.obstacles),
    #          2: set.union(*[set(range(66 + x * scale[0], 66 + x * scale[0] + 13)) for x in range(12)])  - set(gwg.obstacles), 3: set.union(*[set(range(52 + x * scale[0], 52 + x * scale[0] + 14)) for x in range(7)])  - set(gwg.obstacles),
    #          4: set.union(*[set(range(41 + x * scale[0], 41 + x * scale[0] + 11)) for x in range(7)]) - set(gwg.obstacles)}
    pg[0] = {0: set.union(*[set(range(601 + x * scale[0], 601 + x * scale[0] + 14)) for x in range(11)])  - set(gwg.obstacles), 1: set.union(*[set(range(1052 + x * scale[0], 1052 + x * scale[0] + 14)) for x in range(15)])  - set(gwg.obstacles),
         2: set.union(*[set(range(985 + x * scale[0], 985 + x * scale[0] + 12)) for x in range(16)])  - set(gwg.obstacles), 3: set.union(*[set(range(997 + x * scale[0], 997 + x * scale[0] + 14)) for x in range(16)])  - set(gwg.obstacles)}
    pg[1] = {0: set.union(*[set(range(370 + x * scale[0], 370 + x * scale[0] + 14)) for x in range(14)])  - set(gwg.obstacles), 1: set.union(*[set(range(343 + x * scale[0], 343 + x * scale[0] + 12)) for x in range(16)])  - set(gwg.obstacles),
         2: set.union(*[set(range(68 + x * scale[0], 68 + x * scale[0] + 13)) for x in range(12)])  - set(gwg.obstacles), 3: set.union(*[set(range(53 + x * scale[0], 53 + x * scale[0] + 15)) for x in range(7)])  - set(gwg.obstacles),
         4: set.union(*[set(range(42 + x * scale[0], 42 + x * scale[0] + 11)) for x in range(7)]) - set(gwg.obstacles)}
    # pg[2] = {0: allowed_states[2]}
    # pg[3] = {0:allowed_states[3]}

    ######################### Create sensor uncertainty dictionary #############################
    sensor_uncertainty = 1
    belief_ncols = gwg.ncols - sensor_uncertainty + 1
    belief_nrows = gwg.nrows - sensor_uncertainty + 1
    sensor_uncertain_dict = dict.fromkeys(range(belief_ncols * belief_nrows))
    for i in range(belief_nrows):
        for j in range(belief_ncols):
            belief_gridstate = i * belief_ncols + j
            sensor_uncertain_dict[belief_gridstate] = set()
            for srow in range(i, i + sensor_uncertainty):
                for scol in range(j, j + sensor_uncertainty):
                    gridstate = gwg.rcoords((srow, scol))
                    uset = list(itertools.product(['E', 'S', 'R'], repeat=sensor_uncertainty - 1))
                    for u in uset:
                        snext = copy.deepcopy(i * gwg.ncols + j)
                        for v in range(sensor_uncertainty - 1):
                            act = u[v]
                            snext = np.nonzero(gwg.prob[act][snext])[0][0]
                        # if gridstate not in iset[belief_gridstate]:
                        sensor_uncertain_dict[belief_gridstate].add(snext)
                    # sensor_uncertain_dict[belief_gridstate].add(gridstate)
    ##############################################################################################

    beliefcombs = [[None]]*2
    allstates = [[None]]*2
    beliefcombs[int(quad_ros_namespace[-1]) - 1] = powerset(pg[int(quad_ros_namespace[-1]) - 1].keys())
    allstates[int(quad_ros_namespace[-1]) - 1] = copy.deepcopy(sensor_uncertain_dict.keys())
    for i in range(belief_ncols * belief_nrows, belief_ncols * belief_nrows + len(beliefcombs[int(quad_ros_namespace[-1]) - 1])):
        allstates[int(quad_ros_namespace[-1]) - 1].append(i)
    allstates[int(quad_ros_namespace[-1]) - 1].append(len(allstates[int(quad_ros_namespace[-1]) - 1]))  # nominal state if target leaves allowed region


##########################################################################################################

    # Create a publisher to send target positions
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    # Create a publisher to notify when trajectory is comeplete
    send_trajComplete_pub = rospy.Publisher(quad_ros_namespace + '/trajComplete', Bool, queue_size=10)
    # Create subscriber for current position of quad from mavros (sensor)
    rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, quad_pos_callback)
    rospy.sleep(0.1)
    # Create subscriber to get segway position (sensor)
    rospy.Subscriber('/segway/odometry/local_filtered', Odometry, segway_pos_callback)
    rospy.sleep(0.1)

    # Instantiate a converter
    base = Point()
    base.x = base_x
    base.y = base_y
    maximum = Point()
    maximum.x = max_x
    maximum.y = max_y
    converter = Grid(nb_y,nb_x,base,maximum)

    # Generate the automaton from the json file produced by Slugs
    filename = AbsPath + controller_file
    automaton = parseJson(filename) # Dictionary

    # Load up allowed states pickle
    pickle_in = open(AbsPath + Allowed_states_pickle)
    # allowed_states_dict = pickle.load(pickle_in)
    allowed_states_dict = json.load(pickle_in)

    allowed_states_dict = {str(k):v for k,v in allowed_states_dict.items()}

    allowed_states = allowed_states_dict[quad_ros_namespace[1:]]

    # Set and determine initial states
    game_state = 0
    quad_state = get_quad_state(converter, xOffset, yOffset)
    segway_state = get_segway_state(converter)
    # Set the previous states to be the current states since this is the start of the sim
    prev_game_state = game_state
    prev_quad_state = quad_state
    prev_segway_state = segway_state

    # Load the invisible state dictionary
    pickle_in = open(AbsPath + Invis_states_pickle)
    # iset = pickle.load(pickle_in)
    iset = json.load(pickle_in)
    iset = {int(k):v for k,v in iset.items()}
    # Creating a window that will display the current state of the game, segway, and quad
    PIPE_PATH = "/tmp/my_pipe_" + quad_ros_namespace[1:]
    if not os.path.exists(PIPE_PATH):
        os.mkfifo(PIPE_PATH)
    Popen(['xterm', '-e', 'tail -f %s' % PIPE_PATH])

    # Start with writing the initial states
    with open(PIPE_PATH, "w") as p:
        p.write('The Game Initial State is: {}\n'.format(game_state))
        p.write('The Quad Initial State is: {}\n'.format(quad_state))
        p.write('The Segway Initial State is: {}\n'.format(segway_state))
        p.write('-------------------\n')

    # Open a file to record all states, and start with the initial
    stateRecord = open(AbsPath + 'StateRecord_multi_agent_' + quad_ros_namespace[1:] + '.txt', 'w+')
    stateRecord.truncate(0)
    stateRecord.write(('Game State: {0:4d}  |  '.ljust(21) + 'Segway State: {1:4d}  |  '.ljust(23) + 'Quad State: {2:4d}\n').format(game_state,segway_state,quad_state))
    stateRecord.close()

    with open(PIPE_PATH, "w") as p:
        p.write('\nThe Quad is now monitoring the position of the Segway\n')
        p.write('-------------------\n')
    while not rospy.is_shutdown():
        # Determine what state the segway is in
        segway_state = get_segway_state(converter)
        try:
            # # Determine the current game state and quad state based on the segways state and the previous quad state
            # # Also check to see if the segway is even in a state that this controller cares about. If it is not, then set it to the "i dont care" state of 1055 (specific to thsi problem)
            # -----> This will need to be included when there are no belief states. This will have to be fixed on synthesis side ------
            # if segway_state not in allowed_states:
            #     segway_state = allstates[int(quad_ros_namespace[-1]) - 1][-1]
            print quad_ros_namespace
            game_state, quad_state = next_game_and_agent_state(automaton, game_state, segway_state, iset, allowed_states, beliefcombs[int(quad_ros_namespace[-1]) - 1], pg[int(quad_ros_namespace[-1]) - 1],allstates[int(quad_ros_namespace[-1]) - 1])

        except AssertionError as error:
            print(error)
            quit_program = True

        if game_state != prev_game_state or quad_state != prev_quad_state or segway_state != prev_segway_state:
            # open and close the file each time because the simulation may end unexpectedly
            stateRecord = open(AbsPath + 'StateRecord_multi_agent_' + quad_ros_namespace[1:] + '.txt', 'a')
            stateRecord.write(('Game State: {0:4d}  |  '.ljust(21) + 'Segway State: {1:4d}  |  '.ljust(23) + 'Quad State: {2:4d}\n').format(game_state,segway_state,quad_state))
            stateRecord.close()
            with open(PIPE_PATH, "w") as p:
                p.write(('Game State: {0:4d}  |  '.ljust(21) + 'Segway State: {1:4d}  |  '.ljust(23) + 'Quad State: {2:4d}\n').format(game_state,segway_state,quad_state))
            # Update only the game and segway states since the quad state is used to enter the if statement that generates the traj for the quad
            prev_game_state = game_state
            prev_segway_state = segway_state
        
        try:
            quit_program
        except NameError:
            pass
        else:
            sys.exit()

        if quad_state != prev_quad_state:
            # Notify that trajectory is incomplete
            trajCompleted = Bool()
            trajCompleted = False
            send_trajComplete_pub.publish(trajCompleted)

            # Determine next cartesian position of the new quad state
            x_traj, y_traj, z_traj = get_quad_pos_from_state(converter, quad_state, xOffset, yOffset, zOffset, z_nom)

            # Find the complete minimum snap trajectory
            pva_list = generate_traj_3d(x=x_traj , y=y_traj , z=z_traj , traj_time=[0,time_full_traj] , corr=None , freq = freq)

            # Send the generated traj to the vehicle
            for elem in pva_list.pva:
                send_pva_pub.publish(elem)
                wait_rate.sleep()

            # Notify that trajectory is complete
            trajCompleted = True
            send_trajComplete_pub.publish(trajCompleted)

            # Update the previous quad state
            prev_quad_state = quad_state
        else:
            pass

    # Land the vehicle
    start_landing(quad_name= quad_ros_namespace)
