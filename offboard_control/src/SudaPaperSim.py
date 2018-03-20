#!/usr/bin/env python

import sys
sys.path.insert( 0 , '~/catkin_ws/src/offboard_control/src')


import rospy
from qcontrol_defs.msg import *
from utils_functions import *
from mavros_msgs.msg import HomePosition
# from geometry_msgs.msg import PoseStamped


# def curr_pos_callback(msg):
#     global current_position
#     current_position = msg.pose.pose.position


# PVA control is more accurate than just basics position control
# since it considers time constraints between each waypoint and does
# not just rely on the control loop delay implemented in PX4
if __name__ == "__main__":

    # Pull in the trajectory
    traj1 = []
    traj2 = []
    traj3 = []
    traj4 = []

    import csv

    with open('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/statehistory_target.txt','rb') as csvfile:
        totalstate1 = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in totalstate1:
             traj1.append(map(int,row))

    with open('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/statehistory1.txt','rb') as csvfile:
        totalstate2 = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in totalstate2:
             traj2.append(map(int,row))

    with open('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/statehistory2.txt','rb') as csvfile:
        totalstate3 = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in totalstate3:
             traj3.append(map(int,row))

    with open('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/statehistory3.txt','rb') as csvfile:
        totalstate4 = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in totalstate4:
             traj4.append(map(int,row))


    # Initialize node.
    rospy.init_node('Quad_offboard_pvacontrol')



    #### TIME SET
    # Set time stuff
    time_full_traj = 200 #seconds
    freq = 20
    wait_rate = rospy.Rate(freq)



    # Should be the prefix added to the offboard control node (prefix of all mavros topics)
    # for example "" or "/Quad9" or "/Quad10" (or "/uav1" for px4)
    quad_ros_namespace1 = "/uav1"
    quad_ros_namespace2 = "/uav2"
    quad_ros_namespace3 = "/uav3"
    quad_ros_namespace4 = "/uav4"


    # Create a publisher to send target positions
    send_pva_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/qcontrol/pva_control', PVA , queue_size=10)
    send_pva_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/qcontrol/pva_control', PVA , queue_size=10)
    send_pva_pub_3 = rospy.Publisher(quad_ros_namespace3 + '/qcontrol/pva_control', PVA , queue_size=10)
    send_pva_pub_4 = rospy.Publisher(quad_ros_namespace4 + '/qcontrol/pva_control', PVA , queue_size=10)


    #### INITIAL POSITION SET
    # Set up publishers for each quad
    send_init_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/mavros/home_position/set', HomePosition, queue_size=10)
    send_init_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/mavros/home_position/set', HomePosition, queue_size=10)
    send_init_pub_3 = rospy.Publisher(quad_ros_namespace3 + '/mavros/home_position/set', HomePosition, queue_size=10)
    send_init_pub_4 = rospy.Publisher(quad_ros_namespace4 + '/mavros/home_position/set', HomePosition, queue_size=10)
    ############# THIS IS SOMETHING I TRIED BUT ALSO DID NOT WORK #############
    # send_init_pub_1 = rospy.Publisher(quad_ros_namespace1 + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # send_init_pub_2 = rospy.Publisher(quad_ros_namespace2 + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # send_init_pub_3 = rospy.Publisher(quad_ros_namespace3 + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # send_init_pub_4 = rospy.Publisher(quad_ros_namespace4 + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    # Set up the HomePosition message for each of the quads
    p1 = HomePosition()
    p1.position.x = 7
    p1.position.y = 16
    # p1.position.z = 2
    # p1.orientation.x = 1
    print p1
    p2 = HomePosition()
    p2.position.x = 9
    p2.position.y = 5
    # p2.position.z = 2
    # p2.orientation.x = 1
    print p2
    p3 = HomePosition()
    p3.position.x = 2
    p3.position.y = 9
    # p3.position.z = 2
    # p3.orientation.x = 1
    print p3
    p4 = HomePosition()
    p4.position.x = 10
    p4.position.y = 17
    # p4.position.z = 2
    # p4.orientation.x = 1
    print p4

    ############# THIS IS SOMETHING I TRIED BUT ALSO DID NOT WORK #############
    # p1 = PoseStamped()
    # p1.pose.position.x = 7
    # p1.pose.position.y = 16
    # print p1
    # p2 = PoseStamped()
    # p2.pose.position.x = 9
    # p2.pose.position.y = 5
    # print p2
    # p3 = PoseStamped()
    # p3.pose.position.x = 2
    # p3.pose.position.y = 9
    # print p3
    # p4 = PoseStamped()
    # p4.pose.position.x = 10
    # p4.pose.position.y = 17
    # print p4

    # Publish the message to set the home position with mavros. This must be done for some length of time in order for it to make it through.
    for c in range(0,10000):
        send_init_pub_1.publish(p1)
        send_init_pub_2.publish(p2)
        send_init_pub_3.publish(p3)
        send_init_pub_4.publish(p4)

    # Make point out of the initial position
    initial_pos_1 = Point(7,16,2)
    initial_pos_2 = Point(9,5,2)
    initial_pos_3 = Point(2,9,2)
    initial_pos_4 = Point(10,17,2)

    # Set initial positions
    x_traj_1 = [initial_pos_1.x]
    y_traj_1 = [initial_pos_1.y]
    z_traj_1 = [initial_pos_1.z]

    x_traj_2 = [initial_pos_2.x]
    y_traj_2 = [initial_pos_2.y]
    z_traj_2 = [initial_pos_2.z]

    x_traj_3 = [initial_pos_3.x]
    y_traj_3 = [initial_pos_3.y]
    z_traj_3 = [initial_pos_3.z]

    x_traj_4 = [initial_pos_4.x]
    y_traj_4 = [initial_pos_4.y]
    z_traj_4 = [initial_pos_4.z]


    # Start position control with taking off at the beginning
    start_pva_control(quad_name= quad_ros_namespace1 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace2 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace3 , takeoff_before= True)
    start_pva_control(quad_name= quad_ros_namespace4 , takeoff_before= True)



    # For loop that sets the trajectory that is sent out to gazebo
    for s in traj1[:20]:
        temp = Point(s[0] , s[1] , 2)
        x_traj_1.append(temp.x)
        y_traj_1.append(temp.y)
        z_traj_1.append(temp.z)

    for s in traj2[:20]:
        temp = Point(s[0] , s[1] , 2)
        x_traj_2.append(temp.x)
        y_traj_2.append(temp.y)
        z_traj_2.append(temp.z)

    for s in traj3[:20]:
        temp = Point(s[0] , s[1] , 2)
        x_traj_3.append(temp.x)
        y_traj_3.append(temp.y)
        z_traj_3.append(temp.z)

    for s in traj4[:20]:
        temp = Point(s[0] , s[1] , 2)
        x_traj_4.append(temp.x)
        y_traj_4.append(temp.y)
        z_traj_4.append(temp.z)    

    # Make the list that is sent out using this nifty function(?) thing...
    pva_list_1 = generate_traj_3d(x=x_traj_1 , y=y_traj_1 , z=z_traj_1 , traj_time=[0,time_full_traj] , corr=None , freq = freq)
    pva_list_2 = generate_traj_3d(x=x_traj_2 , y=y_traj_2 , z=z_traj_2 , traj_time=[0,time_full_traj] , corr=None , freq = freq)
    pva_list_3 = generate_traj_3d(x=x_traj_3 , y=y_traj_3 , z=z_traj_3 , traj_time=[0,time_full_traj] , corr=None , freq = freq)
    pva_list_4 = generate_traj_3d(x=x_traj_4 , y=y_traj_4 , z=z_traj_4 , traj_time=[0,time_full_traj] , corr=None , freq = freq)
    #print pva_list_1.pva[:30]
    # sys.exit()

    # Send the generated traj to the vehicle
    for i in range(len(pva_list_1.pva)) :
        send_pva_pub_1.publish(pva_list_1.pva[i])
        send_pva_pub_2.publish(pva_list_2.pva[i])
        send_pva_pub_3.publish(pva_list_3.pva[i])
        send_pva_pub_4.publish(pva_list_4.pva[i])
        wait_rate.sleep()



    # Land the vehicle
    start_landing(quad_name= quad_ros_namespace1)
    start_landing(quad_name= quad_ros_namespace2)
    start_landing(quad_name= quad_ros_namespace3)
    start_landing(quad_name= quad_ros_namespace4)

    

    # ####### Initialize grid specifications #################
    # GRID_SIZE     = 1
    # Z_ALTITUDE    = 2.0
    # X_NUMBER_TILE = 10   # X correspond to the larger size -> width
    # Y_NUMBER_TILE = 10   # Y height correspond to the smaller -> height

    # # the point at the left down corner in your coordinate system
    # base = Point()
    # base.x = -5
    # base.y = -5
    # base.z = Z_ALTITUDE

    # #
    # maxi = Point()
    # maxi.x = GRID_SIZE * X_NUMBER_TILE + base.x
    # maxi.y = GRID_SIZE * Y_NUMBER_TILE + base.y
    # maxi.z = Z_LEVEL

    # # Grid creation
    # grid = Grid(X_NUMBER_TILE ,Y_NUMBER_TILE , base , maxi)

    # print grid.x , grid.y , grid.base , grid.maximum , grid.blocklengthX , grid.blocklengthY

    # ####### End grid specifications #############################
