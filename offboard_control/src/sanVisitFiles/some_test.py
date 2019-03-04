#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from Grid_State_Conversions import *
from actionlib_msgs.msg import GoalStatusArray
# import sys
# sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# from qcontrol_defs.msg import *
# from utils_functions import *
# from std_msgs.msg import Bool
# import simplejson as json

# def callback(msg):
#     global odom_msg
#     odom_msg = msg

# def quad_pos_callback(msg):
#     global quad_pos_msg
#     quad_pos_msg = msg

















if __name__ == "__main__":
    # Init node
    # rospy.init_node('test_conversion')
    # rospy.sleep(0.5)
    # rospy.Subscriber('/segway/odometry/local_filtered', Odometry, callback)
    # rospy.sleep(0.5)

    # ''' This is basically the "sensor" that the segway has '''
    # # Get the params needed for the monitoring the quads position
    # quad_ros_namespace = rospy.get_param('/Quad1_action/quad_ros_namespace')
    # xOffset = rospy.get_param('/Quad1_action/xOffset')
    # yOffset = rospy.get_param('/Quad1_action/yOffset')
    # # Get the quads pose
    # rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, quad_pos_callback)
    # rospy.sleep(0.1)

    base = Point()
    base.x = -15
    base.y = -19
    maximum = Point()
    maximum.x = 15
    maximum.y = 15
    converter = Grid(34,31,base,maximum)

    while not rospy.is_shutdown():
        # segway_point = Point()
        # segway_point.x = odom_msg.pose.pose.position.x
        # segway_point.y = odom_msg.pose.pose.position.y
        # state = converter.vicon2state(segway_point)
        # new_pose = converter.state2vicon(state)



        # This is the work of checking the quads state.
        quad_gaz_point = Point()
        # quad_gaz_point.x = quad_pos_msg.pose.position.x + xOffset
        # quad_gaz_point.y = quad_pos_msg.pose.position.y + yOffset
        quad_gaz_point.x = -1.4
        quad_gaz_point.y = -2.5
        quad_state = converter.vicon2state(quad_gaz_point)

        # print 'Printing the quads point:'
        # print quad_gaz_point

        print 'Printing the quads state'
        print quad_state

        # init_pose_target = converter.state2vicon(584)
        # init_pose_agent = converter.state2vicon(708)

        # print 'The segway state is'
        # print state

        # print 'The segway pose from converter is'
        # print new_pose
        rospy.sleep(2.)



