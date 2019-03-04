#!/usr/bin/env python
''' Actions for the shield '''

import sys
import math
import rospy
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# ------------------------------------------------------------------ #

def shield_action_callback(msg):
    global shield_action_orders
    shield_action_orders = msg.action

def current_pos_callback(msg):
    global current_pos_msg
    current_pos_msg = msg

# ------------------------------------------------------------------ #

if __name__ == "__main__":
    # Initialize node.
    rospy.init_node('quad_action', anonymous=True)
    
    # Get the appropriate quad name
    quad_ros_namespace = rospy.get_param('~quad_ros_namespace')
    xOffset = rospy.get_param('~xOffset')
    yOffset = rospy.get_param('~yOffset')
    freq = rospy.get_param('~freq')
    time_full_traj = rospy.get_param('~time_full_traj')
    wait_rate = rospy.Rate(freq)
    z_nom = 2

    # Create a publisher to send target positions
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    send_trajComplete_pub = rospy.Publisher(quad_ros_namespace + '/trajComplete', Bool, queue_size=10)

    # Instantiate a converter
    base = Point()
    base.x = -15
    base.y = -19
    maximum = Point()
    maximum.x = 15
    maximum.y = 15
    converter = Grid(34,31,base,maximum)

    while not rospy.is_shutdown():
        rospy.Subscriber(quad_ros_namespace + '/shield_action_orders', ShieldOutput , shield_action_callback)

        if 'shield_action_orders' in globals():
            # Notify that trajectory is incomplete
            trajCompleted = Bool()
            trajCompleted = False
            send_trajComplete_pub.publish(trajCompleted)

            action = shield_action_orders[0]
            # ------- Fly the predetermined trajectory ------- #
            rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, current_pos_callback)
            rospy.sleep(0.1)
            current_pos = current_pos_msg.pose

            # ----------------------------- Choose the action ----------------------------- #
            if action == 0:
                # # This action maintains current position
                # x_traj = [current_pos.position.x, current_pos.position.x]
                # y_traj = [current_pos.position.y, current_pos.position.y]
                # z_traj = [current_pos.position.z, current_pos.position.z]
                point_from_converter = converter.state2vicon(553)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 1:
                point_from_converter = converter.state2vicon(552)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 2:
                point_from_converter = converter.state2vicon(551)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom) 
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 3:
                point_from_converter = converter.state2vicon(550)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 4:
                point_from_converter = converter.state2vicon(549)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 5:
                point_from_converter = converter.state2vicon(548)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 6:
                point_from_converter = converter.state2vicon(547)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 7:
                point_from_converter = converter.state2vicon(546)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 8:
                point_from_converter = converter.state2vicon(545)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 9:
                point_from_converter = converter.state2vicon(544)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 10:
                point_from_converter = converter.state2vicon(543)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 11:
                point_from_converter = converter.state2vicon(542)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 12:
                point_from_converter = converter.state2vicon(541)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 13:
                point_from_converter = converter.state2vicon(540)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 14:
                point_from_converter = converter.state2vicon(539)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 15:
                point_from_converter = converter.state2vicon(538)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 16:
                point_from_converter = converter.state2vicon(537)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 17:
                point_from_converter = converter.state2vicon(536)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]
                
            elif action == 18:
                point_from_converter = converter.state2vicon(535)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 19:
                point_from_converter = converter.state2vicon(534)
                wp1 = Point(point_from_converter.x, point_from_converter.y, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            # elif action == 20:
                

            # elif action == 21:
            #     # This action flies a circle
            #     x_traj = [current_pos.position.x]
            #     y_traj = [current_pos.position.y]
            #     z_traj = [current_pos.position.z]

            #     delta_theta = math.pi / 8
            #     for ii in range(1, 16):
            #         x_traj.append(current_pos.position.x + 0.75*math.cos(ii * delta_theta) - 1)
            #         y_traj.append(current_pos.position.y + 0.75*math.sin(ii * delta_theta))
            #         z_traj.append(current_pos.position.z)

            else:
                # This does not work. Why?
                print('The action choice you have made is not valid. Try again you crazy cat 8-D')
                pass

            # -------------------------------------------------------------------------------- #

            # Find the complete minimum snap trajectory
            pva_list = generate_traj_3d(x=x_traj , y=y_traj , z=z_traj , traj_time=[0,time_full_traj] , corr=None , freq = freq)

            # Send the generated traj to the vehicle
            for elem in pva_list.pva:
                send_pva_pub.publish(elem)
                wait_rate.sleep()

            # Notify that trajectory is complete
            trajCompleted = True
            send_trajComplete_pub.publish(trajCompleted)

            del globals()['shield_action_orders']
            del globals()['current_pos_msg']

        else:
            pass
