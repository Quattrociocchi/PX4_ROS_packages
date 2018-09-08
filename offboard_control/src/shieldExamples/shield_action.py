#!/usr/bin/env python


import rospy
import math
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
from geometry_msgs.msg import PoseStamped

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
    rospy.init_node('shield_action', anonymous=True)
    
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
    send_trajComplete_pub = rospy.Publisher(quad_ros_namespace + '/trajComplete', ShieldBool, queue_size=10)

    while not rospy.is_shutdown():
        rospy.Subscriber(quad_ros_namespace + '/shield_action_orders', ShieldOutput , shield_action_callback)

        if 'shield_action_orders' in globals():
            action = shield_action_orders[0]
            # ------- Fly the predetermined trajectory ------- #
            rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, current_pos_callback)
            rospy.sleep(0.1)
            current_pos = current_pos_msg.pose

            # ----------------------------- Choose the action ----------------------------- #
            if action == 0:
                # Entrance to lane A.
                wp1 = Point(4.5, 3, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            if action == 1:
                wp1 = Point(4.5, 6, z_nom - 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 2:
                wp1 = Point(4.5, 15, z_nom - 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 3:
                wp1 = Point(4.5, 24, z_nom - 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 4:
                wp1 = Point(9, 6, z_nom - 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 5:
                wp1 = Point(9, 15, z_nom - 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 6:
                wp1 = Point(9, 24, z_nom - 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 7:
                wp1 = Point(4.5, 6, z_nom + 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 8:
                wp1 = Point(4.5, 15, z_nom + 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 9:
                wp1 = Point(4.5, 24, z_nom + 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 10:
                wp1 = Point(9, 6, z_nom + 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 11:
                wp1 = Point(9, 15, z_nom + 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 12:
                wp1 = Point(9, 24, z_nom + 1)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 13:
                wp1 = Point(4, 0, 1.8)
                wp2 = Point(0.5, 0, 1.2)
                x_traj = [current_pos.position.x, wp1.x - xOffset, wp2.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset, wp2.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          , wp2.z          ]

            elif action == 14:
                wp1 = Point(9, 1, 1.5)
                wp2 = Point(11.8, 0.55, 0.1)
                x_traj = [current_pos.position.x, wp1.x - xOffset, wp2.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset, wp2.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          , wp2.z          ]

            elif action == 15:
                # Entrance to lane B.
                wp1 = Point(9, 3, z_nom)
                x_traj = [current_pos.position.x, wp1.x - xOffset]
                y_traj = [current_pos.position.y, wp1.y - yOffset]
                z_traj = [current_pos.position.z, wp1.z          ]

            elif action == 16:
                # This action maintains current position
                x_traj = [current_pos.position.x, current_pos.position.x]
                y_traj = [current_pos.position.y, current_pos.position.y]
                z_traj = [current_pos.position.z, current_pos.position.z]

            # elif action == 17:
                

            # elif action == 18:
            #     # This action increases z by 1
            #     x_traj = [current_pos.position.x, current_pos.position.x]
            #     y_traj = [current_pos.position.y, current_pos.position.y]
            #     z_traj = [current_pos.position.z, current_pos.position.z + 1]
            #     z_nom = z_nom + 1

            # elif action == 19:
            #     # This action decreases z by 1
            #     x_traj = [current_pos.position.x, current_pos.position.x]
            #     y_traj = [current_pos.position.y, current_pos.position.y]
            #     z_traj = [current_pos.position.z, current_pos.position.z - 1]
            #     z_nom = z_nom - 1

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
            trajCompleted = ShieldBool()
            trajCompleted.shield_bool = [True]
            send_trajComplete_pub.publish(trajCompleted)

            del globals()['shield_action_orders']
            del globals()['current_pos_msg']

        else:
            pass


