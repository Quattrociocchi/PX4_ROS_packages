#!/usr/bin/env python


import rospy
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
                # Set the waypoints for action 0
                Target0 = Point(0, 0, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target0.x - xOffset]
                y_traj = [current_pos.position.y, Target0.y - yOffset]
                z_traj = [current_pos.position.z, Target0.z]

            elif action == 1:
                # Set the waypoints for action 1
                Target0 = Point(0, 0, z_nom)
                TargetA = Point(3, 3, z_nom)
                TargetB = Point(3, 9, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, TargetB.x - xOffset, TargetA.x - xOffset, Target0.x - xOffset]
                y_traj = [current_pos.position.y, TargetB.y - yOffset, TargetA.y - yOffset, Target0.y - yOffset]
                z_traj = [current_pos.position.z, TargetB.z, TargetA.z, Target0.z]

            elif action == 2:
                # Set the waypoints for action 2
                Target0 = Point(0, 0, z_nom)
                Target1 = Point(0, 6, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target1.x - xOffset, Target0.x - xOffset]
                y_traj = [current_pos.position.y, Target1.y - yOffset, Target0.y - yOffset]
                z_traj = [current_pos.position.z, Target1.z, Target0.z]

            elif action == 3:
                # Set the waypoints for action 3
                Target1 = Point(0, 6, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target1.x - xOffset]
                y_traj = [current_pos.position.y, Target1.y - yOffset]
                z_traj = [current_pos.position.z, Target1.z]

            elif action == 4:
                # Set the waypoints for action 4
                Target2 = Point(0, 12, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target2.x - xOffset]
                y_traj = [current_pos.position.y, Target2.y - yOffset]
                z_traj = [current_pos.position.z, Target2.z]

            elif action == 5:
                # Set the waypoints for action 5
                Target2 = Point(0, 12, z_nom)
                TargetA = Point(3, 3, z_nom)
                TargetB = Point(3, 9, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, TargetA.x - xOffset, TargetB.x - xOffset, Target2.x - xOffset]
                y_traj = [current_pos.position.y, TargetA.y - yOffset, TargetB.y - yOffset, Target2.y - yOffset]
                z_traj = [current_pos.position.z, TargetA.z, TargetB.z, Target2.z]

            elif action == 6:
                # Set the waypoints for action 6
                Target2 = Point(0, 12, z_nom)
                Target1 = Point(0, 6, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target1.x - xOffset, Target2.x - xOffset]
                y_traj = [current_pos.position.y, Target1.y - yOffset, Target2.y - yOffset]
                z_traj = [current_pos.position.z, Target1.z, Target2.z]

            elif action == 7:
                # Set the waypoints for action 7
                Target3 = Point(6, 0, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target3.x - xOffset]
                y_traj = [current_pos.position.y, Target3.y - yOffset]
                z_traj = [current_pos.position.z, Target3.z]

            elif action == 8:
                # Set the waypoints for action 8
                Target3 = Point(6, 0, z_nom)
                TargetA = Point(3, 3, z_nom)
                TargetB = Point(3, 9, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, TargetB.x - xOffset, TargetA.x - xOffset, Target3.x - xOffset]
                y_traj = [current_pos.position.y, TargetB.y - yOffset, TargetA.y - yOffset, Target3.y - yOffset]
                z_traj = [current_pos.position.z, TargetB.z, TargetA.z, Target3.z]

            elif action == 9:
                # Set the waypoints for action 9
                Target3 = Point(6, 0, z_nom)
                Target4 = Point(6, 6, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target4.x - xOffset, Target3.x - xOffset]
                y_traj = [current_pos.position.y, Target4.y - yOffset, Target3.y - yOffset]
                z_traj = [current_pos.position.z, Target4.z, Target3.z]

            elif action == 10:
                # Set the waypoints for action 10
                Target4 = Point(6, 6, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target4.x - xOffset]
                y_traj = [current_pos.position.y, Target4.y - yOffset]
                z_traj = [current_pos.position.z, Target4.z]

            elif action == 11:
                # Set the waypoints for action 11
                Target5 = Point(6, 12, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target5.x - xOffset]
                y_traj = [current_pos.position.y, Target5.y - yOffset]
                z_traj = [current_pos.position.z, Target5.z]

            elif action == 12:
                # Set the waypoints for action 12
                Target5 = Point(6, 12, z_nom)
                TargetA = Point(3, 3, z_nom)
                TargetB = Point(3, 9, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, TargetA.x - xOffset, TargetB.x - xOffset, Target5.x - xOffset]
                y_traj = [current_pos.position.y, TargetA.y - yOffset, TargetB.y - yOffset, Target5.y - yOffset]
                z_traj = [current_pos.position.z, TargetA.z, TargetB.z, Target5.z]

            elif action == 13:
                # Set the waypoints for action 13
                Target4 = Point(6, 6, z_nom)
                Target5 = Point(6, 12, z_nom)
                # Build the trajectory
                x_traj = [current_pos.position.x, Target4.x - xOffset, Target5.x - xOffset]
                y_traj = [current_pos.position.y, Target4.y - yOffset, Target5.y - yOffset]
                z_traj = [current_pos.position.z, Target4.z, Target5.z]

            elif action == 14:
                # This action increases z by 1
                # Build the trajectory
                x_traj = [current_pos.position.x, current_pos.position.x]
                y_traj = [current_pos.position.y, current_pos.position.y]
                z_traj = [current_pos.position.z, current_pos.position.z + 1]
                z_nom = z_nom + 1

            elif action == 15:
                # This action decreases z by 1
                # Build the trajectory
                x_traj = [current_pos.position.x, current_pos.position.x]
                y_traj = [current_pos.position.y, current_pos.position.y]
                z_traj = [current_pos.position.z, current_pos.position.z - 1]
                z_nom = z_nom - 1

            else:
                print('The action choice you have made is not valid. Try again you crazy cat 8-D')

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


