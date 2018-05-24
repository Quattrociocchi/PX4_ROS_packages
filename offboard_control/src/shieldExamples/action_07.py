#!/usr/bin/env python


import rospy
import sys
sys.path.insert( 0 , '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src')
# import csv
from qcontrol_defs.msg import *
from utils_functions import *
from geometry_msgs.msg import PoseStamped

# ------------------------------------------------------------------ #

def shield_bool_callback(msg):
    global shield_bool_orders
    shield_bool_orders = msg.shield_bool

def current_pos_callback(msg):
    global current_pos_msg
    current_pos_msg = msg

# ------------------------------------------------------------------ #

if __name__ == "__main__":
    # Initialize node.
    rospy.init_node('action_05', anonymous=True)
    
    # Get the appropriate quad name
    quad_ros_namespace = rospy.get_param('~quad_ros_namespace')
    xOffset = rospy.get_param('~xOffset')
    yOffset = rospy.get_param('~yOffset')
    freq = rospy.get_param('~freq')
    time_full_traj = rospy.get_param('~time_full_traj')
    wait_rate = rospy.Rate(freq)

    # Create a publisher to send target positions
    send_pva_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pva_control', PVA , queue_size=10)
    send_trajComplete_pub = rospy.Publisher(quad_ros_namespace + '/trajComplete', ShieldBool, queue_size=10)

    while not rospy.is_shutdown():
        rospy.Subscriber(quad_ros_namespace + '/shield_bool_orders', ShieldBool , shield_bool_callback)

        if 'shield_bool_orders' in globals():
            shield_orders = shield_bool_orders
            if shield_orders[0] == True:
                # ------- Fly the predetermined trajectory ------- #
                rospy.Subscriber(quad_ros_namespace + '/mavros/local_position/pose', PoseStamped, current_pos_callback)
                rospy.sleep(0.1)
                current_pos = current_pos_msg.pose

                # Set the waypoints
                z_nom = 2
                Target3 = Point(6, 0, z_nom)

                # Build the trajectory
                x_traj = [current_pos.position.x, Target3.x - xOffset]
                y_traj = [current_pos.position.y, Target3.y - yOffset]
                z_traj = [current_pos.position.z, Target3.z]

                # Find the complete minimum snap trajectory
                pva_list = generate_traj_3d(x=x_traj , y=y_traj , z=z_traj , traj_time=[0,time_full_traj] , corr=None , freq = freq)

                # Send the generate traj to the vehicle
                for elem in pva_list.pva:
                    send_pva_pub.publish(elem)
                    wait_rate.sleep()

                # Notify that trajectory is complete
                trajCompleted = ShieldBool()
                trajCompleted.shield_bool = [True]
                send_trajComplete_pub.publish(trajCompleted)
            else:
                pass

            del globals()['shield_bool_orders']
            del globals()['current_pos_msg']

        else:
            pass















# # Publish the resulting change in position
# print('Sleeping for 0.5 seconds')
# rospy.sleep(0.5)