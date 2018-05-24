#!/usr/bin/env python

import rospy
from qcontrol_defs.msg import *

# ------------------------------------------------------------------ #

def shield_bool_callback(msg):
    shield_bool = msg

    # Check for contradictions in the message received
    if shield_bool.positiveX == True and shield_bool.negativeX == True:
        print('Error. The shield has ordered a positive and negative movement in X.')
        # Kill the rest of the process somehow ?????
    if shield_bool.positiveY == True and shield_bool.negativeY == True:
        print('Error. The shield has ordered a positive and negative movement in Y.')
    if shield_bool.positiveZ == True and shield_bool.negativeZ == True:
        print('Error. The shield has ordered a positive and negative movement in Z.')

    # Initialize x y z vairables
    x = 0
    y = 0
    z = 0

    # Search through message from the shield to determine the direction of movement
    if shield_bool.positiveX == True:
        x = x + 1
    if shield_bool.negativeX == True:
        x = x - 1
    if shield_bool.positiveY == True:
        y = y + 1
    if shield_bool.negativeY == True:
        y = y - 1
    if shield_bool.positiveZ == True:
        z = z + 1
    if shield_bool.negativeZ == True:
        z = z - 1

    shield_orders_temp = ShieldOutput()
    shield_orders_temp.posChange.append(x)
    shield_orders_temp.posChange.append(y)
    shield_orders_temp.posChange.append(z)

    global shield_orders
    shield_orders = shield_orders_temp

# ------------------------------------------------------------------ #

if __name__ == "__main__":
    # Initialize node.
    rospy.init_node('ShieldOrders', anonymous=True)
    
    # Get the appropriate quad name
    quad_ros_namespace = rospy.get_param('~quad_ros_namespace')

    # Create a publisher to send target positions
    send_shield_pub = rospy.Publisher(quad_ros_namespace + '/shield_orders_pos', ShieldOutput , queue_size=10)

    while not rospy.is_shutdown():

        rospy.Subscriber(quad_ros_namespace + '/shield_bool_list',ShieldInput , shield_bool_callback)

        # print('Listening for ShieldInput')

        if 'shield_orders' in globals():
            # Publish the resulting change in position
            send_shield_pub.publish(shield_orders)
        else:
            pass