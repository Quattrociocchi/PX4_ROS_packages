#!/usr/bin/env python

import rospy
from qcontrol_defs.msg import *




# Header header
# bool positiveX
# bool negativeX
# bool positiveY
# bool negativeY
# bool positiveZ
# bool negativeZ




# ------------------------------------------------------------------ #

def shield_bool_callback(msg):
    global shield_bool_orders
    shield_bool_orders = msg



# ------------------------------------------------------------------ #

if __name__ == "__main__":
	# Get the appropriate quad name
    quad_ros_namespace = rospy.get_param('~quad_ros_namespace')
    # Get the shield bool 
    shield_bool = ShieldInput()
    rospy.Subscriber(quad_ros_namespace + '/shield_bool_list',ShieldInput , shield_bool_callback)
    shield_bool = shield_bool_orders

    # Check for contradictions in the message received
    if shield_bool.positiveX == True and shield_bool.negativeX == True:
    	print('Error. The shield has ordered a positive and negative movement in X.')
    	# Kill the rest of the process somehow ?????
    if shield_bool.positiveY == True and shield_bool.negativeY == True:
    	print('Error. The shield has ordered a positive and negative movement in Y.')
    if shield_bool.positiveZ == True and shield_bool.negativeZ == True:
    	print('Error. The shield has ordered a positive and negative movement in Z.')

	# Initialize node.
	rospy.init_node(quad_ros_namespace + '/ShieldOrders')

    # Create a publisher to send target positions
    send_shield_pub = rospy.Publisher(quad_ros_namespace + '/shield_orders_pos', ShieldOutput , queue_size=10)

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

    shield_orders = []
    shield_orders[0] = x
    shield_orders[1] = y
    shield_orders[2] = z

    # Publish the resulting change in position
    print('Sleeping for 0.5 seconds')
	rospy.sleep(0.5)
	send_shield_pub_1.publish(shield_orders)

