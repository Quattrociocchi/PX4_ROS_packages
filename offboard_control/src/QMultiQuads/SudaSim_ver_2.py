#!/usr/bin/env python

import sys
sys.path.insert( 0 , '~/catkin_ws/src/offboard_control/src')


import rospy
import csv
from qcontrol_defs.msg import *
from utils_functions import *
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import PoseStamped

class Quad:

    def __init__(self, quad_ros_namespace, x, y, z):

        self.ns = '/'+quad_ros_namespace
        self.traj = []
        self.wait_rate = rospy.Rate(20)

        # initial position
        self.x0 = x
        self.y0 = y
        self.z0 = z

        # initial posiiton
        # self.init_pub = self.Publisher(self.ns + '/mavros/home_position/set', HomePosition, queue_size=10)
        self.init_pub = rospy.Publisher(self.ns + '/mavros/home_position/set', HomePosition, queue_size=10)
        self.home = HomePosition()
        
        # for target positions
        self.target_pos_pub = rospy.Publisher(self.ns+'/qcontrol/pva_control', PVA, queue_size=10)

        # current position
        self.x = x
        self.y = y
        self.z = z



    def generate_traj(self, filename, time, freq):
        
        traj_dummy = []

        # add initialize position to traj
        init_pos = Point(self.x,self.y,self.z)
       
        x_traj = []
        # x_traj.append(init_pos.x)
        x_traj.append(0)
        
        y_traj = []
        # y_traj.append(init_pos.y)
        y_traj.append(0)
        
        z_traj = []
        # z_traj.append(init_pos.z)
        z_traj.append(2)

        with open('/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/'+filename, 'rb') as csvfile:
            totalstate = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in totalstate:
                 traj_dummy.append(map(int,row))


        for s in traj_dummy:
            temp = Point(s[0] - init_pos.x , s[1] - init_pos.y , 2)
            x_traj.append(temp.x)
            y_traj.append(temp.y)
            z_traj.append(temp.z)
    
        self.traj = generate_traj_3d(x=x_traj , y=y_traj , z=z_traj , traj_time=[0,time] , corr=None , freq = freq)

    
    def curr_pos_callback(self, msg):

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

    def set_pos_local(self):

        for c in range(0,10000):
            current_pos_sub = rospy.Subscriber(self.ns+'/mavros/local_position/pose', PoseStamped, self.curr_pos_cb)

    def set_pos_home(self):

        self.home.position.x = self.x0
        self.home.position.y = self.y0
        self.home.position.z = self.z0
        self.x = self.x0
        self.y = self.y0
        self.z = self.z0
        # Publish the message to set the home position with mavros. This must be done for some length of time in order for it to make it through.
        for c in range(0,10000):
            self.init_pub.publish(self.home)
            self.wait_rate.sleep()


    # def pub_traj(self, freq):
    #     # wait_rate = rospy.Rate(freq)

    #     # Start position control with taking off at the beginning
    #     start_pva_control(quad_name=self.ns , takeoff_before=True)
    #     for i in range(len(self.traj.pva)):
    #         self.target_pos_pub.publish(self.traj.pva[i])
    #         self.wait_rate.sleep()

    #     # var1 = input('Set Quads to Home Position? 0 or 1')
    #     # var2 = input('Set Quads to Local Position? 0 or 1')

    #     # if var1:
    #     #     self.set_pos_home()
    #     # elif var2:
    #     #     self.set_pos_local()
    #     # else:
    #     #     pass

    #     start_landing(quad_name=self.ns)


    def pub_traj(self, ii):
        self.target_pos_pub.publish(self.traj.pva[ii])


if __name__ == "__main__":

    run_again = 1
    while run_again:
        # Initialize node.
        rospy.init_node('Quad_offboard_pvacontrol')

        #### TIME SET
        # Set time stuff
        time_full_traj = 200 #seconds
        freq = 20
        wait_rate = rospy.Rate(freq)


        # Should be the prefix added to the offboard control node (prefix of all mavros topics)
        # for example "" or "/Quad9" or "/Quad10" (or "/uav1" for px4)
        quad_ros_namespace1 = "uav1"
        quad_ros_namespace2 = "uav2"
        quad_ros_namespace3 = "uav3"
        quad_ros_namespace4 = "uav4"

        uav1 = Quad(quad_ros_namespace1, 7, 16, 2)
        uav2 = Quad(quad_ros_namespace2, 9, 5, 2)
        uav3 = Quad(quad_ros_namespace3, 2, 9, 2)
        uav4 = Quad(quad_ros_namespace4, 10, 17, 2)


        # var1 = input('Set Quads to Home Position? 0 or 1')
        # var2 = input('Set Quads to Local Position? 0 or 1')

        # if var1:
        #     uav1.set_pos_home()
        #     uav2.set_pos_home()
        #     uav3.set_pos_home()
        #     uav4.set_pos_home()
        # elif var2:
        #     uav1.set_pos_local()
        #     uav2.set_pos_local()
        #     uav3.set_pos_local()
        #     uav4.set_pos_local()
      

        uav1.generate_traj('statehistory_target.txt', time_full_traj, freq)
        uav2.generate_traj('statehistory1.txt', time_full_traj, freq)
        uav3.generate_traj('statehistory2.txt', time_full_traj, freq)
        uav4.generate_traj('statehistory3.txt', time_full_traj, freq)

        # uav1.pub_traj(freq)
        # uav2.pub_traj(freq)
        # uav3.pub_traj(freq)
        # uav4.pub_traj(freq)

        # Start position control with taking off at the beginning
        start_pva_control(quad_name=quad_ros_namespace1 , takeoff_before=True)
        start_pva_control(quad_name=quad_ros_namespace2 , takeoff_before=True)
        start_pva_control(quad_name=quad_ros_namespace3 , takeoff_before=True)
        start_pva_control(quad_name=quad_ros_namespace4 , takeoff_before=True)

        # Send the generated traj to the vehicle
        for i in range(len(uav1.traj.pva)) :
            uav1.pub_traj(i)
            uav2.pub_traj(i)
            uav3.pub_traj(i)
            uav4.pub_traj(i)
            wait_rate.sleep()

        # Land the vehicle
        start_landing(quad_name= quad_ros_namespace1)
        start_landing(quad_name= quad_ros_namespace2)
        start_landing(quad_name= quad_ros_namespace3)
        start_landing(quad_name= quad_ros_namespace4)

        run_again = input('run again? 0 or 1')






        


















