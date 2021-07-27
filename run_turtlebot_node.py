#!/usr/bin/env python3

"""
Script combining functionality of all publisher and subscriber
nodes for the turtlebot into one interface
"""

import pid_control
from numpy.lib.function_base import diff
import rospy
import actionlib

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from math import radians, copysign, sqrt, pow, pi, atan2
from actionlib_msgs.msg import *
from sound_play.libsoundplay import SoundClient
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import subprocess
import os
import time
import math



import battery_status
import odometry_status
import message_creator
import laser_data
from battery_status import battery
from odometry_status import callback
from message_creator import create_message
from laser_data import get_laser_data


# just setting an initial value for these; otherwise an error gets thrown
# might be able to just toss these later? depends on how script changes
posX = 0
posY = 0
batteryLevel = 100


class turtlebot(object):
    
    def __init__(self):
        # create a node name run_turtlebot_node
        rospy.init_node('run_turtlebot_node', anonymous=True)


        rospy.Subscriber('/scan', LaserScan, laser_data.get_laser_data)


        # create a subscriber for battery level
        rospy.Subscriber('battery_state', BatteryState, battery_status.battery)


        #TODO what's this line do?
        rospy.on_shutdown(self.shutdown)

        #create a publisher node to send velocity commands to turtlebot
        self.command_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # create a position of type Point
        self.position = Point()
        
        # create an object to send commands to turtlebot of type Twist
        # allows us to send velocity and rotation commands
        self.move_command = Twist()

        # set update rate; i.e. how often we send commands (Hz)
        self.rate = rospy.Rate(10)

        # create transform listener to transform coords from turtlebot frame to absolute frame
        self.listener = tf.TransformListener()

        self.odom_frame = '/odom'

        try:
            self.listener.waitForTransform(self.odom_frame, "base_footprint", rospy.Time(), rospy.Duration(1.0))
            self.base_frame = "base_footprint"
        except(tf.Exception, tf.LookupException, tf.ConnectivityException):
            try:
                self.listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except(tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        #TODO these are hardcoded for testing, but later set this up as an input
        self.p_gain = .1
        self.d_gain = 0.05
        self.i_gain = 0.03
        self.i_min = 0.0001
        self.i_max = 0.1

        self.controller = pid_control.pid_controller(self.p_gain, self.d_gain, self.i_gain, self.i_min, self.i_max)

        (self.position, self.rotation) = self.get_odom()
        self.first_turn = True


    def get_odom(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
            
        
        except(tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("tf Exception")
            return
        
        return (Point(*trans), rotation[2])
    

    def shutdown(self):
        self.command_publisher.publish(Twist())
        rospy.sleep(1)


    def send_messages(self):
        # publish the twist object to the publisher node we created before on the 'cmd_vel' topic
        self.command_publisher.publish(self.twist)

        # create a message and output it to the console
        message_creator.create_message(self.twist, battery_status.batteryLevel, 
        odometry_status.posX, odometry_status.posY, odometry_status.orient_x, 
        odometry_status.orient_y, odometry_status.orient_z, odometry_status.orient_w)
    

    def move_forward(self, goal_x, goal_y):
        (self.position, self.rotation) = self.get_odom()
        current_distance = sqrt(self.position.x**2 + self.position.y**2)
        goal_distance = sqrt(goal_x**2+goal_y**2)
        
        while goal_distance - current_distance > 0.005:
            command = self.controller.get_command(current_distance, goal_distance)
            self.move_command.linear.x = command
            self.move_command.angular.z = 0
            self.command_publisher.publish(self.move_command)
            (self.position, self.rotation) = self.get_odom()
            current_distance = sqrt(self.position.x**2 + self.position.y**2)
            print(self.position.x)
    
    def rotate_turtlebot(self, goal_x, goal_y):
        

        (self.position, self.rotation) = self.get_odom()
        path_angle = atan2(goal_y - self.position.y, goal_x - self.position.x)
        goal_rotation = path_angle

        while abs(goal_rotation - self.rotation) > 0.01:
            
            command = self.controller.get_command(self.rotation, goal_rotation)
            self.move_command.linear.x = 0
            self.move_command.angular.z = command
            self.command_publisher.publish(self.move_command)
            (self.position, self.rotation) = self.get_odom()
            print(command)
            rospy.sleep(0.1)
            # print("HELLO WORLD")
            # print(self.controller.command)
    






# function that sends velocity commands to turtlebot, calls other functions. Run by main function (see below)
def run_turtlebot():
    while not rospy.is_shutdown():
        turtle = turtlebot()
        if turtle.first_turn:
            turtle.rotate_turtlebot(0,1)
            turtle.rotate_turtlebot(1,0)
            turtle.move_forward(0.01, 0)
        turtle.first_turn = False
        print("Loop")


# main function; executes the run_turtlebot function until we hit control + C
if __name__ == '__main__':
    try:
        run_turtlebot()
    except rospy.ROSInterruptException:
        pass
