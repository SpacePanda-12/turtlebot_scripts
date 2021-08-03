#!/usr/bin/env python3

"""
Script combining functionality of all publisher and subscriber
nodes for the turtlebot into one interface
"""

# import python and ros libraries
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import BatteryState, LaserScan
from math import sqrt, pi
from actionlib_msgs.msg import *
from sound_play.libsoundplay import SoundClient
import tf
from tf.transformations import euler_from_quaternion
import os
import time

# import other realm-specific scripts
import pid_control
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

        # set update rate; i.e. how often we send commands (Hz)
        self.rate = rospy.Rate(10)

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
        self.p_gain_linear = 0.05
        self.d_gain_linear = 0.02
        self.i_gain_linear = 0.01

        self.p_gain_angular = .01
        self.d_gain_angular = 0.005
        self.i_gain_angular = 0.003

        self.i_min = 0.0001
        self.i_max = 2

        self.linear_controller = pid_control.pid_controller(
            self.p_gain_linear, self.d_gain_linear, self.i_gain_linear, self.i_min, self.i_max
            )
        
        self.angular_controller = pid_control.pid_controller(
            self.p_gain_angular, self.d_gain_angular, self.i_gain_angular, self.i_min, self.i_max
        )

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
    

    def move_turtlebot(self, goal_distance):
        """

        Moves the turtlebot in a straight line to the goal position

        """

        # reset odometry and timeout after 3 seconds 
        os.system("timeout 1.5 rostopic pub /reset std_msgs/Empty '{}'")
        
        max_command = 0.2

        (self.position, _) = self.get_odom()

        if goal_distance > 0:
            current_distance = sqrt((self.position.x)**2 + (self.position.y)**2)
        else:
            current_distance = -sqrt((self.position.x)**2 + (self.position.y)**2)


        while abs(goal_distance - current_distance) > .01*abs(goal_distance):
            
            command = self.linear_controller.get_command(current_distance, goal_distance)
            
            if command > max_command:
                command = max_command
            elif command < -max_command:
                command = max_command

            self.move_command.linear.x = command
            self.move_command.angular.z = 0

            self.command_publisher.publish(self.move_command)
            (self.position, _) = self.get_odom()

            if goal_distance > 0:
                current_distance = sqrt((self.position.x)**2 + (self.position.y)**2)
            else:
                current_distance = -sqrt((self.position.x)**2 + (self.position.y)**2)

            rospy.sleep(0.01)


    def rotate_turtlebot(self, goal_rotation):
        """

        Rotates the turtlebot towards the goal position
        
        """
        # reset odometry and timeout after 3 seconds 
        os.system("timeout 1.5 rostopic pub /reset std_msgs/Empty '{}'")

        # if the angle is positive, make sure it's
        # between 0 and 360
        if goal_rotation > 360:
            while goal_rotation > 360:
                goal_rotation -= 360
        
        # if the angle is between 180 and 360, then change
        # the angle to be equivalent but negative to reduce
        # turning time, i.e. change a 270 degree spin to a
        # 90 degree spin
        if 180 < goal_rotation <= 360:
            goal_rotation -= 360
        
        # if the angle is negative, make sure it's between
        # -360 and 0
        if goal_rotation < -360:
            while goal_rotation < -360:
                goal_rotation += 360
        
        # if the angle is between -180 and -360, then change the angle 
        # to be equivalent but positive to reduce turning time
        # i.e. change a -270 degree spin to a 90 degree spin
        if -360 <= goal_rotation < -180:
            goal_rotation += 360

        # convert to radians
        goal_rotation = goal_rotation/360*2*pi

        # get current rotation
        (_, self.rotation) = self.get_odom()


        max_command = 1
        start_time = time.time()

        while abs(goal_rotation - (self.rotation)) > 0.25*2*pi/360:

            # break the loop if it takes more than 10 seconds
            current_time = time.time()
            if (current_time - start_time) > 15:
                break

            (_, self.rotation) = self.get_odom()

            # keep the rotation positive and within [0, 2*pi]
            if goal_rotation > 0:
                if self.rotation < -2*pi:
                    self.rotation += 4*pi
                elif self.rotation < 0:
                    self.rotation = -self.rotation
                elif self.rotation >= 2*pi:
                    self.rotation -= 2*pi

            # keep the rotation negative and within [-2*pi, 0]
            if goal_rotation < 0:
                if self.rotation  > 2*pi:
                    self.rotation -= 4*pi
                elif self.rotation  > 0:
                    self.rotation = -self.rotation
                elif self.rotation  <= -2*pi:
                    self.rotation += 2*pi
            
            # evaluate PID command based on goal and state
            command = self.angular_controller.get_command(self.rotation, goal_rotation)

            # set ceiling and floor for command so turtlebot doesn't turn too quickly
            if command > max_command:
                command = max_command
            elif command < -max_command:
                command  = -max_command

            # set and publish the linear and angular commands
            self.move_command.linear.x = 0
            self.move_command.angular.z = command
            self.command_publisher.publish(self.move_command)

            # sleep to avoid overloading turtlebot with commands
            rospy.sleep(0.01)


# function that sends velocity commands to turtlebot, calls other functions. Run by main function (see below)
def run_turtlebot():

    turtle = turtlebot()
    turtle2 = turtlebot()
    turtle3 = turtlebot()
    turtle4 = turtlebot()
    turtle5 = turtlebot()
    turtle6 = turtlebot()
    turtle7 = turtlebot()
    turtle8 = turtlebot()
    while not rospy.is_shutdown():
        if turtle.first_turn is True:
            turtle.move_turtlebot(1)
            print("Maneuver 1 completed")
            rospy.sleep(1)
            turtle2.rotate_turtlebot(90)
            print("Maneuver 2 completed")
            rospy.sleep(1)
            turtle3.move_turtlebot(1)
            print("Maneuver 3 completed")
            rospy.sleep(1)
            turtle4.rotate_turtlebot(90)
            print("Maneuver 4 completed")
            rospy.sleep(1)
            turtle5.move_turtlebot(1)
            print("Maneuver 5 completed")
            rospy.sleep(1)
            turtle6.rotate_turtlebot(90)
            print("Maneuver 6 completed")
            rospy.sleep(1)
            turtle7.move_turtlebot(1)
            print("Maneuver 7 completed")
            rospy.sleep(1)
            turtle8.rotate_turtlebot(90)

        turtle.first_turn = False
        


# main function; executes the run_turtlebot function until we hit control + C
if __name__ == '__main__':
    try:
        run_turtlebot()
    except rospy.ROSInterruptException:
        pass
