from genpy import message
from tf import listener
import rospy
import pid_control
import os
import tf
from math import pi, sqrt
import time
from geometry_msgs.msg import Twist, Point

import message_creator
import odometry_status
import battery_status

class turtlebot_command(object):

    def __init__(self, command_publisher, rate):
        self.rate = rate
        self.command_publisher = command_publisher

        # create transform listener to transform coords from turtlebot frame to absolute frame
        self.listener = tf.TransformListener()

        # create a position of type Point
        self.position = Point()
        
        # create an object to send commands to turtlebot of type Twist
        # allows us to send velocity and rotation commands
        self.move_command = Twist()

        # set odom_frame to the /odom topic being published by the turtlebot
        self.odom_frame = '/odom'

        # Warning: Don't modify these gains. They are tailored to the
        # turtlebots
        self.p_gain_linear = 0.05
        self.d_gain_linear = 0.02
        self.i_gain_linear = 0.01

        self.p_gain_angular = .01
        self.d_gain_angular = 0.005
        self.i_gain_angular = 0.003

        self.linear_controller = pid_control.pid_controller(
            self.p_gain_linear, self.d_gain_linear, self.i_gain_linear
        )
        
        self.angular_controller = pid_control.pid_controller(
            self.p_gain_angular, self.d_gain_angular, self.i_gain_angular
        )



    def rotate_turtlebot(self, goal_rotation):
        """

        Rotates the turtlebot towards the goal position
        
        """
        # reset odometry and timeout after 3 seconds 
        os.system("timeout 3 rostopic pub /reset std_msgs/Empty '{}'")

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
        (self.position, self.rotation) = odometry_status.get_odom(self.listener, self.odom_frame, self.base_frame)

        max_command = 1
        start_time = time.time()

        while abs(goal_rotation - (self.rotation)) > 0.25*2*pi/360:

            # break the loop if it takes more than 10 seconds
            current_time = time.time()
            if (current_time - start_time) > 15:
                break

            (self.position, self.rotation) = odometry_status.get_odom(self.listener, self.odom_frame, self.base_frame)

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

            # publish turtlebot status to console
            message_creator.create_message(
                self.move_command, battery_status.batteryLevel, self.position, self.rotation*360/(2*pi)
            )

            # sleep to avoid overloading turtlebot with commands
            rospy.sleep(0.01)
        
        rospy.sleep(1)
        print("Rotation complete")


    def move_turtlebot(self, goal_distance):
        """

        Moves the turtlebot in a straight line to the goal position

        """

        # reset odometry and timeout after 3 seconds 
        os.system("timeout 3 rostopic pub /reset std_msgs/Empty '{}'")

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
        
        max_command = 0.2

        (self.position, self.rotation) = odometry_status.get_odom(self.listener, self.odom_frame, self.base_frame)

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
            (self.position, self.rotation) = odometry_status.get_odom(self.listener, self.odom_frame, self.base_frame)

            if goal_distance > 0:
                current_distance = sqrt((self.position.x)**2 + (self.position.y)**2)
            else:
                current_distance = -sqrt((self.position.x)**2 + (self.position.y)**2)

            # publish turtlebot status to console
            
            message_creator.create_message(
                self.move_command, battery_status.batteryLevel, self.position, self.rotation*360/(2*pi)
            )

            rospy.sleep(0.01)

        
        rospy.sleep(1)
        print("Movement complete")


    def move_in_a_square(turtle):
            turtle.move_turtlebot(0.5)
            turtle.rotate_turtlebot(90)
            turtle.move_turtlebot(0.5)
            turtle.rotate_turtlebot(90)
            turtle.move_turtlebot(0.5)
            turtle.rotate_turtlebot(90)
            turtle.move_turtlebot(0.5)
            turtle.rotate_turtlebot(90)
