import rospy
import pid_control
import os
import tf
from math import pi, sqrt
import time

import odometry_status
from send_command import execute_command

class turtlebot_command(object):

    def __init__(self, command_publisher, rate, listener, position, move_command, odom_frame, base_frame):
        """
        
        Initializes control script that uses PID control to move the turtlebot.

        When substituting in a new control script, it must be structured like this
        script; i.e. it must be contained within a class and have an init function with 
        the same arguments. These arguments are not used in this script directly, but are 
        passed to other external functions while commands are being run. 
        
        """
        
        self.rate = rate
        self.command_publisher = command_publisher
        self.listener = listener
        # TODO is it necessary to define position here and have it as an input argument?
        self.position = position
        self.move_command = move_command
        self.odom_frame = odom_frame
        self.base_frame = base_frame

        # Warning: Don't modify these gains. They are tailored to the
        # turtlebots
        self.p_gain_linear = 0.05
        self.d_gain_linear = 0.02
        self.i_gain_linear = 0.01

        self.p_gain_angular = .01
        self.d_gain_angular = 0.005
        self.i_gain_angular = 0.003

        # create a PID controller object for translational movement
        self.linear_controller = pid_control.pid_controller(
            self.p_gain_linear, self.d_gain_linear, self.i_gain_linear
        )
        
        # create a PID controller object for angular movement
        self.angular_controller = pid_control.pid_controller(
            self.p_gain_angular, self.d_gain_angular, self.i_gain_angular
        )


    def rotate_turtlebot(self, goal_rotation):
        """

        Rotates the turtlebot towards the goal position
        
        """
        # reset odometry and timeout after 3 seconds 
        # IMPORTANT: If running on odometry only, this must be run before each command
        os.system("timeout 3 rostopic pub /reset std_msgs/Empty '{}'")

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

        # set max rotational command
        max_command = 1
        start_time = time.time()

        while abs(goal_rotation - self.rotation) > 0.25*2*pi/360:
            
            # update position and rotation measurements
            (self.position, self.rotation) = odometry_status.get_odom(self.listener, self.odom_frame, self.base_frame)

            # break the loop if it takes more than 10 seconds
            current_time = time.time()
            if (current_time - start_time) > 15:
                break

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

            # send command to turtlebot and output message to the console
            # IMPORTANT: When substituting a new control script in, this function
            # must be called in order to actually send the command to the turtlebot
            execute_command(self.command_publisher, self.move_command, 0, command, self.position, self.rotation)
        
        # pause to avoid commands starting before the last one has ended
        rospy.sleep(1)
        print("Rotation complete")


    def move_turtlebot(self, goal_distance):
        """

        Moves the turtlebot in a straight line to the goal position

        """

        # reset odometry and timeout after 3 seconds
        # IMPORTANT: If running on odometry only, this must be run before each command
        os.system("timeout 3 rostopic pub /reset std_msgs/Empty '{}'")

        # set max translational movement command
        max_command = 0.2

        # update position and rotation readings
        (self.position, self.rotation) = odometry_status.get_odom(self.listener, self.odom_frame, self.base_frame)

        # check which direction turtlebot needs to move
        if goal_distance > 0:
            current_distance = sqrt((self.position.x)**2 + (self.position.y)**2)
        else:
            current_distance = -sqrt((self.position.x)**2 + (self.position.y)**2)


        while abs(goal_distance - current_distance) > .01*abs(goal_distance):

            # update position and rotation
            (self.position, self.rotation) = odometry_status.get_odom(self.listener, self.odom_frame, self.base_frame)
            
            command = self.linear_controller.get_command(current_distance, goal_distance)
            
            if command > max_command:
                command = max_command
            elif command < -max_command:
                command = max_command

            # send command to turtlebot and output message to the console
            # IMPORTANT: When substituting a new control script in, this function
            # must be called in order to actually send the command to the turtlebot
            execute_command(self.command_publisher, self.move_command, command, 0, self.position, self.rotation)

            if goal_distance > 0:
                current_distance = sqrt((self.position.x)**2 + (self.position.y)**2)
            else:
                current_distance = -sqrt((self.position.x)**2 + (self.position.y)**2)

        # pause to avoid commands starting before the last one has ended
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
