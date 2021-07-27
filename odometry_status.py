#!/usr/bin/env

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

# initial values to avoid errors
posX = 0
posY = 0
orient_x = 0
orient_y = 0
orient_z = 0
orient_w = 0
roll = 0
pitch = 0
yaw = 0


# function to obtain position of turtlebot
def callback(data):
    # create global variables for the x and y coords and orientation of the turtlebot
    global posX, posY, orient_x, orient_y, orient_z, orient_w, roll, pitch, yaw

    # get X and Y coordinates from turtlebot as well as orientation
    posX = data.pose.pose.position.x
    posY = data.pose.pose.position.y
    orient_x = data.pose.pose.orientation.x
    orient_y = data.pose.pose.orientation.y
    orient_z = data.pose.pose.orientation.z
    orient_w = data.pose.pose.orientation.w
    # orientation_q = data.pose.pose.orientation
    orientation_list = [orient_x, orient_y, orient_z, orient_w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
