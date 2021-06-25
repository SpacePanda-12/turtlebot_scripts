#!/usr/bin/env

# setting initial values to avoid errors
orient_x = 0
orient_y = 0
orient_z = 0
orient_w = 0

# retrieve orientation values from the turtlebot

# note to self: try to find a way to convert these 
# orientations to something meaningful. They look
# like unit vectors, but not sure which way
# they are pointing. seems likely that the vectors
# are defined when the turtlebot is turned on.

def get_orientation(data):
    global orient_x
    global orient_y
    global orient_z
    global orient_w

    orient_x = data.orientation.x
    orient_y = data.orientation.y
    orient_z = data.orientation.z
    orient_w = data.orientation.w
