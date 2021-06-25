#!/usr/bin/env

# initial values to avoid errors
posX = 0
posY = 0

# function to obtain position of turtlebot
def callback(data):
    # create global variables for the x and y coords of the turtlebot
    global posX
    global posY
    
    # get X and Y coordinates from turtlebot
    posX = data.pose.pose.position.x
    posY = data.pose.pose.position.y
