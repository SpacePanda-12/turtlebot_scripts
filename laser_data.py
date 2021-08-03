#! /usr/bin/env python
distance_to_object = 0.0
distance_to_object_right = 0.0
distance_to_object_left = 0.0

def get_laser_data(msg):
    global distance_to_object
    global distance_to_object_left
    global distance_to_object_right
    distance_to_object = msg.ranges[0]
    distance_to_object_left = msg.ranges[90]
    distance_to_object_right = msg.ranges[270]
