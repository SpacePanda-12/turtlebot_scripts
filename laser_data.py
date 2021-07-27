#! /usr/bin/env python
distance_to_object = 0.0
distance_to_object_right = 0.0
distance_to_object_left = 0.0
xvelocity = 0
zangular = 0

def get_laser_data(msg):
    global xvelocity
    global zangular
    global distance_to_object
    global distance_to_object_left
    global distance_to_object_right
    distance_to_object = msg.ranges[0]
    distance_to_object_left = msg.ranges[90]
    distance_to_object_right = msg.ranges[270]

    if distance_to_object > 0.2 and distance_to_object_left > 0.2 and distance_to_object_right > 0.2:
        xvelocity = 0.1
        zangular = 0
    else:
        xvelocity = 0
        zangular = 3.141592654
    
