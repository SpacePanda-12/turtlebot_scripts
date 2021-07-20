#!/usr/bin/env

import rospy

# simple if else for low battery warning
def create_message(twist, batteryLevel, posX, posY, orient_x, orient_y, orient_z, orient_w):
    if(batteryLevel>=20):
        msg = """
        Sending commands to turtlebot...
        --------------------------------
        Linear velocities
        X: %f
        Y: %f
        Angular Velocity: %f
        --------------------------------
        Odometry
        X Position: %f
        Y Position: %f
        --------------------------------
        Orientation (IMU)
        x: %f
        y: %f
        z: %f
        w: %f
        --------------------------------
        Estimated battery level: %f/100
        --------------------------------
        
        --------------------------------

        """
    else:
        msg = """
        Sending commands to turtlebot...
        --------------------------------
        Linear velocities
        X: %f
        Y: %f
        Angular Velocity: %f
        --------------------------------
        Odometry
        X Position: %f
        Y Position: %f
        --------------------------------
        Orientation (IMU)
        x: %f
        y: %f
        z: %f
        w: %f
        --------------------------------
        Estimated battery level: %f/100
        BATTERY LEVEL LOW. CHARGE SOON
        --------------------------------
        





        --------------------------------

        """

    # write the above message to the console with velocity, position, battery level values
    rospy.loginfo(msg, twist.linear.x, twist.linear.y, twist.angular.z, posX, posY,
    orient_x, orient_y, orient_z, orient_w, batteryLevel)
    

    
