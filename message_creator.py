#!/usr/bin/env

import rospy

# simple if else for low battery warning
def create_message(twist, batteryLevel, posX, posY, orient_x, orient_y, orient_z, orient_w):
    if(batteryLevel>=10):
        msg = """
        Sending commands to turtlebot...
        --------------------------------
        Linear velocities
        X: %f
        Y: %f
        Angular Velocity (CCW): %f
        --------------------------------
        Odometry

        Position:
        X: %f
        Y: %f

        Orientation:
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
    rospy.loginfo(msg, round(twist.linear.x, 2), round(twist.linear.y, 2), round(twist.angular.z, 2), round(posX, 2), round(posY, 2),
    round(orient_x, 2), round(orient_y, 2), round(orient_z, 2), round(orient_w, 2), batteryLevel)
    

    
