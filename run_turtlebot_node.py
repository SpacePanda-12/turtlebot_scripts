#!/usr/bin/env python3

"""
Script combining functionality of all publisher and subscriber
nodes for the turtlebot into one interface
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu


import battery_status
import odometry_status
import message_creator
import orientation
from battery_status import battery
from odometry_status import callback
from message_creator import create_message
from orientation import get_orientation

# just setting an initial value for these; otherwise an error gets thrown
posX = 0
posY = 0
batteryLevel = 100

# function that sends velocity commands to turtlebot, calls other functions. Run by main function (see below)
def run_turtlebot():
    # create a node name run_turtlebot_node
    rospy.init_node('run_turtlebot_node', anonymous=True)

    #create a publisher node to send velocity commands to turtlebot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # set update rate; i.e. how often we send commands (Hz)
    rate = rospy.Rate(1)

    
    while not rospy.is_shutdown():
        # create an object called twist of type Twist.
        # Twist is the data type that turtlebot odometry uses
        twist = Twist()
        # set linear velocities
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        #set angular velocities
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0.3

        # publish the twist object to the publisher node we created before on the 'cmd_vel' topic
        pub.publish(twist)

        # create a subscriber for odometry to get position
        rospy.Subscriber('odom', Odometry, odometry_status.callback)

        # create a subscriber for battery level
        from battery_status import battery
        rospy.Subscriber('battery_state', BatteryState, battery_status.battery)

        # create a subscriber for IMU orientation (maybe acceleration too)
        rospy.Subscriber('imu', Imu, orientation.get_orientation)

        # create a message and output it to the console
        message_creator.create_message(twist, battery_status.batteryLevel, odometry_status.posX, odometry_status.posY, orientation.orient_x, orientation.orient_y, orientation.orient_z, orientation.orient_w)

        # option to make the turtlebot pause for the rate defined above
        # rate.sleep()

# main function; executes the run_turtlebot function until we hit control + C
if __name__ == '__main__':
    try:
        run_turtlebot()
    except rospy.ROSInterruptException:
        pass
