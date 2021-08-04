#!/usr/bin/env python3

"""
Script combining functionality of all publisher and subscriber
nodes for the turtlebot into one interface
"""

# import python and ros libraries
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import BatteryState, LaserScan
from math import sqrt, pi
from actionlib_msgs.msg import *
from sound_play.libsoundplay import SoundClient
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

# import other realm-specific scripts
import battery_status
import odometry_status
import message_creator
import laser_data
from battery_status import battery
from message_creator import create_message
from laser_data import get_laser_data
from turtlebot_functions import *


# just setting an initial value for these; otherwise an error gets thrown
# might be able to just toss these later? depends on how script changes
posX = 0
posY = 0
batteryLevel = 100


class turtlebot(object):
    
    def __init__(self):
        # create a node name run_turtlebot_node
        rospy.init_node('run_turtlebot_node', anonymous=True)

        # set update rate; i.e. how often we send commands (Hz)
        self.rate = rospy.Rate(10)

        #create a publisher node to send velocity commands to turtlebot
        self.command_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/scan', LaserScan, laser_data.get_laser_data)

        # create a subscriber for battery level
        rospy.Subscriber('battery_state', BatteryState, battery_status.battery)

        #TODO what's this line do?
        rospy.on_shutdown(self.shutdown)

        self.command = turtlebot_command(self.command_publisher, self.rate)

        # bool flag to stop commands from running multiple times (see run_turtlebot() below)
        self.first_turn = True
    

    def shutdown(self):
        """
        
        Publishes empty twist command to turtlebot to make it stop moving

        """

        self.command_publisher.publish(Twist())
        rospy.sleep(1)


    def send_messages(self):
        """
        
        Calls the message_creator.py script to create and output a message
        to the console. Contains info on the turtlebot's battery level, 
        odometry, and movement. 

        """

        # publish the twist object to the publisher node we created before on the 'cmd_vel' topic
        self.command_publisher.publish(self.twist)

        # create a message and output it to the console
        message_creator.create_message(self.twist, battery_status.batteryLevel, 
        odometry_status.posX, odometry_status.posY, odometry_status.orient_x, 
        odometry_status.orient_y, odometry_status.orient_z, odometry_status.orient_w)
    

# function that sends velocity commands to turtlebot, calls other functions. Run by main function (see below)
def run_turtlebot():
    """
    
    Sends commands to the turtlebot

    """
    # Initialize an instance of the turtlebot class
    turtle = turtlebot()

    while not rospy.is_shutdown():
        if turtle.first_turn is True:
            turtle.command.move_in_a_square()


        turtle.first_turn = False
        


# main function; executes the run_turtlebot function until we hit control + C
if __name__ == '__main__':
    try:
        run_turtlebot()
    except rospy.ROSInterruptException:
        pass
