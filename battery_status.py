#!/usr/bin/env

# initial value to avoid errors
batteryLevel = 100

# function to get battery life
def battery(data):
    global batteryLevel
    batteryLevel = 100*1.74*(data.percentage-0.986)/(1.225-0.986)
