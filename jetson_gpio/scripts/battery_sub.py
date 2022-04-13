#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Point32
from sensor_msgs.msg import BatteryState
import time
from jetson_gpio.msg import data

#def callback(value):
    
    #print(point.current)
 #   print(value.current)
#    print(value.power_supply_status)
#    print(value.percentage)
def callback1(msg):
    recv_1=msg.data1
    
    print(msg)

def joy_node():    
    rospy.init_node('batterysubscriber')
    
    #rospy.Subscriber('battery',BatteryState,callback)
    #rospy.Subscriber('driver',Int16,callback1)
    rospy.Subscriber('IO',data,callback1)
    rospy.spin()

if __name__ == '__main__':
    joy_node()
