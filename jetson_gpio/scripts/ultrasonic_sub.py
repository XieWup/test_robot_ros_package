#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

import time


def callback1(msg1):
    
    print(msg1.range)

def callback2(msg2):
    
    print(msg2.range)

def callback3(msg3):
    
    print(msg3.range)

def callback4(msg4):
    
    print(msg4.range)

def joy_node():    
    rospy.init_node('joy')
    
    rospy.Subscriber('ultrasoundpublisher1',Range,callback1)
    rospy.Subscriber('ultrasoundpublisher2',Range,callback2)
    rospy.Subscriber('ultrasoundpublisher3',Range,callback3)
    rospy.Subscriber('ultrasoundpublisher4',Range,callback4)
    rospy.spin()

if __name__ == '__main__':
    joy_node()
