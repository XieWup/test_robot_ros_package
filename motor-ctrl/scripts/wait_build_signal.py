#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
from std_msgs.msg import Int64
import os


def stringSubscriberCallback(data): 
    rospy.loginfo('start building map...')
    #os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation build_map.launch"')
    os.system('gnome-terminal -x bash -c "roslaunch hanning_navigation build_map.launch"')

def stringSubscriber():
    rospy.init_node('subscribe_build_signal_node', anonymous = False) 
    rospy.Subscriber('wait_build_signal', Int64, stringSubscriberCallback) 

    rospy.spin()

if __name__ == '__main__':
    stringSubscriber()
